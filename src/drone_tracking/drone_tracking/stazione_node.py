#!/usr/bin/env python3
"""Stazione di controllo a terra: allerta dai sensori e lancio del drone.

Lo scenario e quello degli intercettori in uso in Ucraina: una rete di sensori
a terra (radar, osservatori, reti acustiche) rileva il bersaglio e manda una
posizione approssimativa a chi deve intervenire. Li l'intercettore viene poi
pilotato a mano; qui la stazione lo fa decollare da sola e la missione lo porta
a cercare il bersaglio dove l'allerta dice.

Il sensore e SIMULATO a partire dalla posizione vera del bersaglio letta da
Gazebo, e ne riproduce i difetti che contano per chi lo usa:
  errore      la posizione ha un errore casuale di decine di metri;
  ritardo     arriva dopo l'istante a cui si riferisce;
  cadenza     arriva ogni pochi secondi, non a flusso continuo;
  perdite     una parte delle allerte non arriva;
  portata     oltre una certa distanza dalla stazione non vede nulla.
I valori di default sono IPOTESI ragionevoli, non misure di un sensore vero:
sono parametri, e vanno dichiarati come tali.

Topic pubblicati:
  /stazione/allerta  PoseWithCovarianceStamped: posizione stimata, con la
                     varianza dell'errore e l'istante a cui si RIFERISCE (non
                     quello di invio: chi la usa deve sapere quanto e vecchia);
  /stazione/stima    Odometry: ultima allerta piu velocita stimata dalle
                     allerte recenti, con la convenzione di mission_node
                     (twist.covariance[0] > 0.5 = velocita valida);
  /stazione/stato    String: IN_ASCOLTO, CONFERMA, LANCIO, IN_VOLO, ERRORE.

Al lancio la stazione fa quello che fa takeoff.sh (GUIDED, arming con
ritentativi, decollo, verifica della quota) e poi avvia la missione su
/mission/avvia.
"""
from collections import deque
import math
import random
import threading

from drone_tracking.mission_node import FaseMissione  # type: ignore
from drone_tracking.parametri import parametro  # type: ignore
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    GZ_BINDINGS = True
except ImportError:
    GZ_BINDINGS = False


class ModelloSensore:
    """La rete di sensori a terra, ridotta a cio che vede chi la usa.

    Separato dal nodo ROS perche lo si possa provare senza simulatore: riceve
    la verita con il suo istante e restituisce, a richiesta, l'allerta che un
    sensore con questi difetti produrrebbe in quel momento.
    """

    def __init__(self, sigma_m, ritardo_s, prob_perdita, portata_m,
                 stazione=(0.0, 0.0), seme=None, memoria_s=30.0):
        self.sigma_m = sigma_m
        self.ritardo_s = ritardo_s
        self.prob_perdita = prob_perdita
        self.portata_m = portata_m
        self.stazione = stazione
        self.memoria_s = memoria_s
        self.casuale = random.Random(seme)
        self.storia = deque()   # (istante, x, y), in ordine di tempo

    def registra_verita(self, istante, x, y):
        self.storia.append((istante, x, y))
        while self.storia and istante - self.storia[0][0] > self.memoria_s:
            self.storia.popleft()

    def verita_a(self, istante):
        """Posizione vera all'istante dato, interpolata; None se non coperto.

        L'interpolazione serve perche la verita arriva a 10 Hz mentre il
        ritardo e un numero qualunque: prendere il campione piu vicino
        aggiungerebbe un errore di quantizzazione che il sensore vero non ha.
        """
        if not self.storia or istante < self.storia[0][0]:
            return None
        precedente = None
        for campione in self.storia:
            if campione[0] >= istante:
                if precedente is None:
                    return campione[1], campione[2]
                t0, x0, y0 = precedente
                t1, x1, y1 = campione
                a = (istante - t0) / (t1 - t0) if t1 > t0 else 0.0
                return x0 + a * (x1 - x0), y0 + a * (y1 - y0)
            precedente = campione
        # Istante piu recente dell'ultima verita: la si usa se e fresca, cioe
        # se il ritardo con cui la si legge e quello voluto e non un buco.
        t, x, y = self.storia[-1]
        return (x, y) if istante - t < 1.0 else None

    def allerta(self, adesso):
        """L'allerta disponibile ora: (istante_riferimento, x, y) o None.

        None quando il bersaglio e fuori portata, quando l'allerta va persa o
        quando non c'e ancora una verita abbastanza vecchia da ritardare.
        """
        istante = adesso - self.ritardo_s
        vera = self.verita_a(istante)
        if vera is None:
            return None
        dx, dy = vera[0] - self.stazione[0], vera[1] - self.stazione[1]
        if math.hypot(dx, dy) > self.portata_m:
            return None
        if self.casuale.random() < self.prob_perdita:
            return None
        return (istante,
                vera[0] + self.casuale.gauss(0.0, self.sigma_m),
                vera[1] + self.casuale.gauss(0.0, self.sigma_m))


def stima_velocita(allerte, minimo=3):
    """Velocita dal moto delle allerte recenti, con i minimi quadrati.

    Con due sole allerte a pochi secondi di distanza e un errore di decine di
    metri, la differenza finita darebbe velocita di decine di m/s anche a un
    bersaglio fermo: la retta interpolata su piu punti media l'errore.
    Restituisce (vx, vy) o None se i punti non bastano.
    """
    if len(allerte) < minimo:
        return None
    t = [a[0] for a in allerte]
    tm = sum(t) / len(t)
    den = sum((ti - tm) ** 2 for ti in t)
    if den < 1e-6:
        return None
    vel = []
    for indice in (1, 2):
        v = [a[indice] for a in allerte]
        vm = sum(v) / len(v)
        vel.append(sum((ti - tm) * (vi - vm) for ti, vi in zip(t, v)) / den)
    return vel[0], vel[1]


def velocita_significativa(allerte, sigma_m, soglia=2.0, minimo=3):
    """La velocita stimata, solo se si distingue dal rumore.

    Con errore `sigma_m` su ogni allerta, la pendenza della retta ha
    deviazione standard sigma_m / sqrt(sum (t - t_medio)^2). Sotto `soglia`
    volte quel valore la velocita non e distinguibile da zero: usarla per
    anticipare il bersaglio sposterebbe il punto d'incontro di decine di metri
    nella direzione del rumore. Nella prima prova un bersaglio fermo era stato
    anticipato di quasi 50 m.

    Restituisce (vx, vy, sigma_v), oppure None se non significativa.
    """
    velocita = stima_velocita(allerte, minimo)
    if velocita is None:
        return None
    t = [a[0] for a in allerte]
    tm = sum(t) / len(t)
    sigma_v = sigma_m / math.sqrt(sum((ti - tm) ** 2 for ti in t))
    if math.hypot(*velocita) <= soglia * sigma_v:
        return None
    return velocita[0], velocita[1], sigma_v


class StazioneNode(Node):
    def __init__(self):
        super().__init__('stazione_node')

        # Il periodo fissa il timer alla creazione: cambiarlo a caldo non ha
        # effetto. Gli altri valori del sensore si leggono anch'essi all'avvio.
        self.periodo_s = parametro(self, 'periodo_allerta_s', 2.0, 'periodo_s')
        self.allerte_per_lancio = parametro(self, 'allerte_per_lancio', 2)
        self.lancio_automatico = parametro(self, 'lancio_automatico', True)
        self.quota_decollo = parametro(self, 'quota_decollo_m', 50.0, 'quota_decollo')
        # Dieci e non cinque come in takeoff.sh: la stazione lancia pochi
        # secondi dopo l'avvio della simulazione, quando gli IMU simulati sono
        # ancora incoerenti. Nella prima prova l'arming e passato al quinto.
        self.tentativi_arming_max = parametro(self, 'tentativi_arming_max', 10)
        self.punti_velocita = parametro(self, 'punti_velocita', 5)
        self.soglia_significativita = parametro(self, 'soglia_significativita', 2.0)
        self.mondo = parametro(self, 'mondo', 'iris_runway')
        self.nome_bersaglio = parametro(self, 'modello_bersaglio', 'bersaglio',
                                        'nome_bersaglio')
        seme = parametro(self, 'seme', 42)
        self.sensore = ModelloSensore(
            sigma_m=parametro(self, 'errore_sigma_m', 20.0),
            ritardo_s=parametro(self, 'ritardo_s', 1.0),
            prob_perdita=parametro(self, 'prob_perdita', 0.1),
            # Piu ampia della zona in cui il bersaglio si muove: con 400 m la
            # sua fuga lo portava fino a 514 m e la stazione lo perdeva.
            portata_m=parametro(self, 'portata_m', 600.0),
            seme=None if seme < 0 else seme)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_allerta = self.create_publisher(
            PoseWithCovarianceStamped, '/stazione/allerta', 10)
        self.pub_stima = self.create_publisher(Odometry, '/stazione/stima', 10)
        self.pub_stato = self.create_publisher(String, '/stazione/stato', 10)
        self.pub_avvia = self.create_publisher(Bool, '/mission/avvia', 10)

        self.stato_volo = None
        self.quota = None
        self.fase_missione = None
        self.blocco_sicurezza = False
        self.create_subscription(State, '/mavros/state', self.su_stato_volo, qos)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self.su_quota, qos)
        self.create_subscription(String, '/mission/stato', self.su_missione, 10)
        self.create_subscription(Bool, '/sicurezza/blocco', self.su_blocco, 10)

        self.cli_modo = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_arming = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_decollo = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.allerte = deque(maxlen=max(int(self.punti_velocita), 2))
        self.allerte_consecutive = 0
        self.fase = 'IN_ASCOLTO'
        self.passo_lancio = None
        self.richiesta = None
        self.tentativi_arming = 0
        self.prossimo_arming = 0.0
        self.istante_passo = None
        self.istante_prima_allerta = None
        self.istante_decollo = None

        # La verita arriva su un thread di gz-transport: la si deposita sotto
        # lucchetto e la si consuma nel timer ROS.
        self.lucchetto = threading.Lock()
        self.ultima_verita = None
        self.gz_node = None
        if GZ_BINDINGS:
            topic = '/world/{}/pose/info'.format(self.mondo)
            try:
                self.gz_node = GzNode()
                if not self.gz_node.subscribe(Pose_V, topic, self.su_pose_info):
                    raise RuntimeError('sottoscrizione rifiutata')
                self.get_logger().info('Verita a terra da {}'.format(topic))
            except Exception as e:                  # noqa: BLE001
                self.gz_node = None
                self.get_logger().error(
                    'Nessuna verita da {} ({}): la stazione non vedra nulla'
                    .format(topic, e))
        else:
            self.get_logger().error(
                'python3-gz-transport13 non disponibile: la stazione non '
                'puo simulare il sensore')

        self.create_timer(0.1, self.registra)
        self.create_timer(self.periodo_s, self.ciclo_allerta)
        self.create_timer(0.5, self.ciclo_lancio)
        self.get_logger().info(
            'Stazione avviata — allerta ogni {:.1f} s, errore {:.0f} m, '
            'ritardo {:.1f} s, perdite {:.0%}, portata {:.0f} m{}'.format(
                self.periodo_s, self.sensore.sigma_m, self.sensore.ritardo_s,
                self.sensore.prob_perdita, self.sensore.portata_m,
                '' if self.lancio_automatico else ', lancio MANUALE'))

    # ------------------------------------------------------------ ingressi
    def su_pose_info(self, msg):
        for posa in msg.pose:
            if posa.name == self.nome_bersaglio:
                with self.lucchetto:
                    self.ultima_verita = (posa.position.x, posa.position.y)
                return

    def su_stato_volo(self, msg):
        self.stato_volo = msg

    def su_quota(self, msg):
        self.quota = msg.data

    def su_missione(self, msg):
        self.fase_missione = msg.data

    def su_blocco(self, msg):
        self.blocco_sicurezza = msg.data

    def adesso(self):
        return self.get_clock().now().nanoseconds / 1e9

    def registra(self):
        """Deposita la verita nella storia del sensore, con l'orologio ROS.

        L'istante e preso qui e non nel callback di Gazebo perche tutta la
        catena ragiona in tempo di simulazione: con la simulazione a un quarto
        del tempo reale, mescolare i due orologi sbaglierebbe il ritardo di un
        fattore quattro.
        """
        with self.lucchetto:
            verita = self.ultima_verita
        if verita is not None:
            self.sensore.registra_verita(self.adesso(), verita[0], verita[1])

    def pubblica_stato(self):
        self.pub_stato.publish(String(data=self.fase))

    # ------------------------------------------------------------- allerta
    def ciclo_allerta(self):
        adesso = self.adesso()
        allerta = self.sensore.allerta(adesso)
        if allerta is None:
            self.allerte_consecutive = 0
            self.pubblica_stato()
            return
        istante, x, y = allerta
        self.allerte.append(allerta)
        self.allerte_consecutive += 1
        if self.istante_prima_allerta is None:
            self.istante_prima_allerta = adesso

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp.sec = int(istante)
        msg.header.stamp.nanosec = int((istante - int(istante)) * 1e9)
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        varianza = self.sensore.sigma_m ** 2
        msg.pose.covariance[0] = varianza
        msg.pose.covariance[7] = varianza
        self.pub_allerta.publish(msg)

        stima = Odometry()
        stima.header = msg.header
        stima.pose.pose = msg.pose.pose
        significativa = velocita_significativa(
            list(self.allerte), self.sensore.sigma_m, self.soglia_significativita)
        velocita = None
        if significativa is not None:
            velocita = significativa[:2]
            stima.twist.twist.linear.x, stima.twist.twist.linear.y = velocita
            stima.twist.covariance[0] = 1.0
            # L'incertezza viaggia con la stima: chi la usa puo pesarla.
            stima.twist.covariance[7] = significativa[2] ** 2
        self.pub_stima.publish(stima)

        if self.fase == 'IN_ASCOLTO':
            self.fase = 'CONFERMA'
        # Lo scarto dalla verita si puo misurare solo in simulazione, ed e il
        # numero che dice quanto vale davvero l'indicazione data al drone.
        vera = self.sensore.verita_a(istante)
        scarto = (math.hypot(x - vera[0], y - vera[1]) if vera is not None
                  else float('nan'))
        self.get_logger().info(
            'Allerta: bersaglio a ({:.0f}, {:.0f}) di {:.1f} s fa, scarto dal vero '
            '{:.0f} m{}'.format(
                x, y, adesso - istante, scarto,
                '' if velocita is None else
                ', velocita ({:+.1f}, {:+.1f}) m/s'.format(*velocita)))
        self.pubblica_stato()

    # --------------------------------------------------------------- lancio
    def pronto_al_lancio(self):
        return (self.lancio_automatico
                and self.fase == 'CONFERMA'
                and self.allerte_consecutive >= self.allerte_per_lancio
                and not self.blocco_sicurezza
                and self.stato_volo is not None and self.stato_volo.connected
                and self.fase_missione is not None
                and FaseMissione.ATTESA.value in self.fase_missione)

    def chiama(self, client, richiesta):
        if not client.service_is_ready():
            return None
        return client.call_async(richiesta)

    def esito(self):
        """True/False quando la richiesta in corso ha risposto, None se no."""
        if self.richiesta is None or not self.richiesta.done():
            return None
        risposta = self.richiesta.result()
        self.richiesta = None
        if risposta is None:
            return False
        # SetMode risponde con mode_sent, CommandBool e CommandTOL con success.
        return bool(getattr(risposta, 'mode_sent', getattr(risposta, 'success', False)))

    def passa(self, passo):
        self.passo_lancio = passo
        self.istante_passo = self.adesso()

    def ciclo_lancio(self):
        if self.fase == 'CONFERMA' and self.pronto_al_lancio():
            self.fase = 'LANCIO'
            self.istante_decollo = None
            self.tentativi_arming = 0
            self.get_logger().warn(
                'LANCIO dopo {} allerte consecutive'.format(self.allerte_consecutive))
            if self.stato_volo.armed and (self.quota or 0.0) > 5.0:
                self.passa('avvia_missione')   # gia in aria: niente decollo
            else:
                self.passa('modo')
        if self.fase != 'LANCIO':
            return
        trascorso = self.adesso() - self.istante_passo

        if self.passo_lancio == 'modo':
            if self.richiesta is None:
                self.richiesta = self.chiama(
                    self.cli_modo, SetMode.Request(custom_mode='GUIDED'))
                return
            ok = self.esito()
            if ok:
                self.passa('arming')
                self.prossimo_arming = self.adesso() + 2.0   # come takeoff.sh
            elif ok is False:
                self.fallisci('cambio di modo in GUIDED rifiutato')

        elif self.passo_lancio == 'arming':
            # Stessa pausa di takeoff.sh fra un tentativo e l'altro: subito
            # dopo l'avvio l'arming viene rifiutato con "Gyros inconsistent"
            # finche gli IMU simulati non si assestano.
            if self.richiesta is None and self.adesso() >= self.prossimo_arming:
                self.tentativi_arming += 1
                self.richiesta = self.chiama(
                    self.cli_arming, CommandBool.Request(value=True))
                return
            ok = self.esito()
            if ok:
                self.passa('decollo')
            elif ok is False:
                if self.tentativi_arming >= self.tentativi_arming_max:
                    self.fallisci('arming rifiutato {} volte'.format(self.tentativi_arming))
                else:
                    self.get_logger().warn('Arming rifiutato (tentativo {}), riprovo fra 3 s'
                                           .format(self.tentativi_arming))
                    self.prossimo_arming = self.adesso() + 3.0

        elif self.passo_lancio == 'decollo':
            if self.richiesta is None:
                self.richiesta = self.chiama(self.cli_decollo, CommandTOL.Request(
                    altitude=float(self.quota_decollo)))
                return
            # Una conferma mancata non e un decollo mancato: su una macchina
            # lenta la risposta puo non arrivare in tempo mentre il drone sale.
            # Fa fede la quota, come in takeoff.sh.
            if self.esito() is not None:
                self.passa('salita')

        elif self.passo_lancio == 'salita':
            if self.quota is not None and self.quota > 0.8 * self.quota_decollo:
                self.istante_decollo = self.adesso()
                self.passa('avvia_missione')
            elif trascorso > 180.0:
                self.fallisci('quota ferma a {} m'.format(self.quota))

        elif self.passo_lancio == 'avvia_missione':
            # Ripetuto finche la missione non lascia ATTESA: un messaggio solo
            # puo perdersi se mission_node non e ancora sottoscritto.
            if self.fase_missione and FaseMissione.ATTESA.value not in self.fase_missione:
                self.fase = 'IN_VOLO'
                self.get_logger().warn(
                    'In volo e missione avviata: {:.1f} s dalla prima allerta'
                    .format(self.adesso() - self.istante_prima_allerta))
            else:
                self.pub_avvia.publish(Bool(data=True))
        self.pubblica_stato()

    def fallisci(self, motivo):
        self.fase = 'ERRORE'
        self.get_logger().error('Lancio fallito: {}'.format(motivo))
        self.pubblica_stato()


def main(args=None):
    rclpy.init(args=args)
    nodo = StazioneNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
