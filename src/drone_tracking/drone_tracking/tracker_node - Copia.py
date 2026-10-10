#!/usr/bin/env python3
"""Filtro di Kalman sulla posizione del bersaglio nell'immagine.

Stato [x, y, vx, vy] in coordinate normalizzate, modello a velocita quasi
costante. Tre scelte che lo distinguono dal caso da manuale:

- il moto del velivolo entra come INGRESSO NOTO, non come rumore di processo.
  La posizione nell'immagine cambia per due motivi, il bersaglio che si muove e
  il velivolo che si muove, e il secondo e misurato da MAVROS: trattarlo come
  rumore costringeva il filtro a spiegare con l'incertezza cio che gia sapeva.
  Con l'ingresso noto la velocita stimata diventa quella propria del bersaglio;
  senza, era la relativa, e chi voleva l'assoluta doveva sommarci quella del
  velivolo ottenendo una stima con errore pari al segnale (10.8 m/s contro 10.0)
- ogni misura passa un test di plausibilita (distanza di Mahalanobis sulla
  covarianza dell'innovazione) prima di essere usata. Il disturbo non e
  gaussiano — il 30% dei messaggi e una perdita totale — e un filtro gaussiano
  senza rifiuto pesa un valore anomalo invece di scartarlo
- il filtro pubblica il proprio NIS, che permette di verificare Q e R senza
  verita a terra: e la sola via percorribile su un velivolo vero
"""
import math
from collections import namedtuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TempoMsg
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Float64, String

from drone_tracking.mission_node import FaseMissione  # type: ignore
from drone_tracking.parametri import parametro  # type: ignore

# Un rilevamento con l'istante a cui si riferisce. L'istante e la ragione
# per cui esiste: il resto era gia nel messaggio.
Misura = namedtuple('Misura', 'x y area istante')

# Semicampo della telecamera, come in controller_node.
TAN_O = 1.0      # orizzontale, 90 gradi
TAN_V = 0.750    # verticale

# Esiti possibili della correzione.
USATA = 'usata'
RIFIUTATA = 'rifiutata'
RIACQUISITO = 'riacquisito'
ASSENTE = 'assente'


class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')

        qos_mavros = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Float32, '/rf/noise_level',
                                 self.on_noise_level, 10)
        self.create_subscription(PointStamped, '/target/jammed_position',
                                 self.on_detection, 10)
        self.create_subscription(Bool, '/tracker/reset', self.on_reset, 10)
        self.create_subscription(String, '/mission/stato',
                                 self.on_stato_missione, 10)

        # Moto del velivolo: e l'ingresso noto della predizione.
        self.create_subscription(TwistStamped,
                                 '/mavros/local_position/velocity_local',
                                 self.on_velocita_drone, qos_mavros)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.on_posa, qos_mavros)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self.on_quota, qos_mavros)

        self.pub = self.create_publisher(
            PointStamped, '/target/tracked_position', 10)
        # Velocita del bersaglio in coordinate immagine al secondo. Con
        # l'ingresso noto attivo e la sua velocita PROPRIA, non piu relativa al
        # velivolo. z vale 1 quando la stima e utilizzabile.
        self.pub_vel = self.create_publisher(
            PointStamped, '/target/tracked_velocity', 10)
        # Innovazione normalizzata: vale 2 in media se Q e R sono coerenti.
        self.pub_nis = self.create_publisher(Float32, '/target/nis', 10)

        self.stato_stimato = np.zeros((4, 1), dtype=np.float32)
        self.incertezza_corrente = np.eye(4, dtype=np.float32)

        # Il nodo e guidato dai messaggi, non da un timer: il ritmo e quello
        # della telecamera, misurato fra 5 e 25 Hz secondo il carico.
        self.dt_nominale = 0.1
        self.dt_min = 0.02
        self.dt_max = 0.5
        self.ultimo_istante = None

        self.evoluzione_stato = np.eye(4, dtype=np.float32)
        self.mappa_osservazione = np.array([[1, 0, 0, 0],
                                            [0, 1, 0, 0]], dtype=np.float32)

        # Intensita del rumore di accelerazione, in (unita normalizzate)^2/s^3.
        # Tarata sull'ERRORE della velocita stimata e non sulla sua distorsione:
        # il valore precedente, 0.5, rendeva la stima non distorta ma rumorosa,
        # con 9.3 m/s di errore contro un bersaglio che ne percorre 10. A 0.01
        # l'errore scende a 3.0, vicino al limite di 2.6 che il rumore di misura
        # e la manovrabilita del bersaglio impongono (scripts/, spazzata
        # offline sul filtro vero).
        self.intensita_rumore_accel = parametro(
            self, 'intensita_rumore_accel', 0.01)

        # R cresce con il rumore dichiarato sul datalink: e il meccanismo con
        # cui il filtro si fida meno della misura quando la misura vale meno.
        #
        # Il valore di base e MISURATO e non scelto: e la varianza della parte
        # bianca dello scarto fra rilevamento e proiezione della posizione vera
        # (`metriche.py rumore`). Lo 0.05 precedente valeva tre volte tanto, e
        # il NIS lo segnalava — un filtro che dichiara piu incertezza di quanta
        # ne abbia corregge meno di quanto potrebbe.
        self.rumore_sensore_base = parametro(self, 'rumore_sensore_base', 0.016)
        self.rumore_sensore_max = parametro(self, 'rumore_sensore_max', 2.0)
        self.livello_rumore = 0.0
        self.incertezza_sensore = (np.eye(2, dtype=np.float32)
                                   * self.rumore_sensore_base)

        # Soglia del test di plausibilita, in unita di chi-quadro a 2 gradi di
        # liberta: 9.21 lascia passare il 99% delle misure legittime. A zero il
        # test e disattivato.
        self.soglia_gating = parametro(self, 'soglia_gating', 9.21)
        # Un filtro che rifiuta tutto diverge in silenzio, convinto di sapere
        # dove sia il bersaglio. Dopo qualche rifiuto di fila la spiegazione
        # piu probabile non e che le misure siano sbagliate ma che lo sia lo
        # stato: si riparte dalla misura.
        self.max_rifiuti = parametro(self, 'max_rifiuti_consecutivi', 5)
        self.rifiuti_consecutivi = 0

        # Quanti fotogrammi senza segnale tollerare continuando a pubblicare la
        # predizione, cosi che il controllo non si fermi a ogni buco.
        self.soglia_perdita = parametro(self, 'soglia_perdita', 15)

        self.bersaglio_acquisito = False
        self.frame_senza_segnale = 0
        self.ultima_area = 0.0

        # Stato del velivolo per l'ingresso noto. Finche non arriva, l'ingresso
        # vale zero e il filtro si comporta come prima.
        self.vel_drone = None
        self.istante_vel_drone = None
        self.yaw = 0.0
        self.quota = None
        self.timeout_stato_s = parametro(self, 'timeout_stato_drone_s', 1.0)
        # Tetto all'estrapolazione in avanti. Oltre, l'errore sulla
        # velocita moltiplicato per il tempo supera il ritardo che si sta
        # correggendo, e la cura fa piu danno del male.
        self.eta_massima_s = parametro(self, 'eta_massima_misura_s', 0.3)
        self.ingresso_applicato = False

        self.get_logger().info('TrackerNode avviato — filtro Kalman attivo')

    # ------------------------------------------------------------ ingressi
    def on_noise_level(self, msg: Float32):
        self.livello_rumore = msg.data
        r = (self.rumore_sensore_base
             + (self.rumore_sensore_max - self.rumore_sensore_base)
             * self.livello_rumore)
        self.incertezza_sensore = np.eye(2, dtype=np.float32) * r

    def on_velocita_drone(self, msg: TwistStamped):
        self.istante_vel_drone = self.get_clock().now().nanoseconds / 1e9
        self.vel_drone = (msg.twist.linear.x, msg.twist.linear.y)

    def on_posa(self, msg: PoseStamped):
        q = msg.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_quota(self, msg: Float64):
        # Stessa sorgente di quota usata dal controllo: la conversione fra
        # coordinate immagine e metri deve essere la stessa nei due nodi.
        self.quota = msg.data

    def on_stato_missione(self, msg: String):
        if FaseMissione.ATTESA.value in msg.data:
            self._reset()

    def on_reset(self, msg: Bool):
        if msg.data:
            self._reset()
            self.get_logger().info('Tracker resettato')

    # -------------------------------------------------------------- modello
    def _matrice_Q(self, dt):
        """Rumore di processo di un modello a velocita quasi costante.

        Un'accelerazione ignota di intensita q su un intervallo dt produce
        varianza q*dt^3/3 sulla posizione, q*dt sulla velocita e covarianza
        q*dt^2/2 fra le due.
        """
        q = self.intensita_rumore_accel
        p, c, v = q * dt ** 3 / 3.0, q * dt ** 2 / 2.0, q * dt
        return np.array([[p, 0, c, 0],
                         [0, p, 0, c],
                         [c, 0, v, 0],
                         [0, c, 0, v]], dtype=np.float32)

    def _ingresso_noto(self, dt):
        """Spostamento d'immagine dovuto al moto del VELIVOLO in dt.

        Un bersaglio fermo scivola nell'immagine quando il velivolo trasla, e
        di quanto e calcolabile: la traslazione va portata in assi velivolo con
        l'imbardata e divisa per l'impronta a terra, che vale quota per la
        tangente del semicampo. Restituisce (dx, dy) in unita normalizzate,
        oppure None se il moto del velivolo non e noto o la quota non e
        utilizzabile: in quel caso il
        filtro torna al comportamento precedente, e la velocita di stato torna
        a essere relativa invece che propria del bersaglio.
        """
        if self.vel_drone is None or self.quota is None or self.quota < 5.0:
            return None
        adesso = self.get_clock().now().nanoseconds / 1e9
        if (self.istante_vel_drone is None
                or adesso - self.istante_vel_drone > self.timeout_stato_s):
            return None

        c, s = math.cos(self.yaw), math.sin(self.yaw)
        avanti = (self.vel_drone[0] * c + self.vel_drone[1] * s) * dt
        laterale = (-self.vel_drone[0] * s + self.vel_drone[1] * c) * dt
        # Segni: il velivolo che avanza spinge il bersaglio verso il fondo
        # dell'inquadratura, dove y e positivo.
        return (laterale / (self.quota * TAN_O),
                avanti / (self.quota * TAN_V))

    def _calcola_dt(self, adesso=None):
        """Intervallo fra due misure, misurato sugli istanti che dichiarano.

        Non sull'ora di arrivo: quella include il tempo di trasporto, che
        varia, e attribuisce alla dinamica del bersaglio il ritardo della
        catena.
        """
        if adesso is None:
            adesso = self.get_clock().now().nanoseconds / 1e9
        if self.ultimo_istante is None:
            self.ultimo_istante = adesso
            return self.dt_nominale
        dt = adesso - self.ultimo_istante
        self.ultimo_istante = adesso
        return float(min(max(dt, self.dt_min), self.dt_max))

    # ------------------------------------------------------------- uscite
    def _stamp(self, istante):
        return TempoMsg(sec=int(istante),
                        nanosec=int((istante - int(istante)) * 1e9))

    def _eta(self, istante):
        """Quanto e vecchia la misura, con un tetto.

        Il tetto serve perche l'estrapolazione moltiplica l'errore sulla
        velocita per il tempo: oltre una frazione di secondo si starebbe
        inventando piu di quanto si corregge.
        """
        eta = self.get_clock().now().nanoseconds / 1e9 - istante
        return float(min(max(eta, 0.0), self.eta_massima_s))

    def _pubblica_posizione(self, area, istante=None):
        """Pubblica dove il bersaglio e ADESSO, non dov'era quando l'ho visto.

        Lo stato si riferisce all'istante della misura. Portarlo avanti
        dell'eta della misura toglie il ritardo della catena di percezione,
        che altrimenti entra nell'anello di controllo come sfasamento.
        """
        x = float(self.stato_stimato[0].item())
        y = float(self.stato_stimato[1].item())
        riferimento = self.get_clock().now().nanoseconds / 1e9
        if istante is not None:
            eta = self._eta(istante)
            riferimento = istante + eta
            x += float(self.stato_stimato[2].item()) * eta
            y += float(self.stato_stimato[3].item()) * eta
            # Anche il velivolo si e mosso nel frattempo, e il suo moto
            # trasla l'inquadratura: e lo stesso termine noto della
            # predizione, sullo stesso intervallo.
            ingresso = self._ingresso_noto(eta)
            if ingresso is not None:
                x += ingresso[0]
                y += ingresso[1]
        msg = PointStamped()
        msg.header.stamp = self._stamp(riferimento)
        msg.point.x, msg.point.y, msg.point.z = x, y, float(area)
        self.pub.publish(msg)

    def _pubblica_vuoto(self, istante=None):
        """Nessun bersaglio. Marcato comunque: dice quando si e guardato."""
        msg = PointStamped()
        msg.header.stamp = self._stamp(
            istante if istante is not None
            else self.get_clock().now().nanoseconds / 1e9)
        self.pub.publish(msg)

    def _pubblica_velocita(self, valida, istante=None):
        stima = PointStamped()
        stima.header.stamp = self._stamp(
            istante if istante is not None
            else self.get_clock().now().nanoseconds / 1e9)
        if valida:
            stima.point.x = float(self.stato_stimato[2].item())
            stima.point.y = float(self.stato_stimato[3].item())
            stima.point.z = 1.0
        self.pub_vel.publish(stima)

    def _reset(self):
        self.bersaglio_acquisito = False
        self.frame_senza_segnale = 0
        self.rifiuti_consecutivi = 0
        self.stato_stimato = np.zeros((4, 1), dtype=np.float32)
        self.incertezza_corrente = np.eye(4, dtype=np.float32)
        self.ultima_area = 0.0
        # Alla ripresa il primo dt ripartirebbe dal tempo trascorso durante la
        # perdita, che non e un intervallo di campionamento valido.
        self.ultimo_istante = None
        self._pubblica_velocita(False)

    def _leggi(self, msg):
        """Dal messaggio alla misura, con l'istante a cui si riferisce.

        L'istante e quello dell'otturatore, propagato dal rilevatore
        attraverso il jammer. Se manca — qualcuno pubblica senza marcatura —
        si ripiega sull'ora d'arrivo, che e esattamente l'approssimazione che
        questo nodo ha smesso di fare: meglio dichiararla che subirla.
        """
        istante = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if istante <= 0.0:
            istante = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().warn(
                'Rilevamento senza marcatura: uso l ora d arrivo',
                throttle_duration_sec=10.0)
        return Misura(msg.point.x, msg.point.y, msg.point.z, istante)

    def _acquisisci(self, m):
        """Fa ripartire lo stato dalla misura. Non pubblica: lo fa il chiamante."""
        self.stato_stimato = np.array([[m.x], [m.y], [0.0], [0.0]],
                                      dtype=np.float32)
        self.incertezza_corrente = np.eye(4, dtype=np.float32)
        self.bersaglio_acquisito = True
        self.rifiuti_consecutivi = 0
        self.frame_senza_segnale = 0
        self.ultima_area = m.area
        self._calcola_dt(m.istante)

    # ---------------------------------------------------------- ciclo del filtro
    def on_detection(self, msg: PointStamped):
        m = self._leggi(msg)
        segnale_presente = m.area != 0.0

        if not self.bersaglio_acquisito:
            if segnale_presente:
                self._acquisisci(m)
                self.get_logger().info('Bersaglio acquisito')
                # Pubblicare subito evita di perdere un messaggio a ogni
                # riacquisizione, e sotto jamming sono continue. La velocita
                # non e ancora stimata, e dirlo evita che il controllo usi uno
                # zero come se fosse una misura.
                self._pubblica_posizione(m.area, m.istante)
                self._pubblica_velocita(False, m.istante)
            else:
                self._pubblica_vuoto(m.istante)
            return

        dt = self._calcola_dt(m.istante)
        self.evoluzione_stato[0, 2] = dt
        self.evoluzione_stato[1, 3] = dt

        self.stato_stimato = self.evoluzione_stato @ self.stato_stimato
        ingresso = self._ingresso_noto(dt)
        self.ingresso_applicato = ingresso is not None
        if ingresso is not None:
            self.stato_stimato[0] += ingresso[0]
            self.stato_stimato[1] += ingresso[1]
        self.incertezza_corrente = (
            self.evoluzione_stato @ self.incertezza_corrente
            @ self.evoluzione_stato.T + self._matrice_Q(dt))

        esito = self._aggiorna(m) if segnale_presente else ASSENTE

        if esito == USATA:
            self.ultima_area = m.area
            self.frame_senza_segnale = 0
            self._pubblica_posizione(m.area, m.istante)
            # Senza ingresso noto la velocita di stato e relativa al velivolo e
            # non propria del bersaglio: e una grandezza diversa, e chi la legge
            # non ha modo di accorgersene. Meglio non pubblicarla.
            self._pubblica_velocita(self.ingresso_applicato, m.istante)
            return

        if esito == RIACQUISITO:
            # Lo stato riparte dalla misura: la posizione e buona, la velocita
            # non ancora.
            self._pubblica_posizione(m.area, m.istante)
            self._pubblica_velocita(False, m.istante)
            return

        # Nessuna misura utilizzabile: o non e arrivata, o non era plausibile.
        self.frame_senza_segnale += 1
        if self.frame_senza_segnale > self.soglia_perdita:
            self._reset()
            self.get_logger().warn('Bersaglio perso — reset tracker')
            self._pubblica_vuoto(m.istante)
            return

        # La POSIZIONE predetta e utilizzabile e va pubblicata: e cio che
        # tollera le micro-interruzioni. La VELOCITA no: durante la predizione
        # non arriva informazione nuova sul moto, e lo stato resta congelato
        # all'ultimo valore. Pubblicarlo come valido significa spacciare per
        # misure ripetute una sola misura ripetuta molte volte.
        self._pubblica_posizione(self.ultima_area, m.istante)
        self._pubblica_velocita(False, m.istante)

    def _aggiorna(self, m):
        """Correzione con la misura. Torna USATA, RIFIUTATA o RIACQUISITO."""
        misura = np.array([[m.x], [m.y]], dtype=np.float32)
        innovazione = misura - self.mappa_osservazione @ self.stato_stimato
        S = (self.mappa_osservazione @ self.incertezza_corrente
             @ self.mappa_osservazione.T + self.incertezza_sensore)
        S_inv = np.linalg.inv(S)

        # Innovazione normalizzata: quanto la misura sorprende il filtro,
        # misurata nell'incertezza che il filtro stesso dichiara. Con due gradi
        # di liberta vale 2 in media se Q e R sono coerenti con la realta, ed e
        # la grandezza con cui si tarano senza verita a terra.
        nis = float((innovazione.T @ S_inv @ innovazione).item())
        self.pub_nis.publish(Float32(data=nis))

        if 0.0 < self.soglia_gating < nis:
            self.rifiuti_consecutivi += 1
            if self.rifiuti_consecutivi <= self.max_rifiuti:
                return RIFIUTATA
            # Troppi rifiuti di fila: a sbagliare e lo stato, non le misure.
            self.get_logger().warn(
                'Misure rifiutate {} volte di fila — riacquisisco'.format(
                    self.rifiuti_consecutivi))
            self._acquisisci(m)
            return RIACQUISITO

        self.rifiuti_consecutivi = 0
        guadagno = self.incertezza_corrente @ self.mappa_osservazione.T @ S_inv
        self.stato_stimato = self.stato_stimato + guadagno @ innovazione
        self.incertezza_corrente = (
            (np.eye(4, dtype=np.float32)
             - guadagno @ self.mappa_osservazione) @ self.incertezza_corrente)
        return USATA


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
