#!/usr/bin/env python3
import math
from statistics import median
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from drone_tracking.mission_node import FaseMissione  # type: ignore
from drone_tracking.parametri import parametro  # type: ignore

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        qos_mavros = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.alt_sub = self.create_subscription(
            Float64, '/mavros/global_position/rel_alt',
            self.on_altitudine, qos_mavros)

        self.mission_sub = self.create_subscription(
            String, '/mission/stato',
            self.on_stato_missione, 10)

        self.sub = self.create_subscription(
            Point, '/target/tracked_position', self.on_tracked, 10)

        # Velocita del bersaglio stimata dal filtro, in coordinate immagine al
        # secondo e relativa al drone.
        self.create_subscription(
            Point, '/target/tracked_velocity', self.on_velocita_stimata, 10)

        self.gps_sub = self.create_subscription(
            Bool, '/gps/jammed', self.on_gps_status, 10)

        # Angoli comandati alla sospensione cardanica. Servono a sapere quanta
        # parte dell'assetto e gia compensata meccanicamente: la compensazione
        # analitica deve occuparsi solo del residuo. Se il gimbal non c'e,
        # nessuno pubblica su questi topic, i valori restano zero e il calcolo
        # torna identico a quello precedente.
        self.create_subscription(
            Float64, '/gimbal/roll/cmd_pos', self.on_gimbal_roll, 10)
        self.create_subscription(
            Float64, '/gimbal/pitch/cmd_pos', self.on_gimbal_pitch, 10)

        # Serve lo yaw per convertire i comandi dal frame del drone a quello del
        # mondo: vedi la nota in on_tracked.
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.on_posa, qos_mavros)

        # Velocita del velivolo nel frame locale ENU. Serve a ricostruire la
        # velocita assoluta del bersaglio: quella stimata dal filtro e
        # relativa, e senza questo termine il comando inseguirebbe una
        # grandezza che dipende anche dal proprio moto.
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self.on_velocita_drone, qos_mavros)

        self.cmd_pub = self.create_publisher(
            Twist, '/drone/cmd_vel', 10)

        self.mavros_vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Stima del bersaglio nel frame del mondo: posizione e velocita in
        # metri, non piu in coordinate immagine. Il controllo le calcola gia
        # per il proprio comando, e pubblicarle evita che mission_node debba
        # rifare la stessa conversione — due copie della stessa formula
        # divergono al primo che ne corregge una sola.
        self.stima_pub = self.create_publisher(
            Odometry, '/target/odometria', 10)

        self.gps_jammed     = False
        self.altitudine     = 0.0
        self.in_volo        = False
        self.fase_missione  = FaseMissione.ATTESA.value
        self.yaw            = 0.0
        self.pitch          = 0.0
        self.roll           = 0.0
        self.gimbal_roll    = 0.0
        self.gimbal_pitch   = 0.0
        self.posizione_drone = None

        # --- Guida predittiva ---
        # Con k = 1 il comando conterrebbe per intero la velocita stimata del
        # bersaglio: a regime il drone la pareggerebbe invece di inseguirla, e
        # l'errore residuo velocita/kp si annullerebbe. In teoria.
        #
        # Misurato, non paga. Contro un bersaglio in fuga a 5.5 m/s, con la
        # stima del filtro gia corretta in scala:
        #     k = 0.0  ->  84.0% di fotogrammi con bersaglio, mediana 3.92 m
        #     k = 0.4  ->  77.6%                              mediana 4.21 m
        #     k = 0.7  ->  82.0%                              mediana 4.06 m
        #     k = 1.0  ->  74.0%                              mediana 6.42 m
        # Lo zero e il punto migliore e i valori intermedi si equivalgono entro
        # la dispersione fra prove ripetute.
        #
        # La ragione e la qualita della stima, non il termine in se: la
        # correlazione fra velocita stimata e velocita vera del bersaglio vale
        # circa 0.6, quindi poco piu di un terzo della varianza della stima e
        # segnale. Un termine di anticipo somma l'intera stima al comando di
        # velocita, rumore compreso, e quel rumore costa piu del ritardo che
        # elimina. Per renderlo conveniente serve una stima migliore, non un
        # guadagno diverso: la via naturale e dare al filtro la velocita del
        # velivolo come ingresso noto, cosi che stimi direttamente la velocita
        # assoluta del bersaglio invece di ricavarla per differenza.
        #
        # Il termine resta disponibile e parametrico, spento per default.
        self.k_anticipo = parametro(self, 'k_anticipo', 0.0)

        # Finestra su cui si prende la mediana della velocita stimata. Un
        # secondo a ~25 Hz sono venticinque campioni: abbastanza perche una
        # direzione che si inverte fra un fotogramma e il successivo non
        # sopravviva, abbastanza pochi perche un moto vero non venga spianato.
        self.finestra_velocita_s = parametro(self, 'finestra_velocita_s', 1.0)
        self.campioni_velocita_min = parametro(
            self, 'campioni_velocita_min', 5)
        # Limite fisico, non una taratura: nessun veicolo terrestre di questo
        # scenario supera i 25 m/s, cioe 90 km/h. Una stima che lo supera e
        # impossibile e va rifiutata.
        self.vel_bersaglio_max = parametro(self, 'vel_bersaglio_max', 25.0)
        # Per quanto una velocita stabilita da misure vere resta utilizzabile
        # dopo che le misure sono cessate. Serve perche i fotogrammi che
        # precedono la perdita sono predizioni, che non alimentano la finestra:
        # senza memoria la velocita risulterebbe ignota proprio alla perdita,
        # cioe nell'unico istante in cui la ricerca deve usarla.
        self.validita_velocita_s = parametro(self, 'validita_velocita_s', 2.0)
        # Campioni recenti: (istante, avanti, laterale).
        self.finestra_velocita = []
        # Ultimo valore ben stabilito: (avanti, laterale, istante).
        self.ultima_velocita_valida = None
        self.vel_stimata_x = 0.0
        self.vel_stimata_y = 0.0
        self.vel_stimata_valida = False
        self.vel_drone_x = 0.0
        self.vel_drone_y = 0.0
        self.istante_vel_drone = None

        # Compensazione d'assetto. La telecamera è solidale al corpo: una
        # rotazione del velivolo trasla l'immagine indipendentemente da dove si
        # trovi il bersaglio. Con FOV orizzontale 1.047 rad su 640x480, i
        # semicampi valgono 0.524 rad in orizzontale e 0.408 in verticale, quindi
        # un radiante di assetto vale 1/0.408 ≈ 2.45 unità normalizzate: bastano
        # 10° di pitch per spostare il bersaglio di mezzo campo.
        # Misurato senza compensazione: correlazione r = -0.665 fra pitch ed
        # errore verticale, con l'errore che spazzava l'intero campo visivo
        # mentre il bersaglio era pressoché fermo.

        # Guadagni PD espressi in 1/s: agiscono sullo scostamento del bersaglio
        # in METRI, non sulle coordinate normalizzate dell'immagine. Quelle
        # cambiano significato al variare di quota e campo visivo, quindi una
        # taratura fatta su di esse va rifatta a ogni modifica dell'ottica.
        #
        # Dimensionati per un bersaglio a 8.3 m/s (30 km/h). L'errore a regime di
        # un controllo proporzionale vale velocita/kp, e va confrontato con la
        # semi-impronta a terra, che a 12 m di quota con FOV 90 gradi misura 12 m:
        #   kp 1.2 -> 6.9 m (58% del semicampo, troppo vicino al bordo)
        #   kp 2.0 -> 4.2 m (35%, margine sufficiente anche nei transitori)
        # Nota: scendere sotto 1.2 e stato provato e peggiora molto (a 0.6 la
        # distanza mediana dal bersaglio passava da 3.5 a 26.7 m): con guadagno
        # basso il drone non tiene il passo.
        # Alzati a 2.0 con lo scenario realistico. Il guadagno era tenuto
        # basso perche piu aggressivita significava piu inclinazione e quindi
        # meno campo utile; con la sospensione cardanica quel conflitto non
        # esiste piu, ed e stato misurato: a 5.5 m/s di fuga, con telecamera
        # fissa il guadagno 2.0 faceva scendere il rilevamento dal 73% al 53%,
        # con la sospensione lo faceva salire dal 74% all'81%.
        # L'errore a regime vale velocita/kp: a 15 m/s sono 7.5 m, entro i 50 m
        # di semicampo disponibili alla quota operativa.
        self.kp_x = parametro(self, 'kp_x', 2.0)      # 1/s
        self.kp_y = parametro(self, 'kp_y', 2.0)
        self.kd_x = parametro(self, 'kd_x', 0.35)
        self.kd_y = parametro(self, 'kd_y', 0.35)
        # Deve superare la velocita del bersaglio, altrimenti il drone non puo
        # recuperare terreno per costruzione. A 3.5 non pareggiava nemmeno gli
        # 8.3 m/s del bersaglio. Il limite e per asse: il modulo diagonale
        # arriva a vel_max*sqrt(2).
        # Deve superare la velocita del bersaglio, altrimenti il drone non
        # puo recuperare terreno per costruzione. Con il bersaglio a 15 m/s,
        # venti danno il margine necessario a chiudere la distanza invece di
        # limitarsi a non perderla.
        self.vel_max = parametro(self, 'vel_max', 20.0)

        # Zona morta ampia, in metri. Con 0.3 m il drone correggeva anche errori
        # minimi: con kp alto questo produce inclinazioni continue, e ogni grado
        # di inclinazione trasla l'inquadratura di 1/semi_fov. Il risultato
        # misurato era la perdita del bersaglio dopo 8 s pur essendo il bersaglio
        # quasi fermo. A 1.5 m il drone ignora gli scarti piccoli e resta piatto
        # quando il bersaglio e sotto di lui, conservando il guadagno alto per
        # quando serve davvero, cioe durante una fuga.
        # Zona morta pari a meta della lunghezza del veicolo: dentro quel
        # raggio il drone e gia sopra il bersaglio, e correggere ancora
        # produrrebbe solo inclinazioni inutili.
        self.deadzone = parametro(self, 'deadzone', 2.0)   # metri
        self.semi_fov_o = 0.7854     # rad, meta del FOV orizzontale (90°)
        self.semi_fov_v = 0.6435     # rad, meta del FOV verticale su 640x480
        # Precalcolate: compaiono in ogni messaggio del tracker, sia nella
        # compensazione d'assetto sia nella conversione in metri.
        self.tan_semi_fov_o = math.tan(self.semi_fov_o)   # = 1.0 a 90° di FOV
        self.tan_semi_fov_v = math.tan(self.semi_fov_v)   # = 0.750

        self.error_x_prev      = 0.0
        self.error_y_prev      = 0.0
        self.primo_aggancio    = True
        self.cmd_corrente      = Twist()

        # --- Watchdog sugli ingressi ---
        # Ogni callback registra l'istante del proprio ultimo messaggio. Il
        # timer di pubblicazione, prima di ripetere il comando, verifica che gli
        # ingressi su cui quel comando e stato calcolato siano ancora vivi.
        # Senza questa verifica la morte di detector_node, o un ponte immagini
        # che si ferma, lasciava il drone a ripetere all'infinito l'ultima
        # velocita nota: volo alla cieca fino a 5 m/s, senza che nulla nei log
        # lo segnalasse.
        self.istante_tracked = None
        self.istante_posa    = None
        self.istante_quota   = None
        self.timeout_percezione_s = parametro(
            self, 'timeout_percezione_s', 0.5)   # ~5 messaggi al ritmo camera
        # I due topic di MAVROS non arrivano allo stesso ritmo: la posa segue
        # LOCAL_POSITION_NED, la quota GLOBAL_POSITION_INT, che ArduPilot
        # trasmette piu lentamente. Soglie separate, altrimenti la piu lenta
        # farebbe scattare il watchdog di continuo a drone perfettamente sano.
        self.timeout_posa_s  = parametro(self, 'timeout_posa_s', 1.0)
        self.timeout_quota_s = parametro(self, 'timeout_quota_s', 2.0)

        # --- Coasting alla perdita di vista ---
        # Azzerare il comando appena il tracker rinuncia lasciava il drone
        # immobile per tutta l'attesa prima di RICERCA: ~1.4 s di inseguimento
        # sulla predizione di Kalman, poi fermo fino allo scadere di
        # soglia_avvia_ricerca_s in mission_node. Mantenere per qualche istante
        # l'ultima velocita comandata, smorzandola a zero, prosegue il moto
        # nella direzione in cui il bersaglio si stava muovendo, che e la piu
        # probabile per riacquisirlo. E una versione povera del termine di
        # anticipo previsto in Fase 5, che usera la velocita stimata dal filtro
        # invece dell'ultimo comando.
        self.istante_perdita_vista = None
        self.cmd_base_coasting     = (0.0, 0.0)
        self.durata_coasting_s     = parametro(self, 'durata_coasting_s', 2.0)

        # on_tracked è guidato dai messaggi del tracker, il cui ritmo segue la
        # telecamera (misurato fra 5 e 13 Hz): il termine derivativo va diviso
        # per l'intervallo reale, non per una costante.
        self.dt_nominale    = 0.1
        self.dt_min         = 0.02
        self.dt_max         = 0.5
        self.ultimo_istante = None

        self.vel_timer = self.create_timer(0.1, self.pubblica_velocita_continua)
        self.get_logger().info('ControllerNode avviato')

    def _calcola_dt(self):
        """Intervallo reale dall'ultima stima ricevuta, con clamp di sicurezza."""
        adesso = self.get_clock().now().nanoseconds / 1e9
        if self.ultimo_istante is None:
            self.ultimo_istante = adesso
            return self.dt_nominale
        dt = adesso - self.ultimo_istante
        self.ultimo_istante = adesso
        return float(min(max(dt, self.dt_min), self.dt_max))

    def _ingressi_scaduti(self):
        """Ingressi che non si aggiornano piu. Lista vuota = tutto vivo."""
        adesso = self.get_clock().now().nanoseconds / 1e9
        scaduti = []
        controlli = (
            ('percezione (/target/tracked_position)',
             self.istante_tracked, self.timeout_percezione_s),
            ('posa (/mavros/local_position/pose)',
             self.istante_posa, self.timeout_posa_s),
            ('quota (/mavros/global_position/rel_alt)',
             self.istante_quota, self.timeout_quota_s),
        )
        for nome, istante, limite in controlli:
            if istante is None or adesso - istante > limite:
                scaduti.append(nome)
        return scaduti

    def pubblica_velocita_continua(self):
        fase_ok = FaseMissione.AGGANCIO.value in self.fase_missione
        if not (self.in_volo and fase_ok):
            return

        scaduti = self._ingressi_scaduti()
        if scaduti:
            # Fermarsi e l'unica opzione sicura: senza la posa il comando non si
            # puo nemmeno ruotare nel frame del mondo (serve lo yaw), e senza
            # percezione non c'e piu un bersaglio da inseguire. Si continua a
            # pubblicare, azzerato: interrompere del tutto lo stream di setpoint
            # farebbe uscire ArduPilot da GUIDED.
            self.cmd_corrente = Twist()
            self.istante_perdita_vista = None
            self.primo_aggancio = True
            self.get_logger().error(
                'Ingressi scaduti: ' + ', '.join(scaduti) + ' — comando azzerato',
                throttle_duration_sec=2.0)
            self.mavros_vel_pub.publish(self.cmd_corrente)
            return

        self.mavros_vel_pub.publish(self._comando_da_pubblicare())

    def _velocita_istantanea(self, quota):
        """Velocita ASSOLUTA del bersaglio ricavata dall'ultimo fotogramma.

        Restituisce (avanti, laterale) oppure None se manca la stima del filtro
        o la velocita del velivolo: senza una delle due la ricostruzione
        sarebbe sbagliata, e una velocita sbagliata e peggio di nessuna.

        Da sola non va usata: un singolo fotogramma misura l'oscillazione del
        bersaglio nell'immagine piu che il suo moto. Alimenta la finestra da
        cui `_velocita_bersaglio` prende la mediana.
        """
        if not self.vel_stimata_valida:
            return None
        adesso = self.get_clock().now().nanoseconds / 1e9
        if (self.istante_vel_drone is None
                or adesso - self.istante_vel_drone > self.timeout_posa_s):
            return None

        # Da coordinate immagine al secondo a metri al secondo al suolo, con la
        # stessa conversione usata per la posizione.
        v_rel_x = self.vel_stimata_x * quota * self.tan_semi_fov_o
        v_rel_y = self.vel_stimata_y * quota * self.tan_semi_fov_v

        # Stessa mappatura fra assi immagine e assi velivolo usata per
        # l'errore: un bersaglio che si sposta verso +x nell'immagine si
        # allontana verso la sinistra del velivolo.
        rel_avanti = -v_rel_y
        rel_laterale = -v_rel_x

        # La velocita del velivolo e nel frame del mondo: va riportata nel
        # frame del velivolo per sommarla, poi il totale torna nel mondo nel
        # punto in cui viene usata.
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        drone_avanti = self.vel_drone_x * cos_y + self.vel_drone_y * sin_y
        drone_laterale = -self.vel_drone_x * sin_y + self.vel_drone_y * cos_y

        return rel_avanti + drone_avanti, rel_laterale + drone_laterale

    def _aggiorna_finestra_velocita(self, quota):
        """Aggiunge la stima di questo fotogramma e scarta le troppo vecchie."""
        istantanea = self._velocita_istantanea(quota)
        if istantanea is None:
            return
        adesso = self.get_clock().now().nanoseconds / 1e9
        self.finestra_velocita.append((adesso, istantanea[0], istantanea[1]))
        limite = adesso - self.finestra_velocita_s
        self.finestra_velocita = [c for c in self.finestra_velocita
                                  if c[0] >= limite]

    def _velocita_bersaglio(self):
        """Velocita ASSOLUTA del bersaglio, mediana sulla finestra recente.

        Restituisce (avanti, laterale) oppure None quando la stima non e
        utilizzabile, che qui vuol dire una di due cose: la finestra non ha
        ancora abbastanza campioni — succede subito dopo un riaggancio, ed e
        proprio allora che il filtro sta ancora convergendo — oppure la
        velocita risultante e fisicamente impossibile.

        Il rifiuto e voluto al posto del troncamento. Una stima da 76 m/s non
        significa "molto veloce": significa che quella misura non descrive il
        bersaglio, e chi la usa deve poterlo sapere invece di ricevere un
        valore plausibile costruito su un dato che non lo era.
        """
        adesso = self.get_clock().now().nanoseconds / 1e9
        if len(self.finestra_velocita) >= self.campioni_velocita_min:
            avanti = median([c[1] for c in self.finestra_velocita])
            laterale = median([c[2] for c in self.finestra_velocita])
            if math.hypot(avanti, laterale) <= self.vel_bersaglio_max:
                self.ultima_velocita_valida = (avanti, laterale, adesso)
                return avanti, laterale
            # Impossibile: non solo non si restituisce, non si ricorda
            # nemmeno. Una misura fuori dal fisico non deve poter sostituire
            # una buona.
            self.get_logger().warn(
                'Velocita del bersaglio impossibile ({:.0f} m/s su un limite '
                'di {:.0f}): stima rifiutata'.format(
                    math.hypot(avanti, laterale), self.vel_bersaglio_max),
                throttle_duration_sec=5.0)

        # Nessuna stima nuova utilizzabile: vale l'ultima ben stabilita, finche
        # e recente. Un bersaglio in fuga rettilinea non cambia velocita in un
        # secondo, e l'alternativa non e una stima migliore ma nessuna stima.
        if self.ultima_velocita_valida is None:
            return None
        avanti, laterale, istante = self.ultima_velocita_valida
        if adesso - istante > self.validita_velocita_s:
            return None
        return avanti, laterale

    def _anticipo(self):
        """Termine di anticipo da sommare al comando, nel frame del velivolo.

        E la velocita assoluta del bersaglio moltiplicata per il guadagno, che
        per default vale zero: la misura dice che non conviene, si veda il
        commento su k_anticipo.
        """
        if self.k_anticipo == 0.0:
            return 0.0, 0.0
        velocita = self._velocita_bersaglio()
        if velocita is None:
            self.get_logger().warn(
                'Velocita del bersaglio non ricostruibile: anticipo disattivato',
                throttle_duration_sec=5.0)
            return 0.0, 0.0
        return self.k_anticipo * velocita[0], self.k_anticipo * velocita[1]

    def _pubblica_stima_mondo(self, error_x, error_y, quota, cos_y, sin_y):
        """Posizione e velocita del bersaglio nel frame del mondo.

        Le stesse grandezze che il controllo usa per il proprio comando, messe
        a disposizione di chi deve decidere dove cercare quando l'aggancio si
        perde. Senza, la ricerca puo solo essere isotropa, cioe ignorare
        l'unica informazione utile che si possiede.
        """
        if self.posizione_drone is None:
            return
        # Stessa mappatura fra assi immagine e assi velivolo usata per il
        # comando: un errore positivo lungo x corrisponde a un bersaglio
        # spostato verso sinistra del velivolo.
        rel_avanti = -error_y
        rel_laterale = -error_x

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = (self.posizione_drone[0]
                                    + rel_avanti * cos_y - rel_laterale * sin_y)
        msg.pose.pose.position.y = (self.posizione_drone[1]
                                    + rel_avanti * sin_y + rel_laterale * cos_y)
        msg.pose.pose.orientation.w = 1.0

        velocita = self._velocita_bersaglio()
        if velocita is not None:
            avanti, laterale = velocita
            msg.twist.twist.linear.x = avanti * cos_y - laterale * sin_y
            msg.twist.twist.linear.y = avanti * sin_y + laterale * cos_y
            # La covarianza non e stimata: si usa il primo elemento come
            # bandiera di validita della velocita, che e l'informazione che
            # serve a valle.
            msg.twist.covariance[0] = 1.0
        self.stima_pub.publish(msg)

    def _avvia_coasting(self):
        """Congela il comando da cui parte la rampa di smorzamento.

        Solo la prima chiamata ha effetto: sui messaggi successivi la base non
        va ritoccata, altrimenti lo smorzamento si applicherebbe piu volte allo
        stesso valore e il coasting si spegnerebbe in un istante.
        """
        if self.istante_perdita_vista is None:
            self.istante_perdita_vista = self.istante_tracked
            self.cmd_base_coasting = (self.cmd_corrente.linear.x,
                                      self.cmd_corrente.linear.y)
        self.primo_aggancio = True
        self.ultimo_istante = None

    def _comando_da_pubblicare(self):
        """Comando corrente, smorzato se il bersaglio non e piu in vista."""
        if self.istante_perdita_vista is None:
            return self.cmd_corrente

        trascorso = (self.get_clock().now().nanoseconds / 1e9
                     - self.istante_perdita_vista)
        if trascorso >= self.durata_coasting_s:
            return Twist()

        fattore = 1.0 - trascorso / self.durata_coasting_s
        cmd = Twist()
        cmd.linear.x = self.cmd_base_coasting[0] * fattore
        cmd.linear.y = self.cmd_base_coasting[1] * fattore
        return cmd

    def on_stato_missione(self, msg: String):
        self.fase_missione = msg.data
        # Reset aggancio quando si esce da AGGANCIO
        if FaseMissione.AGGANCIO.value not in msg.data:
            self.primo_aggancio = True
            self.cmd_corrente = Twist()
            self.istante_perdita_vista = None

    def on_posa(self, msg: PoseStamped):
        self.istante_posa = self.get_clock().now().nanoseconds / 1e9
        p = msg.pose.position
        self.posizione_drone = (p.x, p.y)
        q = msg.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                               1.0 - 2.0 * (q.x * q.x + q.y * q.y))

    def on_velocita_stimata(self, msg: Point):
        self.vel_stimata_x = msg.x
        self.vel_stimata_y = msg.y
        self.vel_stimata_valida = (msg.z != 0.0)

    def on_velocita_drone(self, msg: TwistStamped):
        self.istante_vel_drone = self.get_clock().now().nanoseconds / 1e9
        self.vel_drone_x = msg.twist.linear.x
        self.vel_drone_y = msg.twist.linear.y

    def on_gimbal_roll(self, msg: Float64):
        self.gimbal_roll = msg.data

    def on_gimbal_pitch(self, msg: Float64):
        self.gimbal_pitch = msg.data

    def on_gps_status(self, msg: Bool):
        if msg.data and not self.gps_jammed:
            self.get_logger().warn('GPS perso — modalità visione attiva')
        elif not msg.data and self.gps_jammed:
            self.get_logger().info('GPS ripristinato')
        self.gps_jammed = msg.data

    def on_tracked(self, msg: Point):
        self.istante_tracked = self.get_clock().now().nanoseconds / 1e9
        cmd = Twist()
        target_visible = (msg.x != 0.0 or msg.y != 0.0)

        if not target_visible:
            self._avvia_coasting()
            return

        # Guardia FOV. La stima cade fuori dal campo inquadrabile, quindi non e
        # affidabile e non la si insegue. Prima si azzerava il comando di netto:
        # ma questa e la situazione tipica di una fuga veloce, in cui il
        # bersaglio scivola verso il bordo poco prima di sparire, e azzerare
        # proprio lì svuotava il coasting del suo contenuto (la base sarebbe
        # stata zero). Si tratta come una perdita di vista: non si dà retta alla
        # stima sospetta, ma si prosegue smorzando l'ultimo comando buono.
        if abs(msg.x) > 1.2 or abs(msg.y) > 1.2:
            self._avvia_coasting()
            return

        self.istante_perdita_vista = None

        # Si sottrae la traslazione d'immagine dovuta all'assetto, così l'errore
        # rappresenta la posizione del bersaglio e non l'inclinazione del drone.
        #
        # La sottrazione va fatta sugli ANGOLI, non sulle coordinate
        # normalizzate: in una proiezione prospettica vale
        # u = tan(alpha)/tan(semi_fov), quindi coordinata e angolo non sono
        # proporzionali. Dividere l'assetto per il semicampo in radianti, come
        # si faceva prima, sovracorregge: il denominatore corretto e
        # tan(0.7854) = 1.0 e non 0.7854 in orizzontale (27% in meno di quanto
        # veniva sottratto) e tan(0.6435) = 0.750 e non 0.6435 in verticale
        # (16%). Si converte la misura in angolo, si toglie l'assetto, si torna
        # in coordinate normalizzate.
        # Assetto della TELECAMERA, non del corpo: quello che trasla
        # l'inquadratura e l'inclinazione dell'asse ottico, che vale assetto
        # del corpo piu angolo del giunto. Sottrarre l'assetto del corpo
        # quando il gimbal lo ha gia annullato introduce un errore fantasma
        # invece di rimuoverne uno: misurato come distanza mediana da 3.7 a
        # 10.5 m e un ingresso in RICERCA che senza gimbal non avveniva.
        roll_camera = self.roll + self.gimbal_roll
        pitch_camera = self.pitch + self.gimbal_pitch
        alpha_x = math.atan(msg.x * self.tan_semi_fov_o) - roll_camera
        alpha_y = math.atan(msg.y * self.tan_semi_fov_v) + pitch_camera
        # Il clamp a 80° evita che la tangente esploda in un transitorio
        # anomalo: con la guardia FOV a 1.2 e l'assetto limitato a 25° da
        # ATC_ANGLE_MAX non ci si arriva, e il comando risultante verrebbe
        # comunque saturato a vel_max poche righe piu sotto.
        limite = 1.4   # rad
        alpha_x = max(-limite, min(limite, alpha_x))
        alpha_y = max(-limite, min(limite, alpha_y))
        norm_x = math.tan(alpha_x) / self.tan_semi_fov_o
        norm_y = math.tan(alpha_y) / self.tan_semi_fov_v

        # Conversione in metri sul terreno: con la telecamera a nadir e quota h,
        # il semicampo copre h*tan(semi_fov), quindi una coordinata normalizzata
        # vale quella distanza per unità.
        quota = max(self.altitudine, 1.0)
        error_x = norm_x * quota * self.tan_semi_fov_o
        error_y = norm_y * quota * self.tan_semi_fov_v

        dt = self._calcola_dt()

        # Evita derivative kick al primo frame
        if self.primo_aggancio:
            self.error_x_prev = error_x
            self.error_y_prev = error_y
            self.primo_aggancio = False

        deriv_x = (error_x - self.error_x_prev) / dt
        deriv_y = (error_y - self.error_y_prev) / dt
        self.error_x_prev = error_x
        self.error_y_prev = error_y

        # Nessuna scalatura con la quota: l'errore è già in metri, e la quota è
        # entrata nella conversione da coordinate immagine a distanza al suolo.
        # Comando nel frame del drone.
        v_avanti = 0.0
        v_laterale = 0.0

        if abs(error_x) > self.deadzone:
            v_laterale = -(self.kp_x * error_x + self.kd_x * deriv_x)
            v_laterale = max(-self.vel_max, min(self.vel_max, v_laterale))

        if abs(error_y) > self.deadzone:
            v_avanti = -(self.kp_y * error_y + self.kd_y * deriv_y)
            v_avanti = max(-self.vel_max, min(self.vel_max, v_avanti))

        # Rotazione dal frame del drone a quello locale ENU.
        # `/mavros/setpoint_velocity/cmd_vel_unstamped` viene tradotto da MAVROS
        # in SET_POSITION_TARGET_LOCAL_NED con frame LOCAL_NED, cioè il frame del
        # MONDO: pubblicare lì un vettore calcolato nel frame della telecamera è
        # corretto solo se lo yaw è zero. In volo lo yaw non è controllato e
        # deriva: misurato 25° di media con ±21° di oscillazione, che portava il
        # comando a puntare in media 61° fuori bersaglio — il drone spingeva di
        # traverso e non riusciva a seguire nemmeno un'orbita lenta.
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        cmd.linear.x = v_avanti * cos_y - v_laterale * sin_y
        cmd.linear.y = v_avanti * sin_y + v_laterale * cos_y

        # --- Termine di anticipo ---
        # Il controllo proporzionale corregge lo scarto attuale; questo termine
        # aggiunge la velocita necessaria a non accumularne di nuovo. La stima
        # del filtro e relativa al drone, quindi si somma la velocita del
        # velivolo per ottenere quella assoluta del bersaglio.
        # Prima la finestra, poi chi la legge: l'ordine conta, altrimenti
        # la stima pubblicata sarebbe vecchia di un fotogramma rispetto a
        # quella usata per il comando, e due grandezze che devono coincidere
        # non coinciderebbero.
        self._aggiorna_finestra_velocita(quota)
        self._pubblica_stima_mondo(error_x, error_y, quota, cos_y, sin_y)

        avanti_ff, laterale_ff = self._anticipo()
        cmd.linear.x += avanti_ff * cos_y - laterale_ff * sin_y
        cmd.linear.y += avanti_ff * sin_y + laterale_ff * cos_y

        # La saturazione va applicata al comando completo: i due termini
        # sommati possono superare il limite anche se ciascuno lo rispetta.
        cmd.linear.x = max(-self.vel_max, min(self.vel_max, cmd.linear.x))
        cmd.linear.y = max(-self.vel_max, min(self.vel_max, cmd.linear.y))

        self.cmd_corrente = cmd
        self.cmd_pub.publish(cmd)

        mode = 'GPS+VISIONE' if not self.gps_jammed else 'SOLO VISIONE'
        # self.get_logger().info(
        #     f'[{mode}] err:({error_x:.2f},{error_y:.2f}) '
        #     f'→ v:({cmd.linear.y:.2f},{cmd.linear.x:.2f})')

    def on_altitudine(self, msg):
        self.istante_quota = self.get_clock().now().nanoseconds / 1e9
        self.altitudine = msg.data
        self.in_volo    = self.altitudine > 1.0

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()