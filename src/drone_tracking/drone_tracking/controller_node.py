#!/usr/bin/env python3
import math
from statistics import median
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Twist
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
        # secondo. E gia la sua velocita propria: il filtro riceve il moto del
        # velivolo come ingresso noto.
        self.create_subscription(
            Point, '/target/tracked_velocity', self.on_velocita_stimata, 10)

        self.gps_sub = self.create_subscription(
            Bool, '/gps/jammed', self.on_gps_status, 10)

        # Angoli comandati al gimbal: dicono quanta parte dell'assetto e gia
        # compensata meccanicamente, cosi la compensazione analitica si occupa
        # del solo residuo. Senza gimbal restano zero e il calcolo non cambia.
        self.create_subscription(
            Float64, '/gimbal/roll/cmd_pos', self.on_gimbal_roll, 10)
        self.create_subscription(
            Float64, '/gimbal/pitch/cmd_pos', self.on_gimbal_pitch, 10)

        # Serve lo yaw per convertire i comandi dal frame del drone a quello del
        # mondo: vedi la nota in on_tracked.
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.on_posa, qos_mavros)

        self.cmd_pub = self.create_publisher(
            Twist, '/drone/cmd_vel', 10)

        self.mavros_vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Stima del bersaglio nel mondo, in metri. Il controllo la calcola gia
        # per il proprio comando: pubblicarla evita che la missione rifaccia la
        # stessa conversione, e due copie divergono al primo che ne corregge una.
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

        # Anticipo: pareggia la velocita del bersaglio invece di inseguirla.
        # Spento perche misurato la stima era troppo rumorosa (README, 6.5); da
        # riprovare ora che il filtro ha il moto del velivolo come ingresso.
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
        # Per quanto una velocita stabilita da misure vere resta usabile dopo
        # che le misure sono cessate: i fotogrammi prima della perdita sono
        # predizioni, e senza memoria sarebbe ignota proprio quando serve.
        self.validita_velocita_s = parametro(self, 'validita_velocita_s', 2.0)
        # Campioni recenti: (istante, avanti, laterale).
        self.finestra_velocita = []
        # Ultimo valore ben stabilito: (avanti, laterale, istante).
        self.ultima_velocita_valida = None
        self.vel_stimata_x = 0.0
        self.vel_stimata_y = 0.0
        self.vel_stimata_valida = False

        # Compensazione d'assetto: una rotazione del velivolo trasla l'immagine
        # indipendentemente da dove sia il bersaglio, e bastano 10 gradi di
        # pitch per spostarlo di mezzo campo.

        # Guadagni PD in 1/s, applicati allo scostamento in METRI: sulle
        # coordinate normalizzate la taratura andrebbe rifatta a ogni cambio di
        # quota o ottica. L'errore a regime vale velocita/kp, 7.5 m a 15 m/s.
        self.kp_x = parametro(self, 'kp_x', 2.0)      # 1/s
        self.kp_y = parametro(self, 'kp_y', 2.0)
        self.kd_x = parametro(self, 'kd_x', 0.35)
        self.kd_y = parametro(self, 'kd_y', 0.35)
        # Deve superare la velocita del bersaglio, o il drone non puo recuperare
        # terreno per costruzione. Il limite e per asse: il modulo diagonale
        # arriva a vel_max*sqrt(2).
        self.vel_max = parametro(self, 'vel_max', 20.0)

        # Zona morta pari a meta della lunghezza del veicolo: dentro quel raggio
        # il drone e gia sopra il bersaglio, e correggere ancora produrrebbe
        # solo inclinazioni, che tolgono campo utile senza avvicinare nulla.
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

        # Watchdog: ogni callback registra l'istante del proprio ultimo
        # messaggio, e il timer verifica che gli ingressi su cui il comando e
        # stato calcolato siano ancora vivi. Senza, la morte del rilevatore
        # lascerebbe il drone a ripetere all'infinito l'ultima velocita nota.
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

        # Coasting: alla perdita di vista il comando non si azzera di netto ma
        # si smorza a zero, proseguendo nella direzione in cui il bersaglio si
        # stava muovendo, che e la piu probabile per riacquisirlo. Azzerare
        # subito lasciava il drone immobile per tutta l'attesa prima di RICERCA.
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
            # Fermarsi e l'unica opzione sicura: senza posa il comando non si
            # puo nemmeno ruotare nel frame del mondo. Si continua a pubblicare
            # azzerato, perche interrompere i setpoint farebbe uscire da GUIDED.
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
        """Velocita del bersaglio in assi velivolo, da questo fotogramma.

        Il filtro la stima gia come velocita PROPRIA del bersaglio, perche il
        moto del velivolo vi entra come ingresso noto: qui resta solo da
        convertirla da coordinate immagine a metri al secondo. Prima andava
        sommata la velocita del velivolo, e quella somma fra due grandezze
        grandi per ottenerne una piccola era l'origine dell'errore pari al
        segnale.

        Da sola non va usata: un singolo fotogramma misura l'oscillazione del
        bersaglio nell'immagine piu che il suo moto. Alimenta la finestra da
        cui `_velocita_bersaglio` prende la mediana.
        """
        if not self.vel_stimata_valida:
            return None

        # Da coordinate immagine al secondo a metri al secondo al suolo, con
        # la stessa conversione usata per la posizione. La mappatura fra assi
        # immagine e assi velivolo e quella dell'errore: un bersaglio che si
        # sposta verso +x nell'immagine va verso la sinistra del velivolo.
        return (-self.vel_stimata_y * quota * self.tan_semi_fov_v,
                -self.vel_stimata_x * quota * self.tan_semi_fov_o)

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

        # Guardia FOV: la stima cade fuori dal campo inquadrabile e non e
        # affidabile. Si tratta come una perdita di vista invece di azzerare il
        # comando — e la situazione tipica di una fuga veloce, e azzerare
        # proprio li svuoterebbe il coasting del suo contenuto.
        if abs(msg.x) > 1.2 or abs(msg.y) > 1.2:
            self._avvia_coasting()
            return

        self.istante_perdita_vista = None

        # Si toglie la traslazione d'immagine dovuta all'assetto, cosi che
        # l'errore dica dov'e il bersaglio e non quanto e inclinato il drone.
        # Due avvertenze, entrambe costate una misura sbagliata:
        #  - si sottrae sugli ANGOLI e non sulle coordinate normalizzate, che in
        #    proiezione prospettica valgono tan(alpha)/tan(semi_fov) e quindi
        #    non sono proporzionali all'angolo;
        #  - conta l'assetto della TELECAMERA, corpo piu giunto: sottrarre
        #    quello del corpo dove il gimbal lo ha gia annullato aggiunge un
        #    errore invece di toglierlo.
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

        # Rotazione dal frame del drone a quello locale ENU. MAVROS traduce
        # questo topic in SET_POSITION_TARGET_LOCAL_NED, che e nel frame del
        # MONDO: pubblicarvi un vettore calcolato nel frame della telecamera
        # sarebbe corretto solo a yaw nullo, e in volo lo yaw non e controllato.
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        cmd.linear.x = v_avanti * cos_y - v_laterale * sin_y
        cmd.linear.y = v_avanti * sin_y + v_laterale * cos_y

        # Prima si aggiorna la finestra, poi la leggono entrambi: altrimenti la
        # stima pubblicata sarebbe vecchia di un fotogramma rispetto a quella
        # usata per il comando, e due grandezze che devono coincidere non
        # coinciderebbero.
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