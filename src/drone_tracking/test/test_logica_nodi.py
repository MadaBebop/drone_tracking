"""Prove automatiche sulla logica dei nodi.

Non fanno volare nulla: verificano i percorsi di codice che in simulazione si
osservano male, perche richiedono di provocare un guasto (fermare il detector,
far tacere MAVROS) o di aspettare che scada una soglia. Sono le correzioni piu
facili da rompere in seguito senza accorgersene, ed e per questo che stanno qui.

    colcon test --packages-select drone_tracking
    colcon test-result --verbose

Il tempo non si puo far avanzare, quindi si riavvolgono gli istanti registrati
dai nodi: mettere l'ultimo messaggio "cinque secondi nel passato" equivale ad
aspettare cinque secondi senza riceverne.
"""
import math

import pytest
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

from drone_tracking.controller_node import ControllerNode
from drone_tracking.gimbal_node import GimbalNode
from drone_tracking.gnss_denial_node import MODI, GnssDenialNode
from drone_tracking.mission_node import FaseMissione, MissionNode
from drone_tracking.tracker_node import TrackerNode


@pytest.fixture(scope='module', autouse=True)
def contesto_ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def ora(nodo):
    return nodo.get_clock().now().nanoseconds / 1e9


def controller_in_aggancio():
    """Controller pronto a inseguire: in volo, con posa e fase corrette."""
    nodo = ControllerNode()
    nodo.on_posa(PoseStamped())            # assetto piatto, yaw zero
    # Quota di crociera reale dello scenario. Non e un dettaglio: la zona
    # morta e in metri, e la stessa coordinata immagine vale meno metri di
    # errore piu si vola bassi. A 12 m — la quota della prima versione — un
    # bersaglio a un quinto di semicampo cadeva sotto soglia e il comando
    # usciva nullo, facendo fallire le prove sul coasting per un motivo che
    # con il coasting non c'entra nulla.
    nodo.on_altitudine(Float64(data=50.0))
    nodo.on_stato_missione(String(data=FaseMissione.AGGANCIO.value))
    return nodo


def test_compensazione_dassetto_lavora_sugli_angoli():
    """Se il bersaglio in immagine e solo l'effetto dell'assetto, l'errore e nullo.

    E la proprieta che distingue la correzione giusta da quella precedente, che
    divideva l'assetto per il semicampo in radianti e sovracorreggeva del 27%.
    """
    nodo = controller_in_aggancio()
    try:
        u = 0.3
        nodo.roll = math.atan(u * nodo.tan_semi_fov_o)
        nodo.pitch = 0.0
        nodo.on_tracked(Point(x=u, y=0.0, z=300.0))
        assert abs(nodo.cmd_corrente.linear.x) < 1e-6
        assert abs(nodo.cmd_corrente.linear.y) < 1e-6
    finally:
        nodo.destroy_node()


def test_compensazione_tiene_conto_del_gimbal():
    """Con il gimbal attivo l'assetto e gia compensato: non va sottratto due volte.

    E il difetto che ha fatto peggiorare l'inseguimento quando la sospensione
    cardanica e stata introdotta: due correzioni sullo stesso effetto, di cui
    la seconda con il segno rovesciato.
    """
    nodo = controller_in_aggancio()
    try:
        u = 0.25
        # Il corpo e inclinato, ma il giunto compensa esattamente: la
        # telecamera guarda dove guardava, quindi l'immagine dice il vero e
        # non va corretta. Un bersaglio al centro deve dare errore nullo.
        nodo.roll = 0.30
        nodo.gimbal_roll = -0.30
        nodo.pitch = 0.0
        nodo.gimbal_pitch = 0.0
        nodo.on_tracked(Point(x=0.0, y=0.0, z=300.0))
        assert abs(nodo.cmd_corrente.linear.y) < 1e-6
        assert abs(nodo.cmd_corrente.linear.x) < 1e-6

        # Gimbal in saturazione: resta un residuo, e quello va compensato.
        nodo.primo_aggancio = True
        nodo.roll = 0.60
        nodo.gimbal_roll = -0.45
        residuo = 0.15
        nodo.on_tracked(Point(x=math.tan(residuo) / nodo.tan_semi_fov_o,
                              y=0.0, z=300.0))
        assert abs(nodo.cmd_corrente.linear.y) < 1e-6, (
            'il residuo di saturazione deve essere ancora compensato')
    finally:
        nodo.destroy_node()


def controller_con_anticipo():
    """Controller con il termine di anticipo acceso.

    Il default e zero, perche misurato non conviene: le prove che verificano
    l'aritmetica del termine devono quindi accenderlo esplicitamente,
    altrimenti passerebbero perche il termine non viene calcolato affatto.
    """
    from rclpy.parameter import Parameter

    nodo = controller_in_aggancio()
    nodo.set_parameters([Parameter('k_anticipo', Parameter.Type.DOUBLE, 1.0)])
    return nodo


def riempi_finestra(nodo, quota, ripetizioni=None):
    """Porta la finestra della velocita a un numero di campioni sufficiente.

    La stima istantanea impostata sulle variabili del nodo viene inserita piu
    volte: la grandezza usata e la mediana, e sotto il minimo di campioni
    l'intero termine si disattiva.
    """
    for _ in range(ripetizioni or nodo.campioni_velocita_min):
        nodo._aggiorna_finestra_velocita(quota)


def test_anticipo_pareggia_la_velocita_del_bersaglio():
    """Con il drone fermo, l'anticipo vale la velocita stimata del bersaglio."""
    nodo = controller_con_anticipo()
    try:
        quota = 12.0
        nodo.vel_stimata_valida = True
        nodo.vel_stimata_x = 0.0
        nodo.vel_stimata_y = 0.1          # unita normalizzate al secondo
        riempi_finestra(nodo, quota)
        avanti, laterale = nodo._anticipo()
        atteso = -0.1 * quota * nodo.tan_semi_fov_v
        assert abs(avanti - atteso) < 1e-9
        assert abs(laterale) < 1e-9
    finally:
        nodo.destroy_node()


def test_anticipo_si_disattiva_senza_i_dati_necessari():
    from rclpy.parameter import Parameter

    nodo = controller_con_anticipo()
    try:
        nodo.vel_stimata_valida = True
        nodo.vel_stimata_y = 0.1

        # Stima del filtro non valida. E la sola condizione rimasta al
        # controllo: la validita dell'ingresso noto la verifica il tracker, che
        # semplicemente non pubblica una velocita che non sia quella propria
        # del bersaglio.
        nodo.vel_stimata_valida = False
        riempi_finestra(nodo, 12.0)
        assert nodo._anticipo() == (0.0, 0.0)

        # Un campione solo: sotto il minimo, la mediana non e affidabile.
        nodo.vel_stimata_valida = True
        nodo._aggiorna_finestra_velocita(12.0)
        assert len(nodo.finestra_velocita) == 1
        assert nodo._anticipo() == (0.0, 0.0)

        # Termine disattivato per scelta.
        riempi_finestra(nodo, 12.0)
        nodo.set_parameters([Parameter('k_anticipo', Parameter.Type.DOUBLE, 0.0)])
        assert nodo._anticipo() == (0.0, 0.0)
    finally:
        nodo.destroy_node()


def test_la_velocita_predetta_non_e_valida():
    """Durante la predizione la velocita non porta informazione nuova.

    Lo stato di velocita resta congelato all'ultimo valore stimato finche non
    arriva una misura. Pubblicarlo come valido significa spacciare per misure
    ripetute cio che e una sola misura ripetuta: chi ne fa una mediana la
    trova immobile, e il filtraggio si annulla proprio nei fotogrammi che
    precedono la perdita, gli unici in cui serve.

    La POSIZIONE predetta resta invece valida: e cio che tollera le
    micro-interruzioni dell'inseguimento.
    """
    nodo = tracker_con_velivolo()
    pubblicate = []
    nodo._pubblica_velocita = lambda valida: pubblicate.append(valida)
    try:
        nodo.on_detection(Point(x=0.1, y=0.1, z=100.0))   # acquisizione
        nodo.istante_vel_drone = ora(nodo)
        nodo.on_detection(Point(x=0.2, y=0.1, z=100.0))   # misura
        assert pubblicate[-1] is True, 'una misura deve valere una velocita'

        nodo.on_detection(Point(x=0.0, y=0.0, z=0.0))     # niente segnale
        assert pubblicate[-1] is False, (
            'la velocita predetta e stata pubblicata come valida')
    finally:
        nodo.destroy_node()


def test_velocita_impossibile_viene_rifiutata():
    """Una stima oltre il limite fisico non va troncata, va rifiutata.

    Misurato su quattordici perdite: tre stime valevano 41, 33 e 76 m/s contro
    un bersaglio che non supera i 15. Estrapolare 76 m/s per otto secondi manda
    il punto di ricerca a seicento metri dal vero; troncare a 25 lo manderebbe
    a duecento, sempre nella direzione sbagliata. Un valore impossibile non
    significa "circa quello", significa "non lo so", e la ricerca deve poterlo
    sapere.
    """
    nodo = controller_in_aggancio()
    try:
        quota = 50.0
        nodo.vel_stimata_valida = True
        nodo.vel_stimata_x = 0.0
        # 2.0 unita normalizzate al secondo a 50 m di quota sono 75 m/s.
        nodo.vel_stimata_y = 2.0
        for _ in range(nodo.campioni_velocita_min):
            nodo._aggiorna_finestra_velocita(quota)
        assert nodo._velocita_bersaglio() is None, (
            'una velocita impossibile e stata accettata')

        # Sotto il limite la stessa strada deve invece produrre un valore.
        nodo.finestra_velocita = []
        nodo.vel_stimata_y = 0.2          # 7.5 m/s
        for _ in range(nodo.campioni_velocita_min):
            nodo._aggiorna_finestra_velocita(quota)
        assert nodo._velocita_bersaglio() is not None
    finally:
        nodo.destroy_node()


def test_la_velocita_stabilita_sopravvive_alla_fine_delle_misure():
    """Cessate le misure, la velocita resta utilizzabile per un tempo breve.

    I fotogrammi che precedono la perdita sono predizioni e non alimentano la
    finestra: senza memoria la velocita risulterebbe ignota proprio alla
    perdita, cioe nell'unico istante in cui la ricerca deve usarla. Misurato:
    con la sola esclusione delle predizioni la stima pubblicata alla perdita
    valeva (0.0, 0.0) e la ricerca smetteva di extrapolare.
    """
    nodo = controller_in_aggancio()
    try:
        quota = 50.0
        nodo.vel_stimata_valida = True
        nodo.vel_stimata_x = 0.0
        nodo.vel_stimata_y = 0.2
        riempi_finestra(nodo, quota)
        stabilita = nodo._velocita_bersaglio()
        assert stabilita is not None

        # Le misure cessano: finestra vuota, come dopo qualche fotogramma di
        # sola predizione.
        nodo.finestra_velocita = []
        assert nodo._velocita_bersaglio() == stabilita, (
            'la velocita appena stabilita e stata dimenticata subito')

        # Passato il tempo di validita torna ignota, invece di restare vera
        # per sempre.
        avanti, laterale, istante = nodo.ultima_velocita_valida
        nodo.ultima_velocita_valida = (
            avanti, laterale, istante - nodo.validita_velocita_s - 1.0)
        assert nodo._velocita_bersaglio() is None
    finally:
        nodo.destroy_node()


def test_una_stima_impossibile_non_sostituisce_una_buona():
    """Il valore ricordato non viene aggiornato da una stima fuori dal fisico."""
    nodo = controller_in_aggancio()
    try:
        quota = 50.0
        nodo.vel_stimata_valida = True
        nodo.vel_stimata_x = 0.0
        nodo.vel_stimata_y = 0.2                  # 7.5 m/s, plausibile
        riempi_finestra(nodo, quota)
        buona = nodo._velocita_bersaglio()

        nodo.finestra_velocita = []
        nodo.vel_stimata_y = 2.0                  # 75 m/s, impossibile
        riempi_finestra(nodo, quota)
        assert nodo._velocita_bersaglio() == buona, (
            'una stima impossibile ha sostituito quella buona')
    finally:
        nodo.destroy_node()


def test_la_mediana_scarta_le_inversioni_di_direzione():
    """Una direzione che si inverte fra fotogrammi non deve sopravvivere.

    E la situazione misurata prima di ogni perdita: il bersaglio scivola al
    bordo dell'inquadratura mentre il velivolo manovra, e la sua posizione
    nell'immagine oscilla da un lato all'altro. La velocita istantanea segue
    l'oscillazione, la mediana no.
    """
    nodo = controller_in_aggancio()
    try:
        quota = 50.0
        nodo.vel_stimata_valida = True
        nodo.vel_stimata_x = 0.0

        # Sette campioni: cinque di un moto coerente e due che lo negano.
        for valore in (0.1, 0.1, -0.4, 0.1, 0.1, 0.5, 0.1):
            nodo.vel_stimata_y = valore
            nodo._aggiorna_finestra_velocita(quota)

        avanti, _ = nodo._velocita_bersaglio()
        atteso = -0.1 * quota * nodo.tan_semi_fov_v
        assert abs(avanti - atteso) < 1e-6, (
            'la mediana ha seguito le inversioni: %.2f invece di %.2f'
            % (avanti, atteso))
    finally:
        nodo.destroy_node()


def test_coasting_smorza_invece_di_azzerare():
    nodo = controller_in_aggancio()
    try:
        nodo.on_tracked(Point(x=0.3, y=0.2, z=300.0))
        base = nodo.cmd_corrente.linear.x
        assert abs(base) > 0.1, 'il comando di partenza deve essere non nullo'

        nodo.on_tracked(Point(x=0.0, y=0.0, z=0.0))
        # Tolleranza larga: fra la perdita e questa riga passano microsecondi di
        # orologio reale, che la rampa di smorzamento conta comunque.
        assert abs(nodo._comando_da_pubblicare().linear.x - base) < 0.05

        # Meta della rampa trascorsa: comando dimezzato.
        nodo.istante_perdita_vista = ora(nodo) - nodo.durata_coasting_s / 2
        assert abs(abs(nodo._comando_da_pubblicare().linear.x)
                   - abs(base) / 2) < 0.05

        # Rampa esaurita: comando nullo.
        nodo.istante_perdita_vista = ora(nodo) - nodo.durata_coasting_s - 0.1
        assert nodo._comando_da_pubblicare().linear.x == 0.0
    finally:
        nodo.destroy_node()


def test_guardia_fov_passa_al_coasting():
    """Una stima fuori campo non viene inseguita, ma non azzera il comando.

    E la situazione tipica di una fuga veloce: il bersaglio scivola al bordo
    poco prima di sparire, e azzerare li svuoterebbe il coasting.
    """
    nodo = controller_in_aggancio()
    try:
        nodo.on_tracked(Point(x=0.4, y=0.3, z=300.0))
        base = nodo.cmd_corrente.linear.x
        nodo.on_tracked(Point(x=1.5, y=0.3, z=300.0))
        assert nodo.istante_perdita_vista is not None
        assert abs(nodo._comando_da_pubblicare().linear.x) > 0.9 * abs(base)
    finally:
        nodo.destroy_node()


def test_watchdog_azzera_e_segnala():
    nodo = controller_in_aggancio()
    try:
        nodo.on_tracked(Point(x=0.3, y=0.2, z=300.0))
        nodo.pubblica_velocita_continua()
        assert nodo._ingressi_scaduti() == []

        # Tutti gli ingressi zittiti da cinque secondi.
        adesso = ora(nodo)
        nodo.istante_tracked = adesso - 5.0
        nodo.istante_posa = adesso - 5.0
        nodo.istante_quota = adesso - 5.0
        scaduti = nodo._ingressi_scaduti()
        assert len(scaduti) == 3

        nodo.pubblica_velocita_continua()
        assert nodo.cmd_corrente.linear.x == 0.0
        assert nodo.cmd_corrente.linear.y == 0.0
    finally:
        nodo.destroy_node()


def test_parametro_si_aggiorna_a_caldo():
    """`ros2 param set` deve arrivare all'attributo, non solo al parametro.

    Prima i parametri venivano letti una volta sola nel costruttore: un set a
    runtime cambiava un valore che nessuno rileggeva, e una scansione di
    velocita di fuga ha misurato tre volte la stessa configurazione credendo di
    variarla. La callback in parametri.py chiude quel buco, e questa prova
    impedisce che si riapra.
    """
    from rclpy.parameter import Parameter

    nodo = ControllerNode()
    try:
        nodo.set_parameters([Parameter('kp_x', Parameter.Type.DOUBLE, 3.3)])
        assert abs(nodo.kp_x - 3.3) < 1e-9
        nodo.set_parameters([Parameter('vel_max', Parameter.Type.DOUBLE, 7.5)])
        assert abs(nodo.vel_max - 7.5) < 1e-9
    finally:
        nodo.destroy_node()


def tracker_con_velivolo(avanti=0.0, laterale=0.0, quota=50.0):
    """Tracker che conosce il moto del velivolo, quindi applica l'ingresso noto.

    Senza questi valori il filtro non puo sapere quanta parte dello spostamento
    d'immagine sia sua, e allora non applica l'ingresso e non pubblica la
    velocita: e il comportamento prudente, non un caso da aggirare nelle prove.
    """
    nodo = TrackerNode()
    nodo.yaw = 0.0
    nodo.vel_drone = (avanti, laterale)
    nodo.istante_vel_drone = ora(nodo)
    nodo.quota = quota
    return nodo


def test_ingresso_noto_scarta_il_moto_del_velivolo():
    """Bersaglio fermo, velivolo che avanza: la velocita stimata resta nulla.

    E la proprieta che distingue questo filtro da quello di prima. Un bersaglio
    immobile scivola comunque nell'immagine mentre il velivolo trasla, e senza
    ingresso noto il filtro attribuisce quello scorrimento al bersaglio. Qui il
    velivolo fa 10 m/s in avanti a 50 m di quota: il bersaglio arretra
    nell'immagine di 10*dt/(quota*TAN_V) a ogni campione, e il filtro deve
    riconoscere che il movimento e suo.
    """
    dt, avanti = 0.1, 10.0
    passo = avanti * dt / (50.0 * 0.750)

    nodo = tracker_con_velivolo(avanti=avanti)
    try:
        for i in range(15):
            nodo.ultimo_istante = ora(nodo) - dt
            nodo.istante_vel_drone = ora(nodo)
            nodo.on_detection(Point(x=0.0, y=passo * i, z=300.0))
        vy = float(nodo.stato_stimato[3].item())
        assert abs(vy) < 0.05, (
            'il filtro attribuisce al bersaglio il moto del velivolo: %.3f' % vy)
    finally:
        nodo.destroy_node()

    # Controprova: senza conoscere il moto del velivolo, la stessa sequenza
    # produce una velocita pari allo scorrimento, cioe la diagnosi sbagliata.
    cieco = TrackerNode()
    try:
        for i in range(15):
            cieco.ultimo_istante = ora(cieco) - dt
            cieco.on_detection(Point(x=0.0, y=passo * i, z=300.0))
        vy = float(cieco.stato_stimato[3].item())
        assert vy > 0.1, 'controprova non significativa: %.3f' % vy
    finally:
        cieco.destroy_node()


def test_senza_moto_del_velivolo_la_velocita_non_si_pubblica():
    """Senza ingresso noto la velocita di stato torna relativa.

    E una grandezza diversa da quella promessa, e chi la legge non ha modo di
    accorgersene: meglio tacere che cambiare significato di nascosto.
    """
    nodo = TrackerNode()
    pubblicate = []
    nodo._pubblica_velocita = lambda valida: pubblicate.append(valida)
    try:
        nodo.on_detection(Point(x=0.1, y=0.1, z=100.0))
        nodo.ultimo_istante = ora(nodo) - 0.1
        nodo.on_detection(Point(x=0.12, y=0.1, z=100.0))
        assert pubblicate[-1] is False
    finally:
        nodo.destroy_node()


def test_misura_implausibile_viene_rifiutata():
    """Un valore anomalo va scartato, non pesato.

    Il disturbo non e gaussiano — il 30% dei messaggi e una perdita totale e il
    resto porta rumore con deviazione 0.3, quindici metri al suolo — e un
    filtro gaussiano senza rifiuto non ha modo di distinguere una misura
    sorprendente da una informativa: la media semplicemente.
    """
    nodo = tracker_con_velivolo()
    nis = []
    nodo.pub_nis = Raccoglitore()
    try:
        for i in range(20):
            nodo.ultimo_istante = ora(nodo) - 0.1
            nodo.istante_vel_drone = ora(nodo)
            nodo.on_detection(Point(x=0.0, y=0.0, z=300.0))
        prima = float(nodo.stato_stimato[0].item())

        nodo.ultimo_istante = ora(nodo) - 0.1
        nodo.istante_vel_drone = ora(nodo)
        nodo.on_detection(Point(x=0.9, y=0.0, z=300.0))   # salto impossibile
        dopo = float(nodo.stato_stimato[0].item())

        assert nodo.rifiuti_consecutivi == 1, 'la misura non e stata rifiutata'
        assert abs(dopo - prima) < 0.05, (
            'lo stato ha seguito il valore anomalo: %.3f -> %.3f' % (prima, dopo))
        nis = [m.data for m in nodo.pub_nis.messaggi]
        assert nis[-1] > nodo.soglia_gating, (
            'il NIS non segnala la sorpresa: %.2f' % nis[-1])
    finally:
        nodo.destroy_node()


def test_il_gating_scarta_un_valore_isolato_ma_segue_un_trasferimento():
    """Rifiutare una volta, poi seguire: e la differenza fra i due casi.

    Un valore anomalo isolato va scartato; un bersaglio che si e davvero
    spostato va inseguito. Il filtro distingue i due casi senza saperne nulla:
    rifiutando non corregge, quindi la covarianza cresce, e alla misura
    successiva il varco si e allargato abbastanza da lasciarla passare se
    insiste. Misurato: NIS 10.4 al primo salto, 8.7 al secondo con la soglia a
    9.21.
    """
    nodo = tracker_con_velivolo()
    try:
        for i in range(20):
            nodo.ultimo_istante = ora(nodo) - 0.1
            nodo.istante_vel_drone = ora(nodo)
            nodo.on_detection(Point(x=0.0, y=0.0, z=300.0))

        nodo.ultimo_istante = ora(nodo) - 0.1
        nodo.istante_vel_drone = ora(nodo)
        nodo.on_detection(Point(x=0.9, y=0.0, z=300.0))
        assert nodo.rifiuti_consecutivi == 1, 'il primo salto non e stato scartato'
        assert abs(float(nodo.stato_stimato[0].item())) < 0.05, (
            'lo stato ha seguito un valore isolato')

        for i in range(6):
            nodo.ultimo_istante = ora(nodo) - 0.1
            nodo.istante_vel_drone = ora(nodo)
            nodo.on_detection(Point(x=0.9, y=0.0, z=300.0))
        assert abs(float(nodo.stato_stimato[0].item()) - 0.9) < 0.15, (
            'il filtro non ha seguito un trasferimento che insiste: %.3f'
            % nodo.stato_stimato[0].item())
    finally:
        nodo.destroy_node()


def test_dopo_troppi_rifiuti_il_filtro_riacquisisce():
    """Rifiutare per sempre e il modo silenzioso di divergere.

    La crescita della covarianza di solito riapre il varco da sola, quindi
    questo paracadute non si vede quasi mai: per verificarlo si stringe la
    soglia al punto che nessuna crescita possa riaprirla. Senza, un filtro che
    scarta ogni misura resterebbe convinto di sapere dove sia il bersaglio, e
    nessuno se ne accorgerebbe.
    """
    nodo = tracker_con_velivolo()
    try:
        nodo.on_detection(Point(x=0.0, y=0.0, z=300.0))   # acquisizione
        nodo.soglia_gating = 1e-9                         # varco impossibile

        for i in range(nodo.max_rifiuti + 1):
            nodo.ultimo_istante = ora(nodo) - 0.1
            nodo.istante_vel_drone = ora(nodo)
            nodo.on_detection(Point(x=0.9, y=0.0, z=300.0))

        assert abs(float(nodo.stato_stimato[0].item()) - 0.9) < 1e-6, (
            'il filtro non e ripartito dalla misura')
        assert nodo.rifiuti_consecutivi == 0
    finally:
        nodo.destroy_node()


def test_matrice_Q_scala_con_dt():
    nodo = TrackerNode()
    try:
        import numpy as np

        Q1 = nodo._matrice_Q(0.1)
        Q2 = nodo._matrice_Q(0.2)
        # Termine di velocita lineare in dt, posizione con dt^3.
        assert abs(Q2[2, 2] / Q1[2, 2] - 2.0) < 1e-4
        assert abs(Q2[0, 0] / Q1[0, 0] - 8.0) < 1e-3
        # Termini incrociati presenti e simmetrici.
        assert Q1[0, 2] > 0
        assert abs(Q1[0, 2] - Q1[2, 0]) < 1e-9
        # Una matrice di covarianza non puo avere autovalori negativi.
        assert all(v >= -1e-9 for v in np.linalg.eigvalsh(Q1))
    finally:
        nodo.destroy_node()


def test_velocita_stimata_segue_il_moto_reale():
    """Con un bersaglio a velocita costante la stima deve avvicinarla.

    Lo smorzamento ad hoc rimosso (0.6 per ogni correzione) si componeva: dopo
    dieci aggiornamenti la velocita stimata era lo 0.6% di quella vera, e
    questa prova sarebbe fallita di due ordini di grandezza.
    """
    nodo = TrackerNode()
    try:
        passo = 0.05      # unita normalizzate per campione
        dt = 0.1          # secondi fra due campioni  -> 0.5 u/s
        for i in range(15):
            nodo.ultimo_istante = ora(nodo) - dt
            nodo.on_detection(Point(x=passo * i, y=0.0, z=300.0))
        vx = float(nodo.stato_stimato[2].item())
        assert 0.2 < vx < 0.8, 'velocita stimata fuori scala: %.3f' % vx
    finally:
        nodo.destroy_node()


def test_predizione_estrapola_durante_la_perdita():
    nodo = TrackerNode()
    try:
        for i in range(10):
            nodo.ultimo_istante = ora(nodo) - 0.1
            nodo.on_detection(Point(x=0.05 * i, y=0.0, z=300.0))
        prima = float(nodo.stato_stimato[0].item())
        nodo.ultimo_istante = ora(nodo) - 0.1
        nodo.on_detection(Point(x=0.0, y=0.0, z=0.0))   # segnale assente
        dopo = float(nodo.stato_stimato[0].item())
        assert dopo > prima, 'la predizione deve avanzare, non restare ferma'
    finally:
        nodo.destroy_node()


class Raccoglitore:
    """Publisher finto: raccoglie i messaggi invece di spedirli.

    Serve perche il comando al gimbal e un'uscita, e un'uscita si verifica
    leggendola: senza questo si potrebbe solo controllare che il nodo non
    sollevi eccezioni, che non e la stessa cosa.
    """

    def __init__(self):
        self.messaggi = []

    def publish(self, msg):
        self.messaggi.append(msg)


def gimbal_con_raccoglitori():
    nodo = GimbalNode()
    nodo.pub_roll = Raccoglitore()
    nodo.pub_pitch = Raccoglitore()
    return nodo


def test_gimbal_comanda_l_opposto_dell_assetto():
    """La rotazione del giunto deve annullare quella del corpo.

    Se il segno fosse invertito l'accoppiamento fra assetto e inquadratura
    raddoppierebbe invece di annullarsi, ed e un errore che in volo si vede
    solo come "il gimbal peggiora le cose".
    """
    nodo = gimbal_con_raccoglitori()
    try:
        nodo.roll = 0.20
        nodo.pitch = -0.15
        nodo.istante_posa = ora(nodo)
        nodo.comanda()
        assert abs(nodo.pub_roll.messaggi[-1].data + 0.20) < 1e-9
        assert abs(nodo.pub_pitch.messaggi[-1].data - 0.15) < 1e-9
    finally:
        nodo.destroy_node()


def test_gimbal_satura_al_limite_del_giunto():
    nodo = gimbal_con_raccoglitori()
    try:
        nodo.roll = 1.4          # ben oltre i 45 gradi del giunto
        nodo.pitch = -1.4
        nodo.istante_posa = ora(nodo)
        nodo.comanda()
        assert abs(nodo.pub_roll.messaggi[-1].data + nodo.limite_rad) < 1e-9
        assert abs(nodo.pub_pitch.messaggi[-1].data - nodo.limite_rad) < 1e-9
    finally:
        nodo.destroy_node()


def test_gimbal_disabilitato_comanda_zero():
    """Con la stabilizzazione spenta i giunti vanno tenuti a zero.

    E il modo in cui si ottiene la configurazione di riferimento: stesso
    velivolo, stesse masse, telecamera che si comporta come se fosse fissa.
    """
    from rclpy.parameter import Parameter

    nodo = gimbal_con_raccoglitori()
    try:
        nodo.set_parameters([Parameter('abilitato', Parameter.Type.BOOL, False)])
        nodo.roll = 0.3
        nodo.istante_posa = ora(nodo)
        nodo.comanda()
        assert nodo.pub_roll.messaggi[-1].data == 0.0
        assert nodo.pub_pitch.messaggi[-1].data == 0.0
    finally:
        nodo.destroy_node()


def test_gimbal_senza_posa_non_comanda():
    nodo = gimbal_con_raccoglitori()
    try:
        nodo.istante_posa = None
        nodo.comanda()
        assert nodo.pub_roll.messaggi == []

        # Posa vecchia di cinque secondi: vale come assente.
        nodo.istante_posa = ora(nodo) - 5.0
        nodo.comanda()
        assert nodo.pub_roll.messaggi == []
    finally:
        nodo.destroy_node()


def test_gnss_non_attacca_prima_dell_avvio():
    """Arming e decollo hanno bisogno di un fix valido.

    Negare il GPS a terra farebbe fallire il decollo per un motivo che non ha
    niente a che vedere con l'esperimento, e il fallimento sarebbe facile da
    attribuire al controllo invece che al banco di prova.
    """
    nodo = GnssDenialNode()
    try:
        nodo.fase_missione = FaseMissione.ATTESA.value
        nodo.aggiorna()
        assert nodo.attivo is False

        nodo.fase_missione = FaseMissione.PATTUGLIAMENTO.value
        nodo.aggiorna()
        assert nodo.attivo is True

        # Ritorno in ATTESA: l'attacco va rimosso, non lasciato attivo.
        nodo.fase_missione = FaseMissione.ATTESA.value
        nodo.aggiorna()
        assert nodo.attivo is False
    finally:
        nodo.destroy_node()


def test_gnss_ogni_modo_ha_un_valore_di_riposo_distinto():
    for modo, (nome, attacco, riposo) in MODI.items():
        assert nome.startswith('SIM_'), modo
        assert attacco != riposo, modo


def mission_in_aggancio():
    nodo = MissionNode()
    posa = PoseStamped()
    posa.pose.position.x = 20.0
    posa.pose.position.y = 20.0
    posa.pose.position.z = 50.0
    nodo.on_position(posa)
    nodo.fase = FaseMissione.AGGANCIO
    nodo.bersaglio_agganciato = True
    return nodo


def stima_bersaglio(x, y, vx, vy, valida=True):
    """Messaggio di stima come lo pubblica il controllo."""
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.twist.twist.linear.x = vx
    msg.twist.twist.linear.y = vy
    msg.twist.covariance[0] = 1.0 if valida else 0.0
    return msg


def rilevamento(visto):
    """Messaggio del rilevatore. La convenzione e l'area: z a zero, niente."""
    return Point(x=0.1, y=0.1, z=120.0 if visto else 0.0)


def test_riaggancio_non_scatta_sulla_predizione_del_filtro():
    """Un lampo di un fotogramma non deve valere un riaggancio.

    Misurato: con la ricerca direzionale il drone arriva abbastanza vicino da
    far comparire il bersaglio nell'angolo dell'inquadratura per uno o due
    fotogrammi. Il filtro resta poi valido per `soglia_perdita` fotogrammi
    anche senza altre misure, e la missione, contando quelli, dichiarava il
    riaggancio: ne uscivano agganci di quattro secondi con il rilevatore che
    non vedeva nulla in nessun campione, e per tutta la loro durata la
    missione smetteva di cercare.
    """
    nodo = mission_in_aggancio()
    try:
        nodo.fase = FaseMissione.RICERCA
        nodo.bersaglio_agganciato = False

        # Il tracker pubblica posizioni valide — sono predizioni, ma da fuori
        # non si distinguono — e nessun rilevamento le sostiene.
        for _ in range(20):
            nodo.on_target(Point(x=0.1, y=0.1, z=120.0))
        assert nodo.fase == FaseMissione.RICERCA, (
            'riagganciato senza un solo rilevamento')
    finally:
        nodo.destroy_node()


def test_riaggancio_scatta_sui_rilevamenti_veri():
    """Con rilevamenti consecutivi veri il riaggancio deve invece scattare."""
    nodo = mission_in_aggancio()
    try:
        nodo.fase = FaseMissione.RICERCA
        nodo.bersaglio_agganciato = False

        for _ in range(nodo.frame_conferma_riaggancio - 1):
            nodo.on_rilevamento(rilevamento(True))
        nodo.on_target(Point(x=0.1, y=0.1, z=120.0))
        assert nodo.fase == FaseMissione.RICERCA, 'un rilevamento in meno basta'

        nodo.on_rilevamento(rilevamento(True))
        nodo.on_target(Point(x=0.1, y=0.1, z=120.0))
        assert nodo.fase == FaseMissione.AGGANCIO
    finally:
        nodo.destroy_node()


def test_i_rilevamenti_devono_essere_consecutivi():
    """Un buco azzera il conteggio: due lampi separati non fanno un aggancio."""
    nodo = mission_in_aggancio()
    try:
        nodo.fase = FaseMissione.RICERCA
        nodo.bersaglio_agganciato = False

        nodo.on_rilevamento(rilevamento(True))
        nodo.on_rilevamento(rilevamento(False))
        nodo.on_rilevamento(rilevamento(True))
        nodo.on_target(Point(x=0.1, y=0.1, z=120.0))
        assert nodo.fase == FaseMissione.RICERCA
    finally:
        nodo.destroy_node()


def test_rilevamenti_a_rilevamento_disattivato_non_si_accumulano():
    """Il conteggio non deve sopravvivere alla porta chiusa.

    Altrimenti i rilevamenti arrivati mentre il rilevamento era ancora
    disattivato si sommerebbero, e l'aggancio scatterebbe nell'istante esatto
    in cui la porta si apre, senza aver visto nulla da quel momento.
    """
    nodo = MissionNode()
    try:
        posa = PoseStamped()
        posa.pose.position.z = 50.0
        nodo.on_position(posa)
        nodo.fase = FaseMissione.PATTUGLIAMENTO
        nodo.rilevamento_attivo = False

        for _ in range(20):
            nodo.on_rilevamento(rilevamento(True))
            nodo.on_target(Point(x=0.1, y=0.1, z=120.0))
        assert nodo.fase == FaseMissione.PATTUGLIAMENTO
        assert nodo.rilevamenti_consecutivi == 0
    finally:
        nodo.destroy_node()


def test_ricerca_parte_dalla_posizione_del_bersaglio():
    """Il centro della ricerca e dove era il BERSAGLIO, non dove era il drone.

    A velocita reali le due posizioni differiscono di decine di metri, e
    cercare attorno a se stessi significa cercare dove il bersaglio non e.
    """
    nodo = mission_in_aggancio()
    try:
        nodo.on_stima_bersaglio(stima_bersaglio(200.0, 100.0, 15.0, 0.0))
        nodo.istante_ultimo_target = ora(nodo) - 10.0
        nodo.istante_perdita = ora(nodo) - nodo.soglia_avvia_ricerca_s - 1.0
        nodo.istante_ultima_posa = ora(nodo)
        nodo.aggiorna_missione()

        assert nodo.fase == FaseMissione.RICERCA
        # Il drone era a (20, 20): il centro deve essere quello del bersaglio.
        assert abs(nodo.ricerca_centro_x - 200.0) < 1.0
        assert abs(nodo.ricerca_centro_y - 100.0) < 1.0
    finally:
        nodo.destroy_node()


def test_inseguimento_cieco_va_dove_il_bersaglio_stava_andando():
    """Acceso, il primo tempo della ricerca extrapola il moto.

    Il default e zero, cioe spento, perche misurato non conviene: l'errore
    della velocita stimata vale quanto la velocita stessa. La prova lo accende
    esplicitamente, come quella sull'anticipo, altrimenti passerebbe senza
    verificare nulla.
    """
    from rclpy.parameter import Parameter

    nodo = mission_in_aggancio()
    try:
        nodo.set_parameters([Parameter(
            'durata_inseguimento_cieco_s', Parameter.Type.DOUBLE, 8.0)])
        adesso = ora(nodo)
        nodo.on_stima_bersaglio(stima_bersaglio(200.0, 100.0, 15.0, 0.0))
        nodo.fase = FaseMissione.RICERCA
        nodo.istante_inizio_ricerca = adesso
        # Quattro secondi di estrapolazione a 15 m/s: sessanta metri avanti.
        nodo.stima_bersaglio = (200.0, 100.0, 15.0, 0.0, adesso - 4.0)
        nodo.esegui_ricerca()

        # Il waypoint pubblicato non e ispezionabile da qui, ma il centro
        # della spirale viene aggiornato alla stessa posizione extrapolata,
        # quindi verificarlo verifica l'estrapolazione.
        assert abs(nodo.ricerca_centro_x - 260.0) < 2.0, nodo.ricerca_centro_x
        assert abs(nodo.ricerca_centro_y - 100.0) < 2.0
    finally:
        nodo.destroy_node()


def test_senza_velocita_stimata_non_si_extrapola():
    """Se la velocita non e ricostruibile, l'estrapolazione non deve inventare.

    Volare verso l'ultima posizione nota e cio che faceva la spirale: senza la
    bandiera di validita il primo tempo della ricerca sarebbe indistinguibile
    dal secondo, e sembrerebbe funzionare senza fare nulla.
    """
    nodo = mission_in_aggancio()
    try:
        nodo.on_stima_bersaglio(
            stima_bersaglio(200.0, 100.0, 15.0, 0.0, valida=False))
        assert nodo.stima_bersaglio[2] == 0.0
        assert nodo.stima_bersaglio[3] == 0.0
    finally:
        nodo.destroy_node()


def test_mission_passa_a_ricerca_senza_messaggi():
    """Il silenzio del tracker non deve congelare la fase AGGANCIO.

    Prima il conteggio della perdita viveva solo nella callback dei messaggi:
    se il detector moriva, non arrivava nessun messaggio a farlo partire.
    """
    nodo = mission_in_aggancio()
    try:
        nodo.istante_ultimo_target = ora(nodo) - 10.0
        nodo.aggiorna_missione()
        assert nodo.istante_perdita is not None

        nodo.istante_perdita = ora(nodo) - nodo.soglia_avvia_ricerca_s - 1.0
        nodo.istante_ultima_posa = ora(nodo)
        nodo.aggiorna_missione()
        assert nodo.fase == FaseMissione.RICERCA
    finally:
        nodo.destroy_node()


def test_mission_non_rinuncia_se_il_bersaglio_e_visibile():
    nodo = mission_in_aggancio()
    try:
        nodo.istante_perdita = ora(nodo) - 100.0
        nodo.on_target(Point(x=0.1, y=0.1, z=300.0))
        assert nodo.istante_perdita is None
        assert nodo.fase == FaseMissione.AGGANCIO
    finally:
        nodo.destroy_node()
