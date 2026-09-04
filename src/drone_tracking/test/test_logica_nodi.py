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
    nodo.on_altitudine(Float64(data=12.0))
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
    posa.pose.position.z = 12.0
    nodo.on_position(posa)
    nodo.fase = FaseMissione.AGGANCIO
    nodo.bersaglio_agganciato = True
    return nodo


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
