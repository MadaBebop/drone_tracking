"""Prove sulla fase TRASFERIMENTO: dall'allerta della stazione alla ricerca.

Il calcolo del punto d'incontro e geometria pura; il resto verifica le
decisioni della missione senza far volare nulla, come in test_logica_nodi.

    colcon test --packages-select drone_tracking
"""
import math

from drone_tracking.mission_node import FaseMissione, MissionNode, punto_incontro
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from std_msgs.msg import Bool


@pytest.fixture(scope='module', autouse=True)
def contesto_ros():
    rclpy.init()
    yield
    rclpy.shutdown()


# ------------------------------------------------------------ punto d'incontro

def test_bersaglio_fermo_si_va_dove_e():
    x, y, t = punto_incontro((300.0, 400.0), (0.0, 0.0), (0.0, 0.0), 10.0)
    assert (x, y) == (300.0, 400.0)
    assert math.isclose(t, 50.0)


def test_bersaglio_che_attraversa_si_anticipa():
    x, y, t = punto_incontro((100.0, 0.0), (0.0, 10.0), (0.0, 0.0), 20.0)
    assert math.isclose(t, math.sqrt(10000.0 / 300.0))
    assert math.isclose(x, 100.0) and math.isclose(y, 10.0 * t)
    # Il drone ci arriva esattamente in t: e la definizione di incontro.
    assert math.isclose(math.hypot(x, y), 20.0 * t)


def test_bersaglio_piu_veloce_in_fuga_si_insegue_comunque():
    x, y, t = punto_incontro((100.0, 0.0), (30.0, 0.0), (0.0, 0.0), 15.0)
    assert math.isclose(t, 100.0 / 15.0)
    assert math.isclose(x, 100.0 + 30.0 * t) and y == 0.0


def test_lanticipo_ha_un_limite():
    x, _, t = punto_incontro((1000.0, 0.0), (9.0, 0.0), (0.0, 0.0), 10.0,
                             orizzonte_s=30.0)
    assert t == 30.0
    assert math.isclose(x, 1000.0 + 9.0 * 30.0)


# ------------------------------------------------------------------ missione

def ora(nodo):
    return nodo.get_clock().now().nanoseconds / 1e9


def allerta(nodo, x, y, sigma=20.0, eta=0.0):
    msg = PoseWithCovarianceStamped()
    istante = ora(nodo) - eta
    msg.header.stamp.sec = int(istante)
    msg.header.stamp.nanosec = int((istante - int(istante)) * 1e9)
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    msg.pose.covariance[0] = msg.pose.covariance[7] = sigma ** 2
    return msg


def stima(vx, vy, valida=True):
    msg = Odometry()
    msg.twist.twist.linear.x, msg.twist.twist.linear.y = float(vx), float(vy)
    msg.twist.covariance[0] = 1.0 if valida else 0.0
    return msg


def in_volo(nodo, x=0.0, y=0.0, z=50.0):
    posa = PoseStamped()
    posa.pose.position.x, posa.pose.position.y, posa.pose.position.z = x, y, z
    nodo.on_position(posa)


def registra_waypoint(nodo):
    registro = []
    nodo.pubblica_waypoint = registro.append
    return registro


@pytest.fixture
def nodo():
    n = MissionNode()
    yield n
    n.destroy_node()


def test_con_unallerta_si_parte_in_trasferimento(nodo):
    nodo.on_allerta(allerta(nodo, 200.0, 150.0))
    nodo.on_avvia(Bool(data=True))
    assert nodo.fase == FaseMissione.TRASFERIMENTO


def test_senza_allerta_la_missione_e_quella_di_sempre(nodo):
    nodo.on_avvia(Bool(data=True))
    assert nodo.fase == FaseMissione.PATTUGLIAMENTO


def test_unallerta_vecchia_non_conta(nodo):
    nodo.on_allerta(allerta(nodo, 200.0, 150.0, eta=nodo.validita_allerta_s + 1.0))
    nodo.on_avvia(Bool(data=True))
    assert nodo.fase == FaseMissione.PATTUGLIAMENTO


def test_il_trasferimento_punta_sullallerta(nodo):
    in_volo(nodo)
    registro = registra_waypoint(nodo)
    nodo.on_allerta(allerta(nodo, 200.0, 150.0))
    nodo.on_avvia(Bool(data=True))
    nodo.aggiorna_missione()
    x, y, z = registro[-1]
    assert (round(x), round(y)) == (200, 150)
    assert z == nodo.waypoints[1][2]


def test_con_poche_allerte_la_velocita_non_si_usa(nodo):
    """Il rumore di tre allerte non deve spostare il punto di decine di metri."""
    in_volo(nodo)
    registro = registra_waypoint(nodo)
    for _ in range(nodo.minimo_allerte_velocita - 1):
        nodo.on_allerta(allerta(nodo, 200.0, 150.0))
    nodo.on_stima_stazione(stima(10.0, 0.0))
    nodo.on_avvia(Bool(data=True))
    nodo.aggiorna_missione()
    assert round(registro[-1][0]) == 200


def test_con_abbastanza_allerte_si_anticipa(nodo):
    in_volo(nodo)
    registro = registra_waypoint(nodo)
    for _ in range(nodo.minimo_allerte_velocita):
        nodo.on_allerta(allerta(nodo, 200.0, 150.0))
    nodo.on_stima_stazione(stima(10.0, 0.0))
    nodo.on_avvia(Bool(data=True))
    nodo.aggiorna_missione()
    assert registro[-1][0] > 250.0, 'il punto deve stare avanti al bersaglio'


def test_arrivati_sullallerta_si_cerca_attorno_al_bersaglio(nodo):
    in_volo(nodo, x=195.0, y=148.0)
    registra_waypoint(nodo)
    nodo.on_allerta(allerta(nodo, 200.0, 150.0))
    nodo.on_avvia(Bool(data=True))
    nodo.aggiorna_missione()
    assert nodo.fase == FaseMissione.RICERCA
    assert nodo.ricerca_da_allerta
    assert (round(nodo.ricerca_centro_x), round(nodo.ricerca_centro_y)) == (200, 150)


def test_lallerta_scaduta_in_volo_riporta_alla_pattuglia(nodo):
    in_volo(nodo)
    registra_waypoint(nodo)
    nodo.on_allerta(allerta(nodo, 200.0, 150.0))
    nodo.on_avvia(Bool(data=True))
    nodo.allerta = (ora(nodo) - nodo.validita_allerta_s - 1.0, 200.0, 150.0, 20.0)
    nodo.aggiorna_missione()
    assert nodo.fase == FaseMissione.PATTUGLIAMENTO


def rilevamento(visto=True):
    m = PointStamped()
    m.point.x, m.point.y, m.point.z = 0.1, 0.1, 120.0 if visto else 0.0
    return m


def test_nel_trasferimento_si_aggancia_appena_lo_si_vede(nodo):
    in_volo(nodo)
    nodo.on_allerta(allerta(nodo, 200.0, 150.0))
    nodo.on_avvia(Bool(data=True))
    nodo.rilevamento_attivo = True
    for _ in range(nodo.frame_conferma_richiesti):
        nodo.on_rilevamento(rilevamento())
    nodo.on_target(rilevamento())
    assert nodo.fase == FaseMissione.AGGANCIO


def test_la_ricerca_da_allerta_chiede_la_conferma_piena(nodo):
    """Il primo aggancio vale la soglia della pattuglia, non quella ridotta."""
    in_volo(nodo, x=200.0, y=150.0)
    nodo.fase = FaseMissione.RICERCA
    nodo.ricerca_da_allerta = True
    for _ in range(nodo.frame_conferma_riaggancio):
        nodo.on_rilevamento(rilevamento())
    nodo.on_target(rilevamento())
    assert nodo.fase == FaseMissione.RICERCA, 'agganciato con la soglia ridotta'
    for _ in range(nodo.frame_conferma_richiesti - nodo.frame_conferma_riaggancio):
        nodo.on_rilevamento(rilevamento())
    nodo.on_target(rilevamento())
    assert nodo.fase == FaseMissione.AGGANCIO
    assert not nodo.ricerca_da_allerta
