"""Prove sulle metriche della stazione: colonne del CSV e riepilogo della catena.

Si eseguono con le altre:

    colcon test --packages-select drone_tracking
"""
import csv
import importlib.util
import os

from drone_tracking.metrics_node import COLONNE, MetricsNode
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String


@pytest.fixture(scope='module', autouse=True)
def contesto_ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def metriche(tmp_path):
    """Un MetricsNode che scrive in una cartella temporanea.

    Il costruttore apre subito un file nella cartella di default: lo si chiude
    e lo si cancella, perche una prova automatica non deve lasciare CSV vuoti
    fra le misure vere.
    """
    nodo = MetricsNode()
    primo = nodo.percorso
    nodo.set_parameters([Parameter('cartella_output', value=str(tmp_path))])
    nodo._nuova_prova()
    if os.path.exists(primo) and os.path.getsize(primo) < 2000:
        os.remove(primo)
    yield nodo
    nodo.destroy_node()


def allerta(nodo, x, y, sigma=20.0, eta=1.0):
    msg = PoseWithCovarianceStamped()
    istante = nodo._ora_sim() - eta
    msg.header.stamp.sec = int(istante)
    msg.header.stamp.nanosec = int((istante - int(istante)) * 1e9)
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    msg.pose.covariance[0] = sigma ** 2
    return msg


def righe_scritte(nodo):
    nodo.file.flush()
    with open(nodo.percorso, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def test_senza_stazione_le_colonne_restano_vuote(metriche):
    metriche.campiona()
    riga = righe_scritte(metriche)[-1]
    assert riga['all_x'] == '' and riga['t_prima_allerta'] == ''
    assert len(riga) == len(COLONNE)


def test_lallerta_e_la_catena_finiscono_nel_csv(metriche):
    metriche.gt_bersaglio = (106.0, 108.0, 0.8)
    metriche.on_allerta(allerta(metriche, 100.0, 100.0))
    metriche.on_stato_stazione(String(data='CONFERMA'))
    metriche.on_stato_stazione(String(data='LANCIO'))
    stima = Odometry()
    stima.twist.twist.linear.x = 3.0
    stima.twist.covariance[0] = 1.0
    metriche.on_stima_stazione(stima)
    metriche.campiona()
    metriche.campiona()
    prima, seconda = righe_scritte(metriche)[-2:]

    assert float(prima['all_x']) == 100.0
    assert float(prima['all_scarto']) == 10.0      # 6-8-10 dalla verita
    assert float(prima['all_sigma']) == 20.0
    assert float(prima['all_vx']) == 3.0
    assert 0.9 < float(prima['all_eta']) < 1.5
    assert prima['staz_stato'] == 'LANCIO'
    assert prima['t_prima_allerta'] != '' and prima['t_lancio'] != ''
    assert prima['t_in_volo'] == ''
    # Ogni allerta conta una volta sola, anche se resta in molte righe.
    assert prima['all_nuova'] == '1' and seconda['all_nuova'] == '0'


def test_gli_istanti_della_catena_sopravvivono_alla_rotazione(metriche):
    """Lancio e prima allerta accadono prima che il file della prova si apra."""
    metriche.on_allerta(allerta(metriche, 100.0, 100.0))
    metriche.on_stato_stazione(String(data='LANCIO'))
    metriche.campiona()
    metriche.on_stato(String(data='TRASFERIMENTO'))   # la missione parte
    metriche.campiona()
    riga = righe_scritte(metriche)[-1]
    assert riga['t_prima_allerta'] != '' and riga['t_lancio'] != ''


def script_metriche():
    """metriche.py sta fuori dal pacchetto: nel repository e nel container."""
    qui = os.path.dirname(os.path.abspath(__file__))
    for percorso in (os.path.join(qui, '..', '..', '..', 'scripts', 'metriche.py'),
                     '/usr/local/share/drone_tracking/scripts/metriche.py'):
        if os.path.exists(percorso):
            spec = importlib.util.spec_from_file_location('metriche', percorso)
            modulo = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(modulo)
            return modulo
    pytest.skip('metriche.py non trovato')


def test_il_riepilogo_ricostruisce_i_tempi_della_catena():
    m = script_metriche()

    def riga(t, fase, nuova='0', scarto=''):
        return {'t_sim': str(t), 'fase': fase, 'all_nuova': nuova,
                'all_scarto': scarto, 'all_eta': '1.0' if nuova == '1' else '',
                't_prima_allerta': '10.0', 't_lancio': '14.0', 't_in_volo': '40.0'}

    righe = [riga(42.0, 'TRASFERIMENTO', '1', '25'),
             riga(44.0, 'TRASFERIMENTO'),
             riga(46.0, 'TRASFERIMENTO', '1', '15'),
             riga(57.0, 'AGGANCIO')]
    c = m.catena_stazione(righe)
    assert c['allerta_lancio'] == 4.0
    assert c['lancio_in_volo'] == 26.0
    assert c['allerta_avvio'] == 32.0
    assert c['avvio_aggancio'] == 15.0
    assert c['allerta_aggancio'] == 47.0
    assert c['fase_aggancio'] == 'TRASFERIMENTO'
    assert c['scarti'] == [25.0, 15.0]


def test_senza_stazione_il_riepilogo_non_inventa_la_catena():
    m = script_metriche()
    assert m.catena_stazione([{'t_sim': '0', 'fase': 'PATTUGLIAMENTO'}]) is None
