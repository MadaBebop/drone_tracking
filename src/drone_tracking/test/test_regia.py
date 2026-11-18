"""Prove sul registratore di regia: filtro della console, accelerazione, mappa.

registra_regia.py sta fuori dal pacchetto, come metriche.py: si carica dal
repository o dal container. Le parti provate sono pure, senza simulatore.
"""
import importlib.util
import math
import os

import pytest


def script_regia():
    qui = os.path.dirname(os.path.abspath(__file__))
    for percorso in (os.path.join(qui, '..', '..', '..', 'scripts', 'registra_regia.py'),
                     '/usr/local/share/drone_tracking/scripts/registra_regia.py'):
        if os.path.exists(percorso):
            spec = importlib.util.spec_from_file_location('registra_regia', percorso)
            modulo = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(modulo)
            return modulo
    pytest.skip('registra_regia.py non trovato')


def test_la_console_tiene_i_nodi_e_scarta_il_rumore_di_mavros():
    r = script_regia()
    assert r.riga_console('stazione_node', r.INFO, 'Lancio') == 'Lancio'
    assert r.riga_console('mission_node', 10, 'debug') is None
    # La sincronizzazione dei parametri di MAVROS non dice nulla del volo.
    assert r.riga_console('mavros.param', r.WARN, 'PR: 1382 params still missing') is None
    assert r.riga_console('mavros.cmd', r.WARN, 'CMD: Unexpected command 511') is None
    # I messaggi dell'autopilota e la perdita di collegamento invece si.
    assert r.riga_console('mavros.sys', r.INFO, 'FCU: Arming motors') == 'FCU: Arming motors'
    assert r.riga_console('mavros', r.WARN, 'CON: Lost connection') == 'CON: Lost connection'


def test_la_console_raggruppa_i_messaggi_dello_stesso_tipo():
    r = script_regia()
    righe = []
    r.aggiungi_riga(righe, 1.0, r.INFO, 'mission', 'RICERCA spirale -> (1, 2) raggio:10m')
    r.aggiungi_riga(righe, 1.5, r.INFO, 'stazione', 'Allerta: bersaglio a (5, 5)')
    r.aggiungi_riga(righe, 2.0, r.INFO, 'mission', 'RICERCA spirale -> (3, 4) raggio:12m')
    # La spirale occupa una riga sola, aggiornata, in fondo, con il conteggio.
    assert len(righe) == 2
    assert righe[-1][3].endswith('raggio:12m') and righe[-1][4] == 2
    assert righe[0][2] == 'stazione'
    # Messaggi diversi dello stesso nodo restano separati.
    r.aggiungi_riga(righe, 3.0, r.WARN, 'tracker', 'Bersaglio perso - reset tracker')
    r.aggiungi_riga(righe, 3.5, r.INFO, 'tracker', 'Bersaglio acquisito')
    assert len(righe) == 4
    for k in range(20):
        r.aggiungi_riga(righe, 4.0 + k, r.INFO, 'n{}'.format(k), 'messaggio')
    assert len(righe) == r.RIGHE_CONSOLE


def test_il_testo_diventa_ascii_disegnabile():
    r = script_regia()
    assert r.leggibile('Stazione avviata — perché') == 'Stazione avviata - perche'


def test_si_accelera_solo_lontano_dagli_eventi_e_fuori_dall_aggancio():
    r = script_regia()
    tempi = [0.0, 10.0, 20.0, 30.0, 40.0]
    fasi = ['ATTESA', 'TRASFERIMENTO', 'TRASFERIMENTO', 'AGGANCIO', 'TRASFERIMENTO']
    # Un evento a 21 s: il fotogramma a 20 s e entro i 2 s che lo precedono.
    v = r.velocita_riproduzione(tempi, fasi, [21.0], 4.0)
    assert v == [4.0, 4.0, 1.0, 1.0, 4.0]
    # E un fotogramma entro i 5 s successivi resta a velocita normale.
    assert r.velocita_riproduzione([25.0], ['TRASFERIMENTO'], [21.0], 4.0) == [1.0]
    # La ricerca lontana dagli eventi va al doppio, mai piu veloce di `calma`.
    assert r.velocita_riproduzione([60.0], ['RICERCA'], [21.0], 4.0) == [2.0]
    assert r.velocita_riproduzione([60.0], ['RICERCA'], [21.0], 1.0) == [1.0]


def test_il_piano_rispetta_il_tempo_simulato_diviso_la_velocita():
    r = script_regia()
    tempi = [k * 0.125 for k in range(80)]          # 10 s simulati a 8 campioni/s
    normale = r.piano_uscita(tempi, [1.0] * 80, 16)
    veloce = r.piano_uscita(tempi, [4.0] * 80, 16)
    assert len(normale) == 160                     # 10 s di video a 16 fps
    assert len(veloce) == 40                       # 2,5 s: quattro volte piu corto
    assert normale == sorted(normale)


def test_un_buco_della_simulazione_non_diventa_una_pausa():
    r = script_regia()
    piano = r.piano_uscita([0.0, 100.0, 100.125], [1.0, 1.0, 1.0], 16)
    assert piano.count(0) <= 16                    # al massimo un secondo


def test_la_mappa_ha_il_nord_in_alto_e_la_stazione_al_centro():
    r = script_regia()
    m = r.Mappa(640, 480, raggio_m=560.0)
    assert m.px(0, 0) == (320, 240)
    x_nord, y_nord = m.px(0, 100)
    assert x_nord == 320 and y_nord < 240
    # Il confine dei 500 m entra nel pannello.
    _, y_bordo = m.px(0, -500)
    assert y_bordo < 480


def test_la_vista_si_allarga_solo_quando_serve():
    r = script_regia()
    # Tutto vicino alla stazione: vista stretta, l'azione riempie il pannello.
    assert r.Mappa.raggio_utile([(100.0, 50.0)]) == 220.0
    # Il drone a 400 m: la vista lo contiene, con margine.
    assert r.Mappa.raggio_utile([(400.0, 0.0)]) > 400.0
    # Mai oltre il confine con margine, anche se un'allerta sbaglia di molto.
    assert r.Mappa.raggio_utile([(2000.0, 0.0)]) == 560.0


def test_imbardata_da_quaternione():
    r = script_regia()

    class Q:
        x = y = 0.0
        z = math.sin(math.pi / 4)
        w = math.cos(math.pi / 4)
    assert math.isclose(r.imbardata(Q), math.pi / 2, abs_tol=1e-9)
