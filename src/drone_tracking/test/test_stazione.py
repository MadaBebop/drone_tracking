"""Prove sul sensore simulato della stazione a terra.

Il sensore e logica pura, senza ROS: si verifica che ritardo, portata, perdite
ed errore siano quelli dichiarati, perche sono proprio i numeri che al pitch
vanno presentati come ipotesi, e un'ipotesi implementata male la falsifica.

    colcon test --packages-select drone_tracking
"""
import math
import statistics

from drone_tracking.stazione_node import (
    ModelloSensore, stima_velocita, velocita_significativa)


def sensore(**kw):
    valori = {'sigma_m': 0.0, 'ritardo_s': 1.0, 'prob_perdita': 0.0,
              'portata_m': 400.0, 'seme': 1}
    valori.update(kw)
    return ModelloSensore(**valori)


def test_lallerta_descrive_il_passato_di_quanto_vale_il_ritardo():
    s = sensore(ritardo_s=1.5)
    for i in range(101):                      # bersaglio a 2 m/s lungo x
        t = i * 0.1
        s.registra_verita(t, 100.0 + 2.0 * t, 50.0)
    istante, x, y = s.allerta(10.0)
    assert istante == 8.5
    assert math.isclose(x, 100.0 + 2.0 * 8.5, abs_tol=1e-9)
    assert math.isclose(y, 50.0, abs_tol=1e-9)


def test_la_verita_fra_due_campioni_e_interpolata():
    s = sensore(ritardo_s=0.0)
    s.registra_verita(0.0, 0.0, 0.0)
    s.registra_verita(1.0, 10.0, 20.0)
    assert s.verita_a(0.25) == (2.5, 5.0)


def test_senza_una_verita_abbastanza_vecchia_non_ce_allerta():
    s = sensore(ritardo_s=1.0)
    s.registra_verita(5.0, 100.0, 100.0)
    assert s.allerta(5.5) is None             # servirebbe la verita a 4.5


def test_oltre_la_portata_la_stazione_non_vede_nulla():
    s = sensore(ritardo_s=0.0, portata_m=400.0)
    s.registra_verita(0.0, 300.0, 300.0)      # 424 m dalla stazione
    assert s.allerta(0.0) is None
    s.registra_verita(1.0, 200.0, 200.0)      # 283 m
    assert s.allerta(1.0) is not None


def test_le_perdite_seguono_la_probabilita_dichiarata():
    s = sensore(ritardo_s=0.0, prob_perdita=0.3)
    s.registra_verita(0.0, 10.0, 10.0)
    arrivate = sum(s.allerta(0.0) is not None for _ in range(2000))
    assert 0.65 < arrivate / 2000 < 0.75


def test_lerrore_ha_la_deviazione_standard_dichiarata():
    s = sensore(ritardo_s=0.0, sigma_m=20.0)
    s.registra_verita(0.0, 100.0, -50.0)
    allerte = [s.allerta(0.0) for _ in range(3000)]
    assert 18.0 < statistics.pstdev(a[1] for a in allerte) < 22.0
    assert 18.0 < statistics.pstdev(a[2] for a in allerte) < 22.0
    assert abs(statistics.mean(a[1] for a in allerte) - 100.0) < 1.5


def test_la_velocita_di_un_moto_rettilineo_e_esatta():
    allerte = [(t, 10.0 + 3.0 * t, 5.0 - 2.0 * t) for t in (0.0, 2.0, 4.0, 6.0)]
    vx, vy = stima_velocita(allerte)
    assert math.isclose(vx, 3.0) and math.isclose(vy, -2.0)


def test_con_meno_di_tre_allerte_la_velocita_non_si_stima():
    assert stima_velocita([(0.0, 0.0, 0.0), (2.0, 10.0, 0.0)]) is None


def test_un_bersaglio_fermo_non_sembra_veloce():
    """Il rumore dell'allerta non diventa una velocita fittizia.

    Con 20 m di errore la differenza finita fra due allerte a 2 s darebbe
    velocita dell'ordine di 14 m/s a un bersaglio fermo: la retta su cinque
    punti deve stare molto sotto.
    """
    s = sensore(ritardo_s=0.0, sigma_m=20.0, seme=7)
    s.registra_verita(0.0, 0.0, 0.0)
    errori = []
    for prova in range(200):
        allerte = [(t, *s.allerta(0.0)[1:]) for t in (0.0, 2.0, 4.0, 6.0, 8.0)]
        vx, vy = stima_velocita(allerte)
        errori.append(math.hypot(vx, vy))
    assert statistics.median(errori) < 5.0


def allerte_rumorose(velocita, seme, n=5, sigma=20.0):
    s = sensore(ritardo_s=0.0, sigma_m=sigma, seme=seme)
    allerte = []
    for k in range(n):
        t = 2.0 * k
        s.registra_verita(t, 100.0 + velocita * t, 100.0)
        allerte.append(s.allerta(t))
    return allerte


def test_la_velocita_di_un_bersaglio_fermo_non_e_significativa():
    """Su cento prove, quasi mai: la soglia a due sigma sbaglia una volta su sette."""
    significative = sum(
        velocita_significativa(allerte_rumorose(0.0, seme), 20.0) is not None
        for seme in range(100))
    assert significative < 20


def test_la_velocita_di_un_bersaglio_veloce_e_significativa():
    risultati = [velocita_significativa(allerte_rumorose(15.0, seme), 20.0)
                 for seme in range(100)]
    validi = [r for r in risultati if r is not None]
    assert len(validi) > 95
    assert abs(statistics.median(r[0] for r in validi) - 15.0) < 2.0
