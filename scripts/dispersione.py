#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Quanto rumore c'e fra prove identiche, e quale differenza si puo distinguere.

Il README dichiara che le grandezze d'esito di questo progetto cadono "dentro
la dispersione" e cita un fattore due-quattro fra prove identiche. E una stima
a occhio, fatta guardando le tabelle. Questo script la calcola.

Il metodo e diretto. Le prove ripetute portano il suffisso `_g1`, `_g2`, `_g3`:
stessa configurazione, stesso seme del disturbo, stesso scenario. Tutto cio che
le separa e il rumore del protocollo — scheduling, carico della macchina,
interazioni fra bersaglio reattivo e inseguitore. Misurare lo scarto DENTRO
quei gruppi da la dispersione; confrontarla con la differenza che si vuole
osservare dice se un confronto ha senso prima di spendere ore di macchina.

    python3 scripts/dispersione.py                  # tutti i gruppi _gN
    python3 scripts/dispersione.py 'metrics/*_g*.csv'

Il numero che conta e l'ultimo: la differenza minima distinguibile con n prove
per configurazione. Se e piu grande dell'effetto che cerchi, la batteria non
puo rispondere alla domanda, e lanciarla comunque produce un numero che sembra
un risultato senza esserlo.
"""
import glob
import math
import os
import re
import sys
from statistics import median, mean, stdev

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import metriche  # noqa: E402  (il percorso va sistemato prima dell'import)


# Quantili della t di Student, per non dipendere da scipy. Righe: gradi di
# liberta 2(n-1) per n prove per configurazione.
T_975 = {2: 4.303, 4: 2.776, 6: 2.447, 8: 2.306, 10: 2.228, 12: 2.179}
T_800 = {2: 1.886, 4: 1.533, 6: 1.440, 8: 1.397, 10: 1.372, 12: 1.356}

# Durata di una prova sul banco attuale, avvio dello stack compreso: serve a
# tradurre "quante prove servono" in "quante ore di macchina", che e la forma
# in cui la domanda si decide davvero.
MINUTI = 9

INDICATORI = (
    ('tempo in AGGANCIO', '%'),
    ('visto in AGGANCIO', '%'),
    ('distanza in AGGANCIO', 'm'),
    ('durata dell aggancio', 's'),
    ('errore di velocita', 'm/s'),
)


def errore_velocita(righe):
    """Mediana dell'errore sulla stima di velocita, o None.

    Stessa definizione del comando `stima`, compreso il rifiuto delle prove
    precedenti all'ingresso noto: li `trk_vx/vy` erano velocita relative al
    velivolo e il confronto produrrebbe un numero sbagliato in silenzio.
    """
    if not righe or 'nis' not in righe[0]:
        return None
    vero = metriche.velocita_vera(righe)
    err = []
    for r in righe:
        stimata = metriche.velocita_stimata(r)
        v = vero.get(r['t_sim'])
        if stimata is None or v is None:
            continue
        err.append(math.hypot(stimata[0] - v[0], stimata[1] - v[1]))
    return median(err) if len(err) >= 20 else None


def misura_prova(percorso):
    """I cinque indicatori di una prova. None dove non sono calcolabili."""
    ind = metriche._indicatori(percorso)
    if ind is None:
        return None
    righe = metriche.leggi(percorso)
    durate = ind['agganci']
    return {
        'tempo in AGGANCIO': ind['quota_aggancio'],
        'visto in AGGANCIO': ind['visto_in_aggancio'],
        'distanza in AGGANCIO': ind['dist_aggancio'],
        'durata dell aggancio': median(durate) if durate else None,
        'errore di velocita': errore_velocita(righe),
    }


def raduna(pattern):
    """Raggruppa per configurazione: l'etichetta senza il suffisso _gN."""
    gruppi = {}
    for f in sorted(glob.glob(pattern)):
        nome = os.path.basename(f)
        m = re.search(r'_([A-Za-z0-9]+)_g(\d+)\.csv$', nome)
        if not m:
            continue
        gruppi.setdefault(m.group(1), []).append(f)
    return {k: v for k, v in gruppi.items() if len(v) >= 2}


def traiettoria_bersaglio(percorso):
    """Posizione vera del bersaglio, indicizzata sul tempo dall'avvio."""
    out = {}
    t0 = None
    for r in metriche.leggi(percorso):
        try:
            t = float(r['t_sim'])
            xy = (float(r['gt_target_x']), float(r['gt_target_y']))
        except (ValueError, KeyError, TypeError):
            continue
        if t0 is None:
            t0 = t
        out[round(t - t0, 1)] = xy
    return out


def divergenza(file, istanti=(10.0, 30.0, 60.0)):
    """Quanto si allontanano fra loro i bersagli di prove ripetute.

    E la domanda che viene prima di ogni confronto: due prove della stessa
    configurazione sono lo stesso esperimento? Il bersaglio reagisce al drone,
    quindi la coppia inseguitore-inseguito e un anello chiuso, e in un anello
    chiuso una differenza minima all'inizio non resta minima.
    """
    tr = [t for t in (traiettoria_bersaglio(f) for f in file) if t]
    if len(tr) < 2:
        return None
    fuori = []
    for istante in istanti:
        d = []
        for i in range(len(tr)):
            for j in range(i + 1, len(tr)):
                a, b = tr[i].get(istante), tr[j].get(istante)
                if a and b:
                    d.append(math.hypot(a[0] - b[0], a[1] - b[1]))
        fuori.append(median(d) if d else float('nan'))
    return fuori


def valido(x):
    return x is not None and not (isinstance(x, float) and math.isnan(x))


def prove_necessarie(cv, effetto, limite=500):
    """Prove per gruppo per distinguere `effetto` (in frazione del valore).

    Oltre la tabella della t si usa l'approssimazione normale: con campioni
    cosi grandi la differenza fra le due e trascurabile rispetto
    all'incertezza sul CV stesso.
    """
    for n in range(2, limite + 1):
        df = 2 * (n - 1)
        t1 = T_975.get(df, 1.960)
        t2 = T_800.get(df, 0.842)
        if (t1 + t2) * math.sqrt(2.0 / n) * cv <= effetto:
            return n
    return None


def main(argv):
    pattern = argv[1] if len(argv) > 1 else 'metrics/*_g*.csv'
    gruppi = raduna(pattern)
    if not gruppi:
        print('nessun gruppo di prove ripetute trovato in %s' % pattern)
        return 2

    misure = {g: [misura_prova(f) for f in file] for g, file in gruppi.items()}
    misure = {g: [m for m in v if m] for g, v in misure.items()}

    print('Prove ripetute trovate: %d gruppi, %d prove'
          % (len(misure), sum(len(v) for v in misure.values())))
    print()

    # --- lo stimolo e lo stesso? ---
    print('Distanza fra i bersagli di prove ripetute, cioe quanto lo stimolo')
    print('smette di essere lo stesso col passare del tempo:')
    print('  %-14s %3s %9s %9s %9s' % ('gruppo', 'n', 'a 10 s', 'a 30 s', 'a 60 s'))
    for g in sorted(gruppi):
        d = divergenza(gruppi[g])
        if d is None:
            continue
        print('  %-14s %3d %8.0fm %8.0fm %8.0fm'
              % (g, len(gruppi[g]), d[0], d[1], d[2]))
    print()

    # --- scarto dentro ogni gruppo, indicatore per indicatore ---
    cv_per_indicatore = {}
    for nome, unita in INDICATORI:
        print('%s  [%s]' % (nome, unita))
        print('  %-14s %3s %10s %10s %8s %8s'
              % ('gruppo', 'n', 'media', 'scarto', 'CV%', 'max/min'))
        pesi, quadrati = 0.0, 0.0
        for g in sorted(misure):
            v = [m[nome] for m in misure[g] if valido(m[nome])]
            if len(v) < 2:
                continue
            mu, sd = mean(v), stdev(v)
            if mu == 0:
                continue
            cv = sd / abs(mu)
            rapporto = max(v) / min(v) if min(v) > 0 else float('nan')
            print('  %-14s %3d %10.2f %10.2f %7.0f%% %8.1f'
                  % (g, len(v), mu, sd, 100 * cv, rapporto))
            pesi += len(v) - 1
            quadrati += (len(v) - 1) * cv * cv
        if pesi:
            cv_pool = math.sqrt(quadrati / pesi)
            cv_per_indicatore[nome] = cv_pool
            print('  %-14s %3s %10s %10s %7.0f%%   <-- aggregato'
                  % ('', '', '', '', 100 * cv_pool))
        print()

    # --- differenza minima distinguibile ---
    print('=' * 72)
    print('Differenza minima distinguibile, in percentuale del valore di base')
    print('(confronto fra due configurazioni, n prove ciascuna, 95% di')
    print(' confidenza e 80% di potenza)')
    print()
    print('  %-24s %s' % ('indicatore', ''.join('%8s' % ('n=%d' % n)
                                                for n in (2, 3, 4, 5, 6))))
    for nome, _ in INDICATORI:
        cv = cv_per_indicatore.get(nome)
        if cv is None:
            continue
        celle = ''
        for n in (2, 3, 4, 5, 6):
            df = 2 * (n - 1)
            fattore = (T_975[df] + T_800[df]) * math.sqrt(2.0 / n)
            celle += '%7.0f%%' % (100 * fattore * cv)
        print('  %-24s %s' % (nome, celle))

    print()
    print('Prove per configurazione necessarie a distinguere una differenza')
    print('del 25%%, e ore di macchina che costano (%d min a prova):' % MINUTI)
    for nome, _ in INDICATORI:
        cv = cv_per_indicatore.get(nome)
        if cv is None:
            continue
        n = prove_necessarie(cv, 0.25)
        if n is None:
            print('  %-24s oltre 500 — non e una strada' % nome)
            continue
        ore = 2 * n * MINUTI / 60.0
        print('  %-24s %4d prove per gruppo   %5.1f ore' % (nome, n, ore))

    print()
    print('Ipotesi: prove indipendenti, dispersione simile fra configurazioni,')
    print('distribuzione non troppo lontana dalla normale. Con tre prove per')
    print('gruppo sono ipotesi fragili: i numeri sopra indicano un ordine di')
    print('grandezza, non una soglia esatta.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
