#!/usr/bin/env python3
"""Analisi dei CSV prodotti da metrics_node.

Sostituisce gli script usa-e-getta con cui finora sono stati ricavati i numeri
citati nella relazione. Solo libreria standard: gira identico nel container e
sull'host, senza pandas.

    metriche.py riassumi metrics/*.csv        una riga di riepilogo per prova
    metriche.py confronta A.csv B.csv         ripetibilita di due prove gemelle

Il confronto e il criterio di verifica della Fase 0: due prove con la stessa
configurazione e lo stesso seme devono dare le stesse fasi nello stesso ordine
e distanze che differiscono solo per il rumore di scheduling. Se differiscono
di piu, l'esperimento non e ripetibile e ogni misura successiva vale poco.
"""
import csv
import sys
from statistics import mean, median

NUMERICHE = ('dist_xy_gt', 'dist_3d_gt', 'dist_xy_ekf', 'det_hz', 'trk_hz',
             'gt_drone_x', 'gt_drone_y', 'gt_drone_z',
             'gt_target_x', 'gt_target_y', 'gt_target_z')


def leggi(percorso):
    with open(percorso, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def numeri(righe, colonna):
    """Valori numerici di una colonna, saltando le celle vuote."""
    fuori = []
    for r in righe:
        v = r.get(colonna, '')
        if v not in ('', None):
            try:
                fuori.append(float(v))
            except ValueError:
                pass
    return fuori


def tratti(righe, fase):
    """Durate dei tratti consecutivi in una data fase, in secondi di
    simulazione. Un tratto di AGGANCIO e la durata di un aggancio: e la
    grandezza su cui si confrontano le configurazioni."""
    durate = []
    inizio = None
    for r in righe:
        t = float(r['t_sim'])
        if r['fase'] == fase:
            if inizio is None:
                inizio = t
        elif inizio is not None:
            durate.append(t - inizio)
            inizio = None
    if inizio is not None:
        durate.append(float(righe[-1]['t_sim']) - inizio)
    return durate


def sequenza_fasi(righe):
    """Fasi attraversate, senza ripetizioni consecutive."""
    fuori = []
    for r in righe:
        if not fuori or fuori[-1] != r['fase']:
            fuori.append(r['fase'])
    return fuori


def frazione(righe, colonna):
    valori = numeri(righe, colonna)
    return (100.0 * sum(1 for v in valori if v) / len(valori)) if valori else float('nan')


def percentile(valori, q):
    """Percentile per interpolazione lineare, senza dipendenze esterne."""
    if not valori:
        return float('nan')
    ordinati = sorted(valori)
    if len(ordinati) == 1:
        return ordinati[0]
    posizione = q * (len(ordinati) - 1)
    basso = int(posizione)
    alto = min(basso + 1, len(ordinati) - 1)
    peso = posizione - basso
    return ordinati[basso] * (1 - peso) + ordinati[alto] * peso


def velocita(righe, col_x, col_y):
    """Modulo della velocita ricavato dalla verita a terra, m/s simulati.

    Serve a verificare che un parametro impostato a caldo abbia davvero avuto
    effetto: la velocita del bersaglio e un'uscita osservabile, il valore del
    parametro solo un'intenzione. La differenza fra le due e stata reale.
    """
    fuori = []
    for prima, dopo in zip(righe, righe[1:]):
        try:
            dt = float(dopo['t_sim']) - float(prima['t_sim'])
            if dt <= 0:
                continue
            dx = float(dopo[col_x]) - float(prima[col_x])
            dy = float(dopo[col_y]) - float(prima[col_y])
        except (ValueError, KeyError, TypeError):
            continue
        fuori.append((dx * dx + dy * dy) ** 0.5 / dt)
    return fuori


def correlazione(righe, col_a, col_b):
    """Coefficiente di Pearson fra due colonne, None se non calcolabile.

    Serve a una domanda sola: quanto la posizione del bersaglio nell'immagine
    dipende dall'assetto del velivolo invece che dal bersaglio. E la misura
    dell'accoppiamento che la sospensione cardanica deve annullare, e senza di
    essa l'effetto del gimbal si giudicherebbe a occhio.
    """
    coppie = []
    for r in righe:
        try:
            a, b = r[col_a], r[col_b]
            if a in ('', None) or b in ('', None):
                continue
            # I campioni senza bersaglio non dicono nulla sull'accoppiamento.
            if float(r.get('det_valido', 1)) == 0:
                continue
            coppie.append((float(a), float(b)))
        except (ValueError, KeyError, TypeError):
            continue
    if len(coppie) < 20:
        return None
    ma = mean(a for a, _ in coppie)
    mb = mean(b for _, b in coppie)
    num = sum((a - ma) * (b - mb) for a, b in coppie)
    da = sum((a - ma) ** 2 for a, _ in coppie) ** 0.5
    db = sum((b - mb) ** 2 for _, b in coppie) ** 0.5
    if da == 0 or db == 0:
        return None
    return num / (da * db)


def riassumi(percorso):
    righe = leggi(percorso)
    if not righe:
        print('%s: vuoto' % percorso)
        return None

    durata = float(righe[-1]['t_sim']) - float(righe[0]['t_sim'])
    durata_reale = float(righe[-1]['t_wall']) - float(righe[0]['t_wall'])
    # Rapporto fra tempo simulato e tempo di orologio. Sotto 1 la simulazione e
    # piu lenta del tempo reale, cosa normale senza accelerazione grafica. Va
    # riportato perche condiziona il confronto fra prove: due prove con fattori
    # molto diversi hanno visto la stessa fisica ma con carichi diversi sulla
    # catena di percezione, e il numero di fotogrammi per secondo simulato
    # cambia di conseguenza.
    fattore = (durata / durata_reale) if durata_reale > 0 else float('nan')
    dist = numeri(righe, 'dist_xy_gt')
    agganci = tratti(righe, 'AGGANCIO')
    ricerche = tratti(righe, 'RICERCA')

    print('=' * 72)
    print(percorso)
    print('  campioni / durata      %d in %.1f s di simulazione '
          '(%.0f s reali, fattore %.2fx)' % (
              len(righe), durata, durata_reale, fattore))
    print('  sequenza fasi          %s' % ' -> '.join(sequenza_fasi(righe)))
    print('  bersaglio rilevato     %.1f%% dei campioni (detector)' % frazione(righe, 'det_valido'))
    print('  bersaglio tracciato    %.1f%% dei campioni (kalman)' % frazione(righe, 'trk_valido'))
    print('  sotto jamming          %.1f%% dei campioni' % frazione(righe, 'jam_attivo'))
    print('  GPS negato             %.1f%% dei campioni' % frazione(righe, 'gps_negato'))
    if dist:
        print('  distanza orizzontale   mediana %.2f m  media %.2f m  '
              'min %.2f  max %.2f' % (median(dist), mean(dist), min(dist), max(dist)))
    else:
        print('  distanza orizzontale   nessuna verita a terra nel file')
    if agganci:
        print('  agganci                %d, durate %s (media %.1f s)' % (
            len(agganci), ', '.join('%.1f' % d for d in agganci), mean(agganci)))
    else:
        print('  agganci                nessuno')
    if ricerche:
        print('  ricerche               %d, durate %s' % (
            len(ricerche), ', '.join('%.1f' % d for d in ricerche)))
    for etichetta, px, py in (('bersaglio', 'gt_target_x', 'gt_target_y'),
                              ('drone', 'gt_drone_x', 'gt_drone_y')):
        v = velocita(righe, px, py)
        if v:
            in_moto = [x for x in v if x > 0.05]
            # Il 95esimo percentile invece del massimo: un solo campione
            # anomalo — un salto di posa, un fotogramma perso — sposta il
            # massimo di un ordine di grandezza e nasconde la velocita vera.
            print('  velocita %-13s p95 %.2f m/s  media in moto %.2f m/s  '
                  '(max %.2f)' % (
                      etichetta, percentile(v, 0.95),
                      mean(in_moto) if in_moto else 0.0, max(v)))

    # Accoppiamento assetto-immagine: e il difetto che la sospensione
    # cardanica attacca alla radice, e si legge come correlazione.
    for etichetta, assetto, immagine in (('pitch/det_y', 'pitch', 'det_y'),
                                         ('roll/det_x', 'roll', 'det_x')):
        r = correlazione(righe, assetto, immagine)
        if r is not None:
            print('  accoppiamento %-10s r = %+.3f' % (etichetta, r))

    gr = numeri(righe, 'gimbal_roll')
    if gr:
        gp = numeri(righe, 'gimbal_pitch')
        print('  comando gimbal         rollio |max| %.3f rad  '
              'beccheggio |max| %.3f rad' % (
                  max(abs(v) for v in gr),
                  max(abs(v) for v in gp) if gp else float('nan')))

    ritmo = numeri(righe, 'det_hz')
    if ritmo:
        print('  ritmo percezione       %.1f Hz medi (min %.1f, max %.1f)' % (
            mean(ritmo), min(ritmo), max(ritmo)))
    return righe


def confronta(a, b):
    ra = riassumi(a)
    rb = riassumi(b)
    if not ra or not rb:
        return 1

    print('=' * 72)
    print('CONFRONTO DI RIPETIBILITA')
    fa, fb = sequenza_fasi(ra), sequenza_fasi(rb)
    print('  sequenza fasi          %s' % ('IDENTICA' if fa == fb else 'DIVERSA'))
    if fa != fb:
        print('    A: %s' % ' -> '.join(fa))
        print('    B: %s' % ' -> '.join(fb))
    print('  campioni               A %d  B %d (scarto %d)' % (
        len(ra), len(rb), abs(len(ra) - len(rb))))

    # Il primo campione utile e quello in cui la missione e gia partita: due
    # prove che iniziano a fasi diverse dell'orbita del bersaglio non sono
    # confrontabili riga per riga, e va detto invece di nasconderlo in una
    # media.
    d0a = numeri(ra[:1], 'dist_xy_gt')
    d0b = numeri(rb[:1], 'dist_xy_gt')
    if d0a and d0b:
        print('  distanza al via        A %.2f m  B %.2f m (scarto %.2f m)' % (
            d0a[0], d0b[0], abs(d0a[0] - d0b[0])))

    n = min(len(ra), len(rb))
    print('  scarti sulle prime %d righe, allineate per indice:' % n)
    for col in NUMERICHE:
        va, vb = [], []
        for i in range(n):
            x, y = ra[i].get(col, ''), rb[i].get(col, '')
            if x not in ('', None) and y not in ('', None):
                try:
                    va.append(float(x))
                    vb.append(float(y))
                except ValueError:
                    pass
        if not va:
            continue
        scarti = [abs(x - y) for x, y in zip(va, vb)]
        print('    %-14s medio %8.3f   massimo %8.3f' % (
            col, mean(scarti), max(scarti)))
    print()
    print('  Nota: l allineamento e per indice di riga, non per tempo. Un solo')
    print('  campione in piu o in meno all avvio sfalsa tutto il resto, quindi')
    print('  scarti crescenti nel tempo indicano uno sfasamento, non una')
    print('  divergenza della dinamica.')
    return 0


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    comando, file = argv[1], argv[2:]
    if comando == 'riassumi':
        for f in file:
            riassumi(f)
        return 0
    if comando == 'confronta':
        if len(file) != 2:
            print('confronta vuole esattamente due file')
            return 2
        return confronta(file[0], file[1])
    print('comando sconosciuto: %s' % comando)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv))
