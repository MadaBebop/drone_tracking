#!/usr/bin/env python3
"""Analisi dei CSV prodotti da metrics_node.

Sostituisce gli script usa-e-getta con cui finora sono stati ricavati i numeri
citati nella relazione. Solo libreria standard: gira identico nel container e
sull'host, senza pandas.

    metriche.py riassumi metrics/*.csv        una riga di riepilogo per prova
    metriche.py confronta A.csv B.csv         ripetibilita di due prove gemelle
    metriche.py ricerche metrics/*.csv        esito di ogni ricerca del bersaglio
    metriche.py gruppi 'A*.csv' 'B*.csv'      due configurazioni, piu prove ciascuna
    metriche.py stima metrics/*.csv           qualita della velocita stimata
    metriche.py rumore metrics/*.csv          misura di R dal rilevatore

Il confronto e il criterio di verifica della Fase 0: due prove con la stessa
configurazione e lo stesso seme devono dare le stesse fasi nello stesso ordine
e distanze che differiscono solo per il rumore di scheduling. Se differiscono
di piu, l'esperimento non e ripetibile e ogni misura successiva vale poco.
"""
import csv
import glob
import math
import sys
from statistics import mean, median

# Semicampo della telecamera, come in controller_node.
TAN_O = 1.0
TAN_V = 0.750

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


def episodi_ricerca(righe):
    """Ogni tratto di RICERCA con il suo esito.

    La durata media di una ricerca, da sola, non dice se la ricerca funziona:
    una ricerca corta puo essere un riaggancio riuscito oppure una prova
    finita. Cio che distingue le due strategie e l'esito — quante perdite
    tornano un aggancio — e quanto il velivolo si avvicina davvero al
    bersaglio mentre lo cerca, che e la grandezza su cui una ricerca
    direzionale dovrebbe battere una spirale cieca.
    """
    episodi = []
    inizio = None
    for i, r in enumerate(righe):
        if r.get('fase') == 'RICERCA':
            if inizio is None:
                inizio = i
        elif inizio is not None:
            episodi.append((inizio, i - 1, r.get('fase')))
            inizio = None
    if inizio is not None:
        # Nessuna fase successiva: la prova e finita mentre cercava ancora.
        # Va contato come ricerca non conclusa, non come ricerca lunga.
        episodi.append((inizio, len(righe) - 1, None))
    return episodi


def descrivi_ricerche(percorso):
    """Tabella degli episodi di ricerca di una prova. Torna la lista degli
    episodi come tuple (durata, riagganciato, distanza minima)."""
    righe = leggi(percorso)
    if not righe:
        print('%s: vuoto' % percorso)
        return []

    fuori = []
    print('=' * 72)
    print(percorso)
    episodi = episodi_ricerca(righe)
    if not episodi:
        print('  nessuna ricerca: il bersaglio non e mai stato perso')
        return []

    print('  %-8s %-8s %-10s %-10s %-5s %s' % (
        'inizio', 'durata', 'dist_ini', 'dist_min', 'visto', 'esito'))
    for i, j, dopo in episodi:
        t0 = float(righe[i]['t_sim'])
        durata = float(righe[j]['t_sim']) - t0
        tratto = righe[i:j + 1]
        d = numeri(tratto, 'dist_xy_gt')
        d_ini = d[0] if d else float('nan')
        d_min = min(d) if d else float('nan')
        # Che il bersaglio sia tornato davanti alla telecamera non si deduce
        # dal ritorno in AGGANCIO, che scatta su fotogrammi validi del tracker
        # e quelli sopravvivono sulla predizione. Nemmeno dal campionamento del
        # rilevatore: pubblica a ~25 Hz mentre qui si campiona a 5, quindi un
        # avvistamento di un fotogramma ha una probabilita su cinque di
        # comparire. La prova sta nella semantica del filtro, che dopo
        # `soglia_perdita` si azzera e puo tornare valido SOLO con una misura
        # vera: una risalita di trk_valido da 0 a 1 e un rilevamento avvenuto.
        visti = sum(1 for r in tratto if r.get('det_valido') == '1')
        risalita = any(a.get('trk_valido') == '0' and b.get('trk_valido') == '1'
                       for a, b in zip(tratto, tratto[1:]))
        riagganciato = (dopo == 'AGGANCIO')
        confermato = riagganciato and (visti > 0 or risalita)
        if not riagganciato:
            esito = 'prova finita' if dopo is None else 'passata a %s' % dopo
        elif visti > 0:
            esito = 'riagganciato (visto)'
        elif risalita:
            esito = 'riagganciato (risalita del tracker)'
        else:
            # Ne un rilevamento campionato ne una risalita: il tracker non e
            # mai stato invalidato e il riaggancio poggia solo sulla sua
            # predizione. Non e dimostrato.
            esito = 'riagganciato NON dimostrato'
        print('  %-8.1f %-8.1f %-10.1f %-10.1f %-5d %s' % (
            t0, durata, d_ini, d_min, visti, esito))
        fuori.append((durata, confermato, d_min, riagganciato, visti))

    confermati = [e for e in fuori if e[1]]
    dichiarati = [e for e in fuori if e[3]]
    print('  --> %d ricerche, %d riagganci dichiarati, '
          '%d con un rilevamento dimostrabile' % (
              len(fuori), len(dichiarati), len(confermati)))
    if confermati:
        print('      tempo di riaggancio mediano %.1f s' % median(
            [e[0] for e in confermati]))
    minime = [e[2] for e in fuori if e[2] == e[2]]
    if minime:
        print('      avvicinamento massimo durante la ricerca: '
              'mediana %.1f m, migliore %.1f m' % (median(minime), min(minime)))
    return fuori


def ricerche(percorsi):
    """Esiti delle ricerche su una o piu prove.

    Piu prove insieme perche a scala reale la dispersione fra prove ripetute e
    un fattore due-quattro: una prova sola non distingue una strategia
    migliore dal caso.
    """
    tutti = []
    for percorso in percorsi:
        tutti.extend(descrivi_ricerche(percorso))
    if len(percorsi) > 1 and tutti:
        confermati = [e for e in tutti if e[1]]
        dichiarati = [e for e in tutti if e[3]]
        print('=' * 72)
        print('AGGREGATO su %d prove' % len(percorsi))
        print('  ricerche totali        %d' % len(tutti))
        print('  riagganci dichiarati   %d (%.0f%%)' % (
            len(dichiarati), 100.0 * len(dichiarati) / len(tutti)))
        print('  con rilevamento dimostrabile %d (%.0f%% delle ricerche)' % (
            len(confermati), 100.0 * len(confermati) / len(tutti)))
        if confermati:
            print('  tempo di riaggancio    mediano %.1f s, medio %.1f s' % (
                median([e[0] for e in confermati]),
                mean([e[0] for e in confermati])))
        minime = [e[2] for e in tutti if e[2] == e[2]]
        if minime:
            print('  avvicinamento massimo  mediana %.1f m, migliore %.1f m' % (
                median(minime), min(minime)))
    return 0


def velocita_vera(righe):
    """Velocita del bersaglio per differenze finite sulla verita a terra."""
    fuori = {}
    for a, b in zip(righe, righe[1:]):
        try:
            dt = float(b['t_sim']) - float(a['t_sim'])
            if dt <= 0:
                continue
            fuori[b['t_sim']] = (
                (float(b['gt_target_x']) - float(a['gt_target_x'])) / dt,
                (float(b['gt_target_y']) - float(a['gt_target_y'])) / dt)
        except (KeyError, ValueError):
            continue
    return fuori


def velocita_stimata(riga):
    """Velocita del bersaglio nel mondo, ricostruita dalla stima del filtro.

    Il filtro lavora in coordinate immagine al secondo e, ricevendo il moto del
    velivolo come ingresso noto, stima la velocita PROPRIA del bersaglio: qui
    resta da convertirla in metri usando la quota e da ruotarla con
    l'imbardata. Non si somma la velocita del velivolo — era quel passaggio,
    una differenza fra grandezze grandi per ottenerne una piccola, a produrre
    l'errore pari al segnale.
    """
    vx, vy = numeri([riga], 'trk_vx'), numeri([riga], 'trk_vy')
    if not vx or not vy:
        return None
    quota = numeri([riga], 'gt_drone_z')
    yaw = numeri([riga], 'yaw')
    if not quota or not yaw or quota[0] < 5.0:
        return None
    avanti = -vy[0] * quota[0] * TAN_V
    laterale = -vx[0] * quota[0] * TAN_O
    c, s = math.cos(yaw[0]), math.sin(yaw[0])
    return avanti * c - laterale * s, avanti * s + laterale * c


def stima(percorsi):
    """Errore della velocita stimata, accanto al segnale che dovrebbe misurare.

    Il confronto che conta non e l'errore in se ma il suo rapporto con la
    velocita vera: un errore di 10 m/s su un bersaglio che ne percorre 10
    significa che la stima non porta informazione, per quanto il numero possa
    sembrare piccolo in assoluto.
    """
    tutti_err, tutti_vero = [], []
    print('%-44s %6s %9s %9s' % ('prova', 'n', 'errore', 'v_vera'))
    for percorso in percorsi:
        righe = leggi(percorso)
        # La conversione vale solo dove il filtro riceve il moto del velivolo
        # come ingresso noto: prima, trk_vx/vy erano velocita RELATIVE, e
        # usarle qui produce un numero sbagliato senza che nulla lo segnali
        # (verificato: 12.8 e 17.2 invece di 10.8). La colonna `nis` e comparsa
        # con quella modifica e fa da discriminante.
        if 'nis' not in righe[0]:
            print('%-44s   precedente all ingresso noto: saltata'
                  % percorso.split('/')[-1][:44])
            continue
        vero = velocita_vera(righe)
        err = []
        for r in righe:
            stimata = velocita_stimata(r)
            v = vero.get(r['t_sim'])
            if stimata is None or v is None:
                continue
            err.append(math.hypot(stimata[0] - v[0], stimata[1] - v[1]))
        if len(err) < 20:
            continue
        moduli = [math.hypot(*v) for v in vero.values()]
        tutti_err += err
        tutti_vero += moduli
        print('%-44s %6d %9.1f %9.1f' % (
            percorso.split('/')[-1][:44], len(err), median(err), median(moduli)))

    if not tutti_err:
        print('nessun campione confrontabile')
        return 1
    e, v = median(tutti_err), median(tutti_vero)
    print('-' * 71)
    print('%-44s %6d %9.1f %9.1f' % ('TUTTE', len(tutti_err), e, v))
    print()
    print('  errore / segnale = %.2f  (%s)' % (
        e / v if v else float('nan'),
        'la stima non porta informazione' if e >= 0.8 * v
        else 'utilizzabile' if e <= 0.4 * v else 'al limite'))
    return 0


def proiezione_attesa(riga):
    """Dove il bersaglio vero dovrebbe comparire nell'immagine.

    E la geometria di controller_node percorsa al contrario: dalla posizione
    vera al suolo si ricava lo scostamento in assi velivolo, lo si divide per
    l'impronta a terra e si RIMETTE l'assetto della telecamera che il controllo
    invece sottrae.
    """
    def n(col):
        v = numeri([riga], col)
        return v[0] if v else None

    bx, by = n('gt_target_x'), n('gt_target_y')
    dx, dy = n('gt_drone_x'), n('gt_drone_y')
    quota, yaw = n('gt_drone_z'), n('yaw')
    roll, pitch = n('roll'), n('pitch')
    g_roll = n('gimbal_roll') or 0.0
    g_pitch = n('gimbal_pitch') or 0.0
    if None in (bx, by, dx, dy, quota, yaw, roll, pitch) or quota < 5.0:
        return None

    c, sn = math.cos(yaw), math.sin(yaw)
    avanti = (bx - dx) * c + (by - dy) * sn
    laterale = -(bx - dx) * sn + (by - dy) * c
    norm_x = -laterale / (quota * TAN_O)
    norm_y = -avanti / (quota * TAN_V)

    alpha_x = math.atan(norm_x * TAN_O) + (roll + g_roll)
    alpha_y = math.atan(norm_y * TAN_V) - (pitch + g_pitch)
    limite = 1.4
    alpha_x = max(-limite, min(limite, alpha_x))
    alpha_y = max(-limite, min(limite, alpha_y))
    return math.tan(alpha_x) / TAN_O, math.tan(alpha_y) / TAN_V


def rumore(percorsi):
    """Varianza dello scarto fra rilevamento e proiezione attesa: e R.

    Si usano solo i campioni con rilevamento valido e fuori dalle finestre di
    disturbo: dentro, lo scarto misurerebbe il jammer e non il rilevatore, e R
    di base deve descrivere il sensore sano.
    """
    scarti_x, scarti_y = [], []
    for percorso in percorsi:
        righe = leggi(percorso)
        for r in righe:
            if r.get('det_valido') != '1':
                continue
            disturbo = numeri([r], 'rumore_rf')
            if disturbo and disturbo[0] > 0.01:
                continue
            attesa = proiezione_attesa(r)
            u, v = numeri([r], 'det_x'), numeri([r], 'det_y')
            if attesa is None or not u or not v:
                continue
            if abs(u[0]) > 1.0 or abs(v[0]) > 1.0:
                continue
            scarti_x.append(u[0] - attesa[0])
            scarti_y.append(v[0] - attesa[1])

    if len(scarti_x) < 30:
        print('campioni insufficienti: %d' % len(scarti_x))
        return 1

    def varianza(valori):
        m = mean(valori)
        return sum((x - m) ** 2 for x in valori) / (len(valori) - 1)

    vx, vy = varianza(scarti_x), varianza(scarti_y)
    print('campioni (rilevamento valido, senza disturbo)  %d' % len(scarti_x))
    print('  scarto x   media %+.4f   deviazione %.4f   varianza %.5f'
          % (mean(scarti_x), math.sqrt(vx), vx))
    print('  scarto y   media %+.4f   deviazione %.4f   varianza %.5f'
          % (mean(scarti_y), math.sqrt(vy), vy))
    r_misurata = max(vx, vy)
    print()
    print('  R misurata (limite superiore)   %.5f' % r_misurata)
    print('  rumore_sensore_base in uso      0.05000')
    print('  rapporto                        %.0fx troppo grande'
          % (0.05 / r_misurata) if r_misurata > 0 else '')
    print()
    print('  Lo scarto contiene anche l errore della ricostruzione, che usa')
    print('  quota e assetto campionati a 5 Hz contro i 25 del rilevatore:')
    print('  il valore e quindi un limite superiore del rumore vero.')
    return 0


def _indicatori(percorso):
    """Gli indicatori di una prova, come numeri e senza stampare nulla."""
    righe = leggi(percorso)
    if not righe:
        return None
    in_aggancio = [r for r in righe if r.get('fase') == 'AGGANCIO']
    totale = len(righe)
    d_agg = numeri(in_aggancio, 'dist_xy_gt')
    episodi = []
    for i, j, dopo in episodi_ricerca(righe):
        tratto = righe[i:j + 1]
        d = numeri(tratto, 'dist_xy_gt')
        visti = sum(1 for r in tratto if r.get('det_valido') == '1')
        risalita = any(a.get('trk_valido') == '0' and b.get('trk_valido') == '1'
                       for a, b in zip(tratto, tratto[1:]))
        episodi.append({
            'durata': float(righe[j]['t_sim']) - float(righe[i]['t_sim']),
            'confermato': dopo == 'AGGANCIO' and (visti > 0 or risalita),
            'dist_min': min(d) if d else None,
        })
    return {
        'quota_aggancio': 100.0 * len(in_aggancio) / totale if totale else 0.0,
        'visto_in_aggancio': frazione(in_aggancio, 'det_valido') if in_aggancio else float('nan'),
        'dist_aggancio': median(d_agg) if d_agg else float('nan'),
        'agganci': tratti(righe, 'AGGANCIO'),
        'ricerche': episodi,
    }


def gruppi(pattern_a, pattern_b):
    """Confronto fra due configurazioni, piu prove ciascuna.

    A scala reale la dispersione fra prove ripetute e un fattore due-quattro
    sulla durata dell'aggancio: `confronta` fra due prove sole non distingue
    una configurazione migliore dal caso, e questo comando esiste per non
    invitare piu a farlo. Le mediane sono sulle prove, non sui campioni: una
    prova andata male non pesa in proporzione a quanto e andata male.
    """
    for etichetta, pattern in (('A', pattern_a), ('B', pattern_b)):
        if not sorted(glob.glob(pattern)):
            print('gruppo %s: nessun file per %s' % (etichetta, pattern))
            return 2

    misure = {}
    for etichetta, pattern in (('A', pattern_a), ('B', pattern_b)):
        file = sorted(glob.glob(pattern))
        prove = [x for x in (_indicatori(f) for f in file) if x]
        ricerche = [e for p in prove for e in p['ricerche']]
        confermate = [e for e in ricerche if e['confermato']]
        minime = [e['dist_min'] for e in ricerche if e['dist_min'] is not None]
        agganci = [d for p in prove for d in p['agganci']]
        misure[etichetta] = {
            'file': file,
            'prove': len(prove),
            'tempo in AGGANCIO (%)': median([p['quota_aggancio'] for p in prove]),
            'visto in AGGANCIO (%)': median([p['visto_in_aggancio'] for p in prove]),
            'distanza in AGGANCIO (m)': median([p['dist_aggancio'] for p in prove]),
            'durata aggancio (s)': median(agganci) if agganci else float('nan'),
            'ricerche (n)': len(ricerche),
            'riagganciate (%)': (100.0 * len(confermate) / len(ricerche)
                                 if ricerche else float('nan')),
            'tempo di riaggancio (s)': (median([e['durata'] for e in confermate])
                                        if confermate else float('nan')),
            'distanza minima (m)': median(minime) if minime else float('nan'),
        }

    print('=' * 72)
    for etichetta in ('A', 'B'):
        print('gruppo %s: %d prove' % (etichetta, misure[etichetta]['prove']))
        for f in misure[etichetta]['file']:
            print('    %s' % f)
    print('-' * 72)
    # Due blocchi, perche le due meta hanno unita diverse e confonderle
    # sarebbe il modo piu facile di leggere male questa tabella. Sopra, una
    # mediana sulle prove: la domanda e come va una missione tipica. Sotto,
    # tutti gli episodi di ricerca messi insieme: la domanda e, dato che il
    # bersaglio e stato perso, se la ricerca lo ritrova — e una prova che non
    # perde mai il bersaglio non ha voce in capitolo su quella domanda, ma
    # peserebbe eccome su una mediana per prova.
    def blocco(titolo, chiavi):
        print('  %-28s %10s %10s' % (titolo, 'A', 'B'))
        for chiave in chiavi:
            print('  %-28s %10.1f %10.1f' % (chiave, misure['A'][chiave],
                                             misure['B'][chiave]))

    blocco('mediana sulle prove', (
        'tempo in AGGANCIO (%)', 'visto in AGGANCIO (%)',
        'distanza in AGGANCIO (m)', 'durata aggancio (s)'))
    print('-' * 72)
    blocco('su tutte le ricerche', (
        'ricerche (n)', 'riagganciate (%)',
        'tempo di riaggancio (s)', 'distanza minima (m)'))
    return 0


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


def residuo_telecamera(righe):
    """Inclinazione residua dell'asse ottico, in gradi: mediana e massimo.

    Vale assetto del corpo piu angolo del giunto. Se la sospensione lavora e
    zero; se va a fondo corsa cresce, e il massimo lo rivela anche quando la
    mediana resta nulla. Serve a distinguere una sospensione che funziona da
    una che satura, distinzione che il solo comando al giunto non permette:
    un comando al valore limite puo essere il picco di una richiesta
    soddisfatta oppure una richiesta tosata.
    """
    import math as _math
    residui = []
    for r in righe:
        try:
            roll = float(r['roll']) + float(r['gimbal_roll'])
            pitch = float(r['pitch']) + float(r['gimbal_pitch'])
        except (ValueError, KeyError, TypeError):
            continue
        residui.append(_math.degrees(max(abs(roll), abs(pitch))))
    if not residui:
        return None
    return median(residui), max(residui)


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
    # Il denominatore giusto per la visibilita e la fase di aggancio, non la
    # prova intera: durante l'avvicinamento il bersaglio non e ancora
    # raggiungibile, e contarlo come "non rilevato" misura la distanza di
    # partenza invece della qualita dell'inseguimento. Su un circuito da 150 m
    # quella distinzione vale venti punti percentuali.
    in_aggancio = [r for r in righe if r.get('fase') == 'AGGANCIO']
    if in_aggancio:
        print('    di cui in AGGANCIO    %.1f%% visto, %.1f%% stimato '
              '(%d campioni)' % (frazione(in_aggancio, 'det_valido'),
                                 frazione(in_aggancio, 'trk_valido'),
                                 len(in_aggancio)))
        d_agg = numeri(in_aggancio, 'dist_xy_gt')
        if d_agg:
            print('    distanza in AGGANCIO  mediana %.2f m  media %.2f m  '
                  'max %.2f' % (median(d_agg), mean(d_agg), max(d_agg)))
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

    # Assetto residuo della telecamera: assetto del corpo piu angolo del
    # giunto, cioe quanto l'asse ottico resta inclinato rispetto al terreno.
    # E la misura diretta dell'effetto della sospensione, e a differenza della
    # correlazione qui sotto non e ambigua.
    residui = residuo_telecamera(righe)
    if residui:
        # Solo la mediana e affidabile: assetto e comando al giunto arrivano da
        # callback diverse e vengono campionati a istanti fino a qualche
        # decimo di secondo di distanza, quindi durante una manovra aggressiva
        # il massimo misura lo sfasamento fra i due campionamenti, non un
        # errore di stabilizzazione. La saturazione si verifica sul comando.
        print('  residuo telecamera     mediano %.2f gradi' % residui[0])
    gr = numeri(righe, 'gimbal_roll')
    if gr:
        gp = numeri(righe, 'gimbal_pitch') or [0.0]
        picco = max(max(abs(v) for v in gr), max(abs(v) for v in gp))
        print('  comando gimbal         picco %.3f rad su limite 1.047 -> %s'
              % (picco, 'SATURA' if picco > 1.04 else 'entro corsa'))

    # Correlazione fra assetto e posizione del bersaglio nell'immagine.
    # ATTENZIONE alla lettura: misura un'associazione, non una causa. Con la
    # telecamera solidale al corpo l'associazione viene dal supporto, ed e la
    # grandezza che la sospensione deve annullare. Con la sospensione attiva e
    # il residuo prossimo a zero, invece, l'associazione che resta va nella
    # direzione opposta — il velivolo si inclina PERCHE il bersaglio e
    # decentrato — e un valore alto indica un controllo reattivo, non un
    # difetto. Il residuo qui sopra dice quale delle due letture vale.
    for etichetta, assetto, immagine in (('pitch/det_y', 'pitch', 'det_y'),
                                         ('roll/det_x', 'roll', 'det_x')):
        r = correlazione(righe, assetto, immagine)
        if r is not None:
            print('  associazione %-11s r = %+.3f' % (etichetta, r))

    # Con due gradi di liberta il NIS vale 2 in media se Q e R descrivono la
    # realta. E la sola verifica del filtro che non richiede la verita a terra,
    # quindi la sola ripetibile su un velivolo vero. Va letto separando le
    # finestre di disturbo: li R viene gonfiata apposta, e un NIS basso e il
    # comportamento voluto invece che un difetto.
    pulito = [numeri([r], 'nis')[0] for r in righe
              if numeri([r], 'nis')
              and not (numeri([r], 'rumore_rf')
                       and numeri([r], 'rumore_rf')[0] > 0.01)]
    disturbato = [numeri([r], 'nis')[0] for r in righe
                  if numeri([r], 'nis')
                  and numeri([r], 'rumore_rf')
                  and numeri([r], 'rumore_rf')[0] > 0.01]
    if pulito:
        medio = mean(pulito)
        if medio > 3.0:
            giudizio = 'troppo sicuro di se: Q o R sottostimate'
        elif medio < 1.0:
            giudizio = 'troppo prudente: Q o R sovrastimate, corregge poco'
        else:
            giudizio = 'coerente'
        print('  NIS senza disturbo     %.2f medio su %d campioni — atteso 2.0 (%s)'
              % (medio, len(pulito), giudizio))
    if disturbato:
        print('  NIS sotto disturbo     %.2f medio su %d campioni (R gonfiata '
              'apposta: qui basso e voluto)'
              % (mean(disturbato), len(disturbato)))

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
    if comando == 'ricerche':
        return ricerche(file)
    if comando == 'stima':
        return stima(file)
    if comando == 'rumore':
        return rumore(file)
    if comando == 'gruppi':
        if len(file) != 2:
            print('gruppi vuole due pattern, uno per configurazione')
            return 2
        return gruppi(file[0], file[1])
    if comando == 'confronta':
        if len(file) != 2:
            print('confronta vuole esattamente due file')
            return 2
        return confronta(file[0], file[1])
    print('comando sconosciuto: %s' % comando)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv))
