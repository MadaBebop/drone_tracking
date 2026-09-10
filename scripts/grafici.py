#!/usr/bin/env python3
"""Figure del filtro di Kalman, rigenerabili da un CSV di metrics_node.

    grafici.py /ws/metrics/<prova>.csv [cartella_uscita]

Produce due immagini:

    kalman_stima.png    stato stimato e matrice R nel tempo
    kalman_errore.png   errore di posizione con e senza filtro

Esistono come script e non come sessione interattiva per la stessa ragione per
cui esiste metriche.py: una figura che finisce in una relazione deve poter
essere rifatta da chiunque, dallo stesso dato, ottenendo la stessa immagine.

Il termine di paragone del secondo grafico e la misura DISTURBATA, non il
rilevamento pulito. E una scelta di sostanza: l'alternativa al filtro non e il
segnale pulito, che nessuno possiede, ma quello che arriva davvero dal
rilevatore dopo il disturbo. Confrontare l'uscita del filtro con il segnale
pulito misurerebbe l'errore residuo, non il guadagno.
"""
import csv
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt          # noqa: E402
from matplotlib.lines import Line2D      # noqa: E402
from matplotlib.patches import Patch     # noqa: E402

# --- Geometria della telecamera (la stessa di controller_node) --------------
TAN_O = 1.0      # tan del semicampo orizzontale, 90 gradi
TAN_V = 0.750    # tan del semicampo verticale

# --- Matrice R (la stessa legge di tracker_node.on_noise_level) -------------
R_BASE = 0.05
R_MAX = 2.00

# --- Tavolozza --------------------------------------------------------------
# Slot categorici 1-3 della tavolozza di riferimento, validati su tutte le
# coppie in modo chiaro: peggiore separazione CVD 9.2, visione normale 24.0.
# L'acqua sta sotto 3:1 di contrasto con la superficie, e per questo porta
# un'etichetta diretta: l'identita non e mai affidata al solo colore.
BLU = '#2a78d6'
ARANCIO = '#eb6834'
ACQUA = '#1baf7a'
# La verita si distingue dalla stima per LARGHEZZA prima che per tinta: banda
# spessa sotto, linea sottile sopra. Le due curve si sovrappongono quasi
# ovunque — e il punto della figura — e affidare la distinzione al solo colore
# sarebbe fragile proprio dove serve. Con la larghezza in gioco, l'acqua puo
# restare: separazione dal blu 24.0 in visione normale contro i 16.3 del
# violetto, che pure passerebbe ogni soglia.

SUPERFICIE = '#fcfcfb'
INCHIOSTRO = '#0b0b0b'
SECONDARIO = '#52514e'
SPENTO = '#898781'
GRIGLIA = '#e1e0d9'
ASSE = '#c3c2b7'
BANDA = '#ececE6'      # finestre di disturbo: neutra, non ruba una tinta


def stile():
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.sans-serif': ['DejaVu Sans'],
        'font.size': 9,
        'figure.facecolor': SUPERFICIE,
        'axes.facecolor': SUPERFICIE,
        'axes.edgecolor': ASSE,
        'axes.linewidth': 0.8,
        'axes.labelcolor': SECONDARIO,
        'axes.titlecolor': INCHIOSTRO,
        'xtick.color': SPENTO,
        'ytick.color': SPENTO,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'grid.color': GRIGLIA,
        'grid.linewidth': 0.8,
        'grid.linestyle': '-',        # mai tratteggiata: aggiunge rumore
        'legend.frameon': False,
        'savefig.facecolor': SUPERFICIE,
    })


def pulisci(ax):
    """Griglia sottile, assi recessivi, niente cornice superflua."""
    ax.set_axisbelow(True)
    ax.grid(True, axis='y')
    for lato in ('top', 'right'):
        ax.spines[lato].set_visible(False)
    for lato in ('left', 'bottom'):
        ax.spines[lato].set_color(ASSE)


# ---------------------------------------------------------------- lettura
def numero(valore):
    try:
        return float(valore)
    except (TypeError, ValueError):
        return None


def leggi(percorso):
    with open(percorso, newline='', encoding='utf-8') as f:
        righe = list(csv.DictReader(f))
    if not righe:
        raise SystemExit('%s: vuoto' % percorso)
    mancanti = [c for c in ('jam_x', 'jam_y', 'jam_valido', 'rumore_rf')
                if c not in righe[0]]
    if mancanti:
        raise SystemExit(
            'Il CSV non contiene %s: e stato prodotto da una versione di\n'
            'metrics_node precedente alla registrazione dell\'ingresso del\n'
            'filtro. Serve una prova nuova.' % ', '.join(mancanti))
    return righe


def finestre_disturbo(righe, chiave='rumore_rf', soglia=0.01):
    """Intervalli (inizio, fine) in cui il disturbo e dichiarato attivo."""
    fuori, inizio = [], None
    for r in righe:
        v = numero(r.get(chiave))
        attivo = v is not None and v > soglia
        t = float(r['t_sim'])
        if attivo and inizio is None:
            inizio = t
        elif not attivo and inizio is not None:
            fuori.append((inizio, t))
            inizio = None
    if inizio is not None:
        fuori.append((inizio, float(righe[-1]['t_sim'])))
    return fuori


def finestra_aggancio(righe, massimo=45.0):
    """Il tratto continuo di AGGANCIO piu lungo, al piu `massimo` secondi.

    La restrizione non e una scelta estetica. AGGANCIO e il regime in cui il
    filtro lavora: in PATTUGLIAMENTO non c'e bersaglio da stimare, e in
    RICERCA il filtro e azzerato. Prendere il tratto piu lungo, invece di una
    finestra scelta a occhio, rende la figura leggibile senza che nessuno
    debba fidarsi di dove e stata puntata.
    """
    migliore, corrente = [], []
    for r in righe:
        if r.get('fase') == 'AGGANCIO':
            corrente.append(r)
        else:
            if len(corrente) > len(migliore):
                migliore = corrente
            corrente = []
    if len(corrente) > len(migliore):
        migliore = corrente
    if not migliore:
        return righe
    t0 = float(migliore[0]['t_sim'])
    return [r for r in migliore if float(r['t_sim']) - t0 <= massimo]


def spezza(t, y, salto=0.6):
    """Interrompe la linea dove la serie ha un buco.

    Senza, `plot` unisce i due lati di un'interruzione con un segmento retto,
    cioe disegna un andamento che nel dato non c'e. I buchi qui non sono
    accidenti: sono i fotogrammi in cui il bersaglio non e stato rilevato, ed e
    proprio l'informazione che il grafico deve mostrare.
    """
    tt, yy = [], []
    for k in range(len(t)):
        if k and t[k] - t[k - 1] > salto:
            tt.append(t[k - 1] + salto / 2)
            yy.append(float('nan'))
        tt.append(t[k])
        yy.append(y[k])
    return tt, yy


def ombreggia(ax, finestre, etichetta=False):
    for k, (a, b) in enumerate(finestre):
        ax.axvspan(a, b, color=BANDA, linewidth=0, zorder=0,
                   label='disturbo attivo' if (etichetta and k == 0) else None)


# ------------------------------------------------------ figura 1: stima
def figura_stima(righe, uscita):
    """Stato stimato e matrice R.

    Pannello superiore: la coordinata x del bersaglio nell'immagine, nelle
    unita in cui lavora il filtro. Tre grandezze: cio che il filtro riceve
    (misura disturbata), cio che produce (stima) e cio che sarebbe vero
    (rilevamento pulito, che il filtro non vede mai).

    Pannello inferiore: l'elemento diagonale di R nel tempo. R non e costante:
    cresce con il rumore dichiarato sul datalink, ed e il meccanismo con cui il
    filtro si fida meno della misura quando la misura vale meno. I due
    pannelli condividono l'asse dei tempi perche e quello il punto — la stima
    si irrigidisce nelle stesse finestre in cui R sale.
    """
    t_mis, x_mis = [], []
    t_stima, x_stima = [], []
    t_vero, x_vero = [], []
    t_r, r = [], []
    senza_misura = []

    for riga in righe:
        t = float(riga['t_sim'])
        if riga.get('jam_valido') == '1':
            v = numero(riga['jam_x'])
            if v is not None:
                t_mis.append(t)
                x_mis.append(v)
        if riga.get('trk_valido') == '1':
            v = numero(riga['trk_x'])
            if v is not None:
                t_stima.append(t)
                x_stima.append(v)
            if riga.get('jam_valido') != '1':
                senza_misura.append(t)
        if riga.get('det_valido') == '1':
            v = numero(riga['det_x'])
            if v is not None:
                t_vero.append(t)
                x_vero.append(v)
        rumore = numero(riga.get('rumore_rf'))
        if rumore is not None:
            t_r.append(t)
            r.append(R_BASE + (R_MAX - R_BASE) * rumore)

    finestre = finestre_disturbo(righe)

    fig, (alto, basso) = plt.subplots(
        2, 1, figsize=(9.5, 7.0), sharex=True,
        gridspec_kw={'height_ratios': [2.4, 1.0], 'hspace': 0.18})

    # --- pannello superiore -------------------------------------------------
    ombreggia(alto, finestre)
    alto.plot(*spezza(t_vero, x_vero), color=ACQUA, linewidth=4.5, alpha=0.55,
              zorder=2, solid_capstyle='round')
    alto.plot(t_mis, x_mis, linestyle='none', marker='o', markersize=3.2,
              markerfacecolor=ARANCIO, markeredgecolor='none', alpha=0.75,
              zorder=3)
    alto.plot(*spezza(t_stima, x_stima), color=BLU, linewidth=1.8, zorder=4,
              solid_capstyle='round')

    if senza_misura:
        tutti = x_mis + x_stima + x_vero
        minimo, massimo = min(tutti), max(tutti)
        margine = 0.09 * (massimo - minimo)
        alto.plot(senza_misura, [minimo - margine] * len(senza_misura),
                  linestyle='none', marker='|', markersize=6, color=ARANCIO,
                  alpha=0.9, zorder=2)
        alto.set_ylim(minimo - 1.8 * margine, massimo + 0.5 * margine)
    alto.set_ylabel('coordinata x nell’immagine\n(unità normalizzate)')
    # Il titolo va sopra la legenda, che sta fuori dall'area dei dati.
    alto.set_title('Stima dello stato: il filtro fra una misura disturbata e '
                   'il vero', loc='left', fontsize=11, fontweight='semibold',
                   pad=56)
    pulisci(alto)

    # Etichette dirette: l'acqua sta sotto la soglia di contrasto, e la regola
    # di sollievo chiede che l'identita non dipenda dal colore. Non vanno pero
    # al bordo destro, dove le due curve convergono e le etichette si
    # accavallavano: si mettono dove le curve sono piu LONTANE, che e anche il
    # punto in cui il lettore ha piu bisogno di sapere quale e quale.
    comuni = {}
    for t, x in zip(t_vero, x_vero):
        comuni[round(t, 3)] = x
    separazione = [(abs(x - comuni[round(t, 3)]), t, x, comuni[round(t, 3)])
                   for t, x in zip(t_stima, x_stima)
                   if round(t, 3) in comuni]
    if separazione:
        _, t_eti, x_stima_eti, x_vero_eti = max(separazione)
        sopra_e_sotto = ((x_stima_eti, 'stima', BLU, x_stima_eti > x_vero_eti),
                         (x_vero_eti, 'vero', SECONDARIO, x_vero_eti > x_stima_eti))
        for y, testo, colore, in_alto in sopra_e_sotto:
            alto.annotate(testo, xy=(t_eti, y),
                          xytext=(0, 9 if in_alto else -14),
                          textcoords='offset points', ha='center',
                          fontsize=8, color=SECONDARIO)

    # La legenda sta FUORI dall'area dei dati: dentro copriva la curva.
    alto.legend(handles=[
        Line2D([], [], color=BLU, linewidth=1.8, label='stima del filtro'),
        Line2D([], [], color=ARANCIO, marker='o', markersize=4,
               linestyle='none', label='misura ricevuta (disturbata)'),
        Line2D([], [], color=ACQUA, linewidth=4.5, alpha=0.55,
               label='rilevamento pulito, il vero (il filtro non lo vede)'),
        Line2D([], [], color=ARANCIO, marker='|', markersize=6,
               linestyle='none', label='nessuna misura: il filtro predice'),
        Patch(facecolor=BANDA, label='finestra di disturbo'),
    ], loc='lower left', bbox_to_anchor=(0.0, 1.02), ncols=2, fontsize=8,
        labelcolor=SECONDARIO, borderaxespad=0.0)

    # --- pannello inferiore -------------------------------------------------
    ombreggia(basso, finestre)
    basso.plot(t_r, r, color=BLU, linewidth=1.8, drawstyle='steps-post',
               solid_capstyle='round')
    basso.set_ylabel('R  (elemento diagonale)')
    basso.set_xlabel('tempo di simulazione (s)')
    basso.set_title('Matrice R adattiva:  R = diag(r, r),   '
                    'r = %.2f + %.2f · livello di rumore'
                    % (R_BASE, R_MAX - R_BASE),
                    loc='left', fontsize=10, color=SECONDARIO, pad=8)
    pulisci(basso)
    if r:
        # Spazio sopra il picco, cosi l'annotazione non finisce sul titolo.
        basso.set_ylim(-0.08 * max(r), 1.45 * max(r))
    if r:
        # Il rapporto si calcola, non si scrive a mano: e la stessa disciplina
        # per cui i valori nel README rimandano ai parametri invece di
        # ripeterne il numero.
        picco = max(r)
        basso.annotate('r = %.2f sotto disturbo, %.0f volte il valore a '
                       'riposo (%.2f)' % (picco, picco / R_BASE, R_BASE),
                       xy=(t_r[r.index(picco)], picco), xytext=(8, 8),
                       textcoords='offset points', fontsize=8,
                       color=SECONDARIO)

    fig.text(0.01, 0.012,
             'Tratto di AGGANCIO continuo piu lungo. Il rilevamento pulito e '
             'registrato a parte: il filtro non lo riceve mai.',
             fontsize=7, color=SPENTO)
    # Lo spazio in alto va riservato: il titolo sta sopra una legenda di tre
    # righe, e senza margine finiva tagliato dal bordo.
    fig.tight_layout()
    fig.savefig(uscita, dpi=200, bbox_inches='tight', pad_inches=0.22)
    plt.close(fig)
    print('scritto', uscita)


# ------------------------------------------------- figura 2: errore in metri
def posizione_mondo(riga, u, v):
    """Posizione del bersaglio nel mondo, con la matematica di controller_node.

    Si usa la quota e la posa VERE del velivolo, non quelle stimate: cosi la
    differenza fra le due curve resta l'effetto del filtro e non vi si somma
    l'errore di navigazione, che e lo stesso per entrambe.
    """
    quota = numero(riga.get('gt_drone_z'))
    roll = numero(riga.get('roll'))
    pitch = numero(riga.get('pitch'))
    yaw = numero(riga.get('yaw'))
    g_roll = numero(riga.get('gimbal_roll')) or 0.0
    g_pitch = numero(riga.get('gimbal_pitch')) or 0.0
    dx = numero(riga.get('gt_drone_x'))
    dy = numero(riga.get('gt_drone_y'))
    if None in (quota, roll, pitch, yaw, dx, dy) or quota < 5.0:
        return None

    # Assetto della telecamera: corpo piu giunto.
    alpha_x = math.atan(u * TAN_O) - (roll + g_roll)
    alpha_y = math.atan(v * TAN_V) + (pitch + g_pitch)
    limite = 1.4
    alpha_x = max(-limite, min(limite, alpha_x))
    alpha_y = max(-limite, min(limite, alpha_y))
    err_x = (math.tan(alpha_x) / TAN_O) * quota * TAN_O
    err_y = (math.tan(alpha_y) / TAN_V) * quota * TAN_V

    avanti, laterale = -err_y, -err_x
    c, s = math.cos(yaw), math.sin(yaw)
    return (dx + avanti * c - laterale * s,
            dy + avanti * s + laterale * c)


def errore(riga, u, v):
    p = posizione_mondo(riga, u, v)
    bx, by = numero(riga.get('gt_target_x')), numero(riga.get('gt_target_y'))
    if p is None or bx is None or by is None:
        return None
    return math.hypot(p[0] - bx, p[1] - by)


def mediana(v):
    v = sorted(v)
    n = len(v)
    if not n:
        return float('nan')
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


def _sotto_disturbo(riga, soglia=0.01):
    v = numero(riga.get('rumore_rf'))
    return v is not None and v > soglia


def _statistiche(coppie):
    """(mediana senza, mediana con, p90 senza, p90 con, quota di vittorie)."""
    if not coppie:
        return None
    senza = [a for a, _ in coppie]
    con = [b for _, b in coppie]

    def p90(v):
        return sorted(v)[min(int(0.9 * len(v)), len(v) - 1)]

    meglio = sum(1 for a, b in coppie if b < a)
    return (mediana(senza), mediana(con), p90(senza), p90(con),
            100.0 * meglio / len(coppie), len(coppie))


def figura_errore(righe, uscita):
    """Errore di posizione con e senza filtro.

    Entrambe le curve sono la distanza, in metri al suolo, fra il bersaglio
    vero e dove lo si crede. Cambia solo da dove viene la coordinata immagine:
    dalla misura disturbata che il rilevatore consegna, oppure dalla stima del
    filtro. Quota, assetto, imbardata e posa del velivolo sono i valori veri per
    entrambe, quindi la differenza fra le due e il filtro e nient'altro.

    Il confronto e APPAIATO — solo istanti in cui esistono entrambe — e
    CONDIZIONATO al disturbo. Appaiato perche il filtro produce una posizione
    anche quando misura non ce n'e, e quegli istanti sono i piu difficili:
    metterli in una mediana confrontata con l'alternativa la penalizzerebbe per
    aver risposto dove l'altra taceva. Condizionato perche fuori dalle finestre
    di disturbo il jammer lascia passare la misura intatta, e li il filtro puo
    solo aggiungere ritardo: una mediana unica sarebbe la media di due risposte
    opposte.
    """
    t_senza, e_senza = [], []
    t_con, e_con = [], []
    coppie = {'disturbo': [], 'pulito': []}
    solo_filtro = []

    for riga in righe:
        t = float(riga['t_sim'])
        es = ec = None
        if riga.get('jam_valido') == '1':
            u, v = numero(riga['jam_x']), numero(riga['jam_y'])
            if u is not None and v is not None:
                es = errore(riga, u, v)
        if riga.get('trk_valido') == '1':
            u, v = numero(riga['trk_x']), numero(riga['trk_y'])
            if u is not None and v is not None:
                ec = errore(riga, u, v)
        if es is not None:
            t_senza.append(t)
            e_senza.append(es)
        if ec is not None:
            t_con.append(t)
            e_con.append(ec)
        if es is not None and ec is not None:
            coppie['disturbo' if _sotto_disturbo(riga) else 'pulito'].append(
                (es, ec))
        elif ec is not None:
            solo_filtro.append(t)

    finestre = finestre_disturbo(righe)
    fig, (alto, basso) = plt.subplots(
        2, 1, figsize=(9.5, 7.4),
        gridspec_kw={'height_ratios': [1.5, 1.0], 'hspace': 0.62})

    # --- nel tempo ----------------------------------------------------------
    ombreggia(alto, finestre)
    alto.plot(t_senza, e_senza, linestyle='none', marker='o', markersize=3.0,
              markerfacecolor=ARANCIO, markeredgecolor='none', alpha=0.8,
              zorder=3)
    alto.plot(*spezza(t_con, e_con), color=BLU, linewidth=1.8, zorder=4,
              solid_capstyle='round')
    if solo_filtro:
        alto.plot(solo_filtro, [0.0] * len(solo_filtro), linestyle='none',
                  marker='|', markersize=6, color=ARANCIO, alpha=0.9, zorder=2)
    alto.set_ylabel('errore di posizione (m)')
    alto.set_xlabel('tempo di simulazione (s)')
    alto.set_title('Errore di posizione nel tempo', loc='left', fontsize=11,
                   fontweight='semibold', pad=34)
    pulisci(alto)
    alto.legend(handles=[
        Line2D([], [], color=BLU, linewidth=1.8, label='con filtro di Kalman'),
        Line2D([], [], color=ARANCIO, marker='o', markersize=4,
               linestyle='none', label='senza filtro (misura grezza)'),
        Line2D([], [], color=ARANCIO, marker='|', markersize=6,
               linestyle='none',
               label='nessuna misura: senza filtro, nessuna posizione'),
        Patch(facecolor=BANDA, label='finestra di disturbo'),
    ], loc='lower left', bbox_to_anchor=(0.0, 1.02), ncols=2, fontsize=8,
        labelcolor=SECONDARIO, borderaxespad=0.0)

    # --- confronto condizionato --------------------------------------------
    # Barre orizzontali: mediana. La tacca oltre la punta e il 90esimo
    # percentile, cioe quanto va male quando va male — che per un filtro conta
    # quanto il caso tipico.
    gruppi = [('sotto disturbo', coppie['disturbo']),
              ('misura pulita', coppie['pulito'])]
    altezza = 0.3
    posizioni, etichette = [], []
    for k, (nome, dati) in enumerate(gruppi):
        st = _statistiche(dati)
        base = k * 1.1
        posizioni.append(base)
        if st is None:
            etichette.append('%s\n(nessun campione)' % nome)
            continue
        med_s, med_c, p90_s, p90_c, vittorie, n = st
        etichette.append(nome)
        for spostamento, med, p90v, colore, nome_serie in (
                (+altezza / 1.7, med_s, p90_s, ARANCIO, 'senza filtro'),
                (-altezza / 1.7, med_c, p90_c, BLU, 'con filtro')):
            y = base + spostamento
            basso.barh(y, med, height=altezza, color=colore, zorder=3)
            basso.plot([med, p90v], [y, y], color=colore, linewidth=1.4,
                       alpha=0.45, zorder=2)
            basso.plot([p90v, p90v], [y - altezza / 2.4, y + altezza / 2.4],
                       color=colore, linewidth=1.6, alpha=0.6, zorder=4)
            basso.annotate('%.1f m' % med, xy=(med, y), xytext=(6, 0),
                           textcoords='offset points', va='center',
                           fontsize=8, color=SECONDARIO)
        # Numerosita e quota di vittorie sotto la coppia, dentro il riquadro:
        # nell'etichetta dell'asse venivano tagliate dal bordo.
        basso.annotate('%d istanti · il filtro è più vicino al vero nel %.0f%%'
                       % (n, vittorie),
                       xy=(0, base + altezza * 1.35), xytext=(2, 0),
                       textcoords='offset points', va='center',
                       fontsize=7.5, color=SPENTO)
    basso.set_yticks(posizioni)
    basso.set_yticklabels(etichette, fontsize=9, color=SECONDARIO)
    basso.set_ylim(posizioni[-1] + altezza * 2.1, posizioni[0] - altezza * 1.3)
    basso.set_xlabel('errore mediano (m) — la tacca è il 90° percentile')
    basso.set_title('Il filtro serve dove la misura è corrotta, e costa '
                    'ritardo dove non lo è', loc='left', fontsize=10,
                    color=SECONDARIO, pad=8)
    basso.grid(True, axis='x')
    basso.grid(False, axis='y')
    for lato in ('top', 'right', 'left'):
        basso.spines[lato].set_visible(False)
    basso.spines['bottom'].set_color(ASSE)
    basso.set_axisbelow(True)

    nota = ('Una prova, tratto di AGGANCIO continuo più lungo. Confronto '
            'appaiato: solo gli istanti in cui esistono entrambe le posizioni.')
    if solo_filtro:
        totale = len(t_con)
        nota += ('\nIn altri %d istanti su %d (%.0f%%) una misura non arriva '
                 'affatto: lì il filtro è l’unica posizione disponibile, e '
                 'non entra in questo confronto.'
                 % (len(solo_filtro), totale,
                    100.0 * len(solo_filtro) / totale))
    fig.text(0.01, 0.010, nota, fontsize=7, color=SPENTO, linespacing=1.5)
    fig.tight_layout()
    fig.savefig(uscita, dpi=200, bbox_inches='tight', pad_inches=0.22)
    plt.close(fig)
    print('scritto', uscita)

    for nome, dati in gruppi:
        st = _statistiche(dati)
        if st is None:
            print('  %-16s nessun campione appaiato' % nome)
            continue
        med_s, med_c, p90_s, p90_c, vittorie, n = st
        print('  %-16s n=%3d  senza %5.2f (p90 %5.2f)  con %5.2f (p90 %5.2f)'
              '  filtro meglio %.0f%%'
              % (nome, n, med_s, p90_s, med_c, p90_c, vittorie))
    print('  istanti coperti dal solo filtro: %d' % len(solo_filtro))


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    righe = leggi(argv[1])
    tratto = finestra_aggancio(righe)
    print('finestra: %.1f-%.1f s di simulazione, %d campioni'
          % (float(tratto[0]['t_sim']), float(tratto[-1]['t_sim']), len(tratto)))
    cartella = argv[2] if len(argv) > 2 else os.path.dirname(argv[1]) or '.'
    os.makedirs(cartella, exist_ok=True)
    stile()
    figura_stima(tratto, os.path.join(cartella, 'kalman_stima.png'))
    figura_errore(tratto, os.path.join(cartella, 'kalman_errore.png'))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
