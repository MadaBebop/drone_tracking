#!/usr/bin/env python3
"""
Monta un video di registra_video.py tenendo solo i tratti con il bersaglio in
vista, con qualche secondo di contesto prima e dopo. I tratti tagliati vengono
sostituiti da un cartello che dice quanto tempo e stato saltato: la giuria
deve sapere che il video e montato, non scoprirlo.

    monta_video.py video.mp4 [prima_s] [dopo_s]

    monta_video.py /ws/snapshots/demo_20261112_152845.mp4
    monta_video.py /ws/snapshots/demo_20261112_152845.mp4 4 3

Scrive accanto all'originale un file *_montato.mp4. L'originale resta intatto.

Il bersaglio si riconosce dai segni che il rilevatore disegna sul fotogramma:
il contorno verde attorno al blob e il rosso del bersaglio stesso. La fascia
in alto con fase e quota viene esclusa, perche e testo bianco su nero e non
c'entra. Si conta ogni fotogramma visibile, anche quando il rilevatore non lo
riquadra: la perdita e la ripresa dell'aggancio sono la parte interessante.

Su un portatile lento la registrazione di una missione e quasi tutta
pattugliamento a vuoto: nella prova del 12 novembre il bersaglio si vedeva in
11 secondi su 150. Il montaggio rende il video proiettabile senza far
aspettare nessuno.
"""
import os
import sys

import cv2
import numpy as np

PRIMA_S = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
DOPO_S = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
# Due tratti piu vicini di cosi si uniscono: un cartello che salta un secondo
# disturberebbe piu di quel secondo di erba.
UNISCI_S = 4.0
CARTELLO_S = 1.5
# Altezza della fascia di testo disegnata da registra_video.py: 34 px con una
# riga, 60 con la riga della stazione a terra. Si esclude la piu alta.
FASCIA_PX = 62
# Pixel minimi perche un colore conti: sotto, e rumore di compressione. Con il
# bersaglio lontano il riquadro verde e piccolo: a 20 si perdevano i tratti di
# aggancio. L'erba non supera mai il verde 180 e il disturbo e bianco, quindi
# una soglia bassa non porta falsi.
MIN_PX = 6


def bersaglio_visibile(img):
    corpo = img[FASCIA_PX:, :].astype(np.int16)
    b, g, r = corpo[..., 0], corpo[..., 1], corpo[..., 2]
    contorno_verde = np.count_nonzero((g > 180) & (r < 90) & (b < 90))
    rosso = np.count_nonzero((r > 150) & (g < 70) & (b < 70))
    return contorno_verde > MIN_PX or rosso > MIN_PX


def tratti_visibili(percorso):
    """Indici dei fotogrammi con il bersaglio, raggruppati in tratti."""
    cap = cv2.VideoCapture(percorso)
    tratti, inizio, i = [], None, 0
    while True:
        ok, img = cap.read()
        if not ok:
            break
        visto = bersaglio_visibile(img)
        if visto and inizio is None:
            inizio = i
        elif not visto and inizio is not None:
            tratti.append((inizio, i - 1))
            inizio = None
        i += 1
    if inizio is not None:
        tratti.append((inizio, i - 1))
    cap.release()
    return tratti, i


def allarga_e_unisci(tratti, totale, fps):
    prima, dopo, unisci = int(PRIMA_S * fps), int(DOPO_S * fps), int(UNISCI_S * fps)
    larghi = [(max(0, a - prima), min(totale - 1, b + dopo)) for a, b in tratti]
    uniti = []
    for a, b in larghi:
        if uniti and a - uniti[-1][1] <= unisci:
            uniti[-1] = (uniti[-1][0], max(uniti[-1][1], b))
        else:
            uniti.append((a, b))
    return uniti


def cartello(dimensione, testo):
    larghezza, altezza = dimensione
    img = np.zeros((altezza, larghezza, 3), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(testo, font, 0.7, 2)
    cv2.putText(img, testo, ((larghezza - tw) // 2, (altezza + th) // 2), font,
                0.7, (255, 255, 255), 2, cv2.LINE_AA)
    return img


def durata(n_frame, fps):
    s = int(round(n_frame / fps))
    return f'{s // 60} min {s % 60} s' if s >= 60 else f'{s} s'


def main():
    if len(sys.argv) < 2:
        print(__doc__.strip().split('\n\n')[1], file=sys.stderr)
        sys.exit(2)
    sorgente = sys.argv[1]
    base, _ = os.path.splitext(sorgente)
    destinazione = base + '_montato.mp4'

    cap = cv2.VideoCapture(sorgente)
    if not cap.isOpened():
        print(f'Non riesco ad aprire {sorgente}', file=sys.stderr)
        sys.exit(1)
    fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
    dimensione = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                  int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    cap.release()

    print(f'Analizzo {sorgente} ...', flush=True)
    tratti, totale = tratti_visibili(sorgente)
    if not tratti:
        print('Il bersaglio non compare mai: niente da montare.', file=sys.stderr)
        sys.exit(1)
    tenuti = allarga_e_unisci(tratti, totale, fps)

    scrittore = cv2.VideoWriter(destinazione, cv2.VideoWriter_fourcc(*'mp4v'),
                                fps, dimensione)
    n_cartello = int(CARTELLO_S * fps)
    cap = cv2.VideoCapture(sorgente)
    i, scritti, prossimo = 0, 0, 0
    for a, b in tenuti:
        if a > prossimo:
            img = cartello(dimensione,
                           f'... {durata(a - prossimo, fps)} senza bersaglio in vista ...')
            for _ in range(n_cartello):
                scrittore.write(img)
            scritti += n_cartello
        while i < a:
            cap.grab()
            i += 1
        while i <= b:
            ok, img = cap.read()
            if not ok:
                break
            scrittore.write(img)
            scritti += 1
            i += 1
        prossimo = b + 1
    if prossimo < totale:
        img = cartello(dimensione,
                       f'... altri {durata(totale - prossimo, fps)} senza bersaglio ...')
        for _ in range(n_cartello):
            scrittore.write(img)
        scritti += n_cartello
    cap.release()
    scrittore.release()

    visti = sum(b - a + 1 for a, b in tratti)
    print(f'Fatto: {destinazione}')
    print(f'  originale {durata(totale, fps)}, bersaglio in vista per '
          f'{durata(visti, fps)}')
    print(f'  montato {durata(scritti, fps)}: {len(tenuti)} tratti')
    for a, b in tenuti:
        print(f'    da {a / fps:6.1f} s a {b / fps:6.1f} s dell originale')


if __name__ == '__main__':
    main()
