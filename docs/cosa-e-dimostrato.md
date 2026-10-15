# Cosa questo progetto dimostra, e cosa no

Il README descrive il sistema e racconta ogni misura nel dettaglio in cui è
stata fatta. Questo documento fa l'operazione inversa: prende le affermazioni
che il progetto può sostenere, mette accanto a ciascuna il numero che la regge,
e separa nettamente quelle dimostrate da quelle che non lo sono.

Serve per due ragioni. La prima è che un lettore deve poter distinguere in
pochi minuti un risultato da un'aspettativa. La seconda è che diverse
affermazioni plausibili di questo progetto si sono rivelate false quando sono
state misurate, e dichiararlo è più utile che nasconderlo.

Convenzione usata qui: un risultato è **dimostrato** se non dipende da una
media fra prove, oppure se la differenza misurata supera la dispersione
quantificata in `scripts/dispersione.py`. Tutto il resto è **indicativo**, e
viene segnalato come tale.

---

## 1. Dimostrato

### Il livello di sicurezza interviene, e si misura quanto costa fermarsi

Con il confine abbassato a 250 m perché l'inseguimento lo superasse davvero, il
blocco è scattato a 258 m, il velivolo è passato in `RTL`, è rientrato
all'origine e non ha più ripreso l'inseguimento benché il bersaglio continuasse
a orbitare a portata.

Due numeri ne escono, entrambi utili a chi deve configurare il sistema:

* l'escursione oltre il confine è stata di **59 m**, da 250 a 308.8, perché a
  20 m/s fermarsi e invertire richiede tempo. Un geofence non è una barriera,
  è la soglia oltre la quale si comincia a fermarsi;
* `raggio_max` deve superare la diagonale del circuito, 212 m per 150 × 150.
  Con il confine a 200 m il blocco scattava durante il normale pattugliamento.

Non serve statistica: o il velivolo è rientrato o non lo è.

### La latenza della catena di percezione ha un costo, e il filtro non la vede

Iniettando 0.15 s di ritardo fra otturatore e rilevamento, con due prove per
configurazione:

| | 0 s | 0.15 s |
|---|---|---|
| errore sulla stima di velocità | 5.6 e 5.9 m/s | 7.6 e 7.3 m/s |
| bersaglio inquadrato in `AGGANCIO` | 67 % e 74 % | 54 % e 55 % |
| durata mediana dell'aggancio | 38.0 s | 8.4 s |

Il conto torna con la geometria: a 10 m/s, 0.15 s valgono 1.5 m, che a 50 m di
quota sono 0.03 in coordinate normalizzate, circa il doppio del rumore di
misura. Ma è un errore costante e direzionale, quindi non si media via.

Il risultato che conta di più è negativo: **il NIS non si muove**, 1.05 senza
ritardo contro 1.03 e 1.07 con. Un ritardo puro applicato a una traiettoria
regolare è ancora una traiettoria regolare, quindi le innovazioni restano
coerenti e il filtro è sano e sbagliato allo stesso tempo. Un difetto che
nessun controllo a tempo d'esecuzione può vedere va tolto per costruzione, e
questa è la ragione per cui i contratti fra i nodi sono stati marcati
temporalmente.

La latenza vera della catena, misurabile solo dopo quella modifica, è di
**33 ms sul ponte della telecamera e 2 ms sul rilevamento**.

### La stima di velocità è migliorata di un terzo, e resta il limite

L'errore sulla stima della velocità del bersaglio valeva **10.8 m/s contro un
segnale di 10.0**, su 5426 campioni: la stima non portava informazione. Dopo
aver fornito al filtro il moto del velivolo come ingresso noto e aver tarato
`Q` e `R` sulla misura invece che a occhio, è sceso a **6.7 m/s**, rapporto
0.67.

Il pavimento teorico calcolato sul rumore di misura è **2.6 m/s**, quindi
restano un fattore due e mezzo di distanza. Questa misura non risente della
dispersione fra prove, perché nasce da migliaia di campioni dentro ogni prova.

### Quattro difetti categorici, corretti

* la spirale di ricerca isotropa **non chiudeva la distanza**: la minima era
  uguale a quella iniziale in tre prove su cinque;
* i riagganci del criterio precedente erano ciechi, con **zero** fotogrammi
  effettivamente rilevati su tre episodi;
* tre stime di velocità su quattordici erano **fisicamente impossibili**, e ora
  vengono rifiutate dal controllo di plausibilità;
* con il limite attivo **nessuna** ricerca si è allontanata dal bersaglio.

Sono conteggi, non medie: o il fotogramma c'era o non c'era.

### I controlli di arming funzionano con tutti i controlli attivi

Il progetto ha volato a lungo con `ARMING_SKIPCHK 1`, cioè spegnendone una
ventina per aggirarne uno. Con la EEPROM cancellata, così che il file dei
parametri fosse autoritativo, il primo tentativo è stato rifiutato con
`Arm: Gyros inconsistent` e il secondo accettato, con decollo regolare a 43 m.
Il ciclo di ritentativi già presente assorbe il caso.

### Il livello di sicurezza si accorge se la telemetria muore

Fino al 16/10 il nodo memorizzava l'ultima posa ricevuta senza guardarne
l'età: con il ponte verso l'autopilota morto a metà volo i valori restavano
congelati all'ultima lettura buona, che era dentro i limiti, e la sorveglianza
restava accesa e muta. Era un'omissione e non una scelta, perché il nodo di
controllo e quello di missione avevano già lo stesso controllo.

Verificato in volo uccidendo `mavros_node` a quota 49.7 m con la missione in
corso: il blocco è scattato dopo **1.8 secondi**, coerente con il timeout di
1.5 s valutato a 2 Hz, con motivo `telemetria ferma da 1.8 s: la posizione non
è più nota`, e il nodo di controllo ha azzerato il comando. La reazione
arriva quindi a valle e non si ferma al nodo che se ne accorge.

Il controllo precede tutti gli altri limiti, e non per ordinamento estetico:
gli altri leggono valori memorizzati, e su dati vecchi risponderebbero entro i
limiti guardando dove il velivolo era. Resta scoperto il caso di una
telemetria che muore durante la salita, prima della quota operativa e prima
che arrivi uno stato di armato.

### Il bersaglio reattivo rende due prove identiche due esperimenti diversi

Misurando la distanza fra le posizioni del bersaglio in prove con la stessa
configurazione e lo stesso seme: **da 44 a 357 metri dopo sessanta secondi**,
con una crescita di circa un fattore dieci ogni venticinque secondi. È la firma
di un anello chiuso sensibile alle condizioni iniziali, inevitabile quando il
bersaglio scappa dal drone e il drone insegue il bersaglio.

Con il modo `prefissato`, in cui la traiettoria è funzione del solo tempo, la
stessa distanza scende a **1 metro**, e la traiettoria registrata coincide con
il profilo teorico entro **5 cm**.

### La guida predittiva non aiuta, e costa

Nove voli, tre configurazioni disposte a blocchi, confronto appaiato giro per
giro. Con `k_anticipo` a 1.0 contro la base:

| | giro 1 | giro 2 | giro 3 | medio | intervallo 95 % |
|---|---|---|---|---|---|
| bersaglio inquadrato | +0.4 | −0.7 | +0.5 | **+0.1 %** | −1.6 … +1.7 |
| distanza in `AGGANCIO` | +3.0 | +2.6 | +3.3 | **+2.97 m** | +2.0 … +3.9 |

Sull'inquadratura l'effetto è nullo misurato con precisione, non semplicemente
non osservato. Sulla distanza è reale e nel verso sbagliato.

---

## 2. Non dimostrato

### La resistenza alla negazione del GNSS

È l'affermazione originale del progetto, e va ritirata. Il disturbo è stato
iniettato nei parametri del simulatore, quindi sul sistema vero e non a livello
di topic, in tre modi: disturbo, negazione totale, falsificazione del fix.

Con il ricevitore spento per 60 secondi, lo scarto fra la stima di posizione
dell'autopilota e la verità di Gazebo **resta sotto il metro e mezzo**, lo
stesso valore che si misura con il GPS sano. Un offset falsificato di 22 m si
vede sul fix grezzo ma la stima non lo segue, perché l'EKF rifiuta la misura
incoerente con barometro e inerziali.

Il sistema quindi non fallisce: semplicemente **non c'è nulla a cui resistere**,
e una capacità che non si può mettere alla prova non si può dichiarare.

### L'inseguimento cieco

Con `durata_inseguimento_cieco_s` a 8.0, nessuna differenza supera il rumore. Il
dato interessante è però la forma delle differenze, che **cambiano segno** fra
un giro e l'altro: nel primo peggiora l'inquadratura di 14 punti, nel secondo la
migliora di 17. Non sposta la media, allarga la dispersione. Per una funzione
che interviene quando il bersaglio è già perso, è una proprietà peggiore di un
piccolo danno medio.

### La velocità a cui l'inseguimento cede

Una spazzata su cinque velocità del bersaglio, da 12 a 36 m/s, due prove
ciascuna. Le due grandezze non condizionate all'aggancio hanno la stessa forma,
salgono fino a 24 m/s e poi dimezzano, e la soglia cade appena sopra i 20 m/s
di velocità massima del drone. Ma nessun confronto preso singolarmente supera
il rumore, e la distanza alla fine di ogni fuga ha una dispersione del 46 % fra
episodi della stessa velocità.

Indicativo, non dimostrato. Il disegno che servirebbe è noto e quantificato:
tre velocità, prove da 300 s, tre per dose, diciotto episodi per velocità.

### Qualunque cosa sull'hardware

Il sistema non ha mai volato. La latenza è stata iniettata, non subita. Gli IMU
girano con il rumore azzerato. Il limite di batteria è verificato a banco ma
mai esercitato in volo, perché il SITL non riporta la percentuale di carica.

---

## 3. Il contributo di metodo

È la parte meno visibile e probabilmente la più solida.

**Le grandezze d'esito di questo progetto, per come veniva misurato all'inizio,
non potevano distinguere quasi nulla.** Quantificato: con tre prove per
configurazione servivano differenze del 59 % sull'errore di velocità, del 116 %
sul tempo agganciato, del **206 %** sulla durata dell'aggancio. Per vedere un
effetto del 25 % sarebbero servite da 8 a 87 prove per configurazione, cioè da
due a ventisei ore di macchina.

Da lì sono seguiti tre interventi, ognuno con il suo effetto misurato:

1. **stimolo ripetibile**, che porta la dispersione su distanza e visibilità dal
   21 e 20 per cento al 2 e 7, e le prove necessarie da 12 e 11 a 2 e 3;
2. **disegno a blocchi**, cioè tutte le configurazioni dentro ogni giro invece
   che in sequenza. Senza, la caduta comune del secondo giro, dovuta al carico
   della macchina, avrebbe nascosto il risultato sulla guida predittiva;
3. **confronto appaiato** giro per giro invece che fra medie, che è ciò che
   estrae il segnale: sulla distanza la dispersione dentro i gruppi era del
   24 %, ma le differenze appaiate erano +3.0, +2.6, +3.3, cioè uno scarto di
   36 centimetri.

Nel farlo sono emersi difetti che nessuno cercava e che falsavano misure già
pubblicate nel README: un NIS riscritto a ogni riga del registro, che contava
più volte lo stesso campione e aveva fatto sembrare fallita una taratura
riuscita; cinque parametri che sembravano modificabili a caldo e non lo erano,
con il comando che riusciva e la rilettura che confermava; un bersaglio che
all'inizio di ogni fuga frenava a **48 m/s², quasi cinque g**, presentando al
filtro una discontinuità che nessun veicolo produce, proprio nell'istante su
cui si misura tutto.

---

## 4. I limiti che restano

**Il rilevatore riconosce un colore, non un oggetto.** In campo aperto
aggancerebbe un tetto, un cartello, un'altra auto rossa. Il controllo di
plausibilità scarta ciò che ha la taglia sbagliata, quindi ferma il tetto ma non
l'auto. È il limite che da solo rende il sistema non utilizzabile, e non è
questione di taratura.

**La geometria assume terreno piatto.** La quota usata è quella sul punto di
decollo, e il bersaglio è assunto su quel piano: su un rilievo di 20 m con il
drone a 50, la distanza stimata sbaglia del 40 %, e quella stima scala
direttamente l'errore di controllo.

**Il soffitto percettivo è ~90 m**, misurato dalla legge con cui l'area
apparente cala con il quadrato della quota. È più basso del tetto regolamentare
di 120 m, quindi l'inviluppo utile è più stretto di quanto la norma suggerisca.

**Il bersaglio si muove per teletrasporto.** L'accelerazione è limitata perché
le è stato imposto un limite, ma nulla lega il raggio di sterzata alla velocità
e non esiste una strada.
