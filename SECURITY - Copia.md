# Politica di sicurezza

## Natura del progetto

Questo è software di **simulazione** per ricerca accademica. Non è certificato per
il volo, non è stato sottoposto a revisione di sicurezza funzionale e non
implementa alcuna delle protezioni che un sistema di controllo di volo operativo
richiede.

Le sezioni seguenti distinguono i rischi informatici da quelli fisici, perché in
un progetto di controllo di volo i secondi sono i più gravi.

## Rischi fisici: prima di toccare hardware reale

**`ARMING_SKIPCHK 1` non va mai portato su un velivolo reale.** In simulazione
disattiva controlli su sensori finti; su hardware disattiva esattamente le
verifiche che impediscono il decollo con una IMU guasta o una bussola scalibrata.
Il valore è presente in `docker/sitl-defaults.parm` **solo** perché quel file
serve al simulatore.

Altre precauzioni, se un giorno l'architettura venisse portata su un Pixhawk:

- Radiocomando di override sempre attivo, con pilota pronto a riprendere il
  controllo. I nodi pubblicano setpoint di velocità senza alcun limite geografico.
- `vel_max` in `controller_node` vale 8 m/s, tarato per la simulazione. Sul campo
  va ridotto drasticamente per i primi voli.
- Nessun geofence, nessuna logica di rientro, nessun controllo di batteria sono
  implementati in questo progetto: dipendono interamente dalla configurazione
  dell'autopilota.
- Il rilevamento è cromatico e può agganciare qualunque oggetto rosso, incluse
  persone che indossano rosso. Non usarlo per inseguimenti autonomi in presenza
  di persone.

## Rischi informatici

**MAVLink non è autenticato né cifrato.** Chiunque possa raggiungere le porte del
simulatore può armare il velivolo, cambiarne la modalità e comandarne il
movimento. Il `docker-compose.yml` espone:

| Porta | Servizio |
|---|---|
| 5760/tcp | MAVLink diretto dal SITL |
| 14550/udp | uscita MAVProxy verso ground control station |

Sono pubblicate per comodità di sviluppo, su una simulazione. **Non esporle a reti
non fidate** e non replicare quella configurazione con hardware collegato.

**Il container gira come root** e monta cartelle del repository. È adeguato a un
ambiente di sviluppo locale, non a un host condiviso o esposto.

**I file SDF di terze parti sono codice.** I mondi e i modelli Gazebo possono
caricare plugin che eseguono codice arbitrario nel processo del simulatore. Non
aprire file SDF di provenienza sconosciuta.

## Segnalare una vulnerabilità

Non aprire una issue pubblica. Scrivi a **riccardo.mahdavi@gmail.com** indicando:

- il componente interessato e in quale ambiente si riproduce;
- i passi per riprodurre il problema;
- l'impatto che ritieni possibile.

Riceverai un riscontro entro **14 giorni**. Trattandosi di un progetto
universitario mantenuto da una sola persona, non esistono garanzie di tempi di
correzione né un processo di rilascio di patch.

## Versioni supportate

Viene mantenuto il solo ramo `main`. Non esistono release con supporto esteso.

## Dipendenze

Il progetto dipende da ArduPilot, Gazebo, ROS 2 e MAVROS. Le vulnerabilità di
questi componenti vanno segnalate ai rispettivi progetti, che hanno processi di
divulgazione propri.
