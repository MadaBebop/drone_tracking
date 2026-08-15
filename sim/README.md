# Asset di simulazione personalizzati

Questa cartella contiene **solo** i file Gazebo modificati per il progetto.
ArduPilot, il plugin `ardupilot_gazebo` e i modelli standard **non** vanno qui:
il `Dockerfile` li scarica e compila da sorgente.

```
sim/
├── worlds/
│   └── iris_runway.sdf        # mondo con pista e sfera rossa "bersaglio" (inline)
└── models/
    └── iris_with_ardupilot/   # drone con telecamera inclinata a 60°
        ├── model.sdf
        └── model.config
```

Il bersaglio **non è un modello separato**: è dichiarato inline dentro il world,
quindi non esiste (né serve) una cartella `models/bersaglio/`.

`iris_with_standoffs` e `runway`, inclusi via `model://`, arrivano da upstream
`ardupilot_gazebo` e sono già presenti nell'immagine.

In fase di build questi file vengono copiati **sopra** quelli upstream di
`ardupilot_gazebo`, quindi devono mantenere gli stessi nomi degli originali.

Se la cartella è vuota l'immagine si costruisce comunque, ma userà il mondo
standard di `ardupilot_gazebo`: **senza bersaglio rosso e senza telecamera**,
quindi `detector_node` non rileverà nulla.

## Come popolarla

Dalla VM Ubuntu dove il progetto gira nativamente:

```bash
./scripts/export_sim_assets.sh "/home/mada/Desktop/Progetto Drone/ardupilot_gazebo"
```

Oppure copiando i due percorsi a mano:

| Sorgente sulla VM | Destinazione |
|---|---|
| `ardupilot_gazebo/worlds/iris_runway.sdf` | `sim/worlds/` |
| `ardupilot_gazebo/models/iris_with_ardupilot/` | `sim/models/` |

## Verifica

Il nome del mondo conta: `target_mover_node` chiama il servizio
`/world/iris_runway/set_pose`, quindi l'attributo `<world name="...">` dell'SDF
deve restare `iris_runway`, e la sfera deve chiamarsi `bersaglio`. Se non
corrispondono il nodo lo segnala a log (`set_pose rifiutato dal simulatore`).

La sfera deve inoltre restare `<static>true</static>`: il suo moto è comandato
interamente dal nodo, e lasciandola dinamica la fisica la fa rotolare via fra un
comando e l'altro.

Il topic della telecamera dev'essere `/drone/camera/image_raw`: è quello
che `tracking.launch.py` passa a `ros_gz_bridge`.
