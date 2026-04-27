## Analisi criticità e punti aperti — Proposta Architettura Mappa 3D

**Documento analizzato:** `cartesian_velocity_controller/docs/Proposta_Architettura_Mappa_3D.md` (v2.0, 19 Gennaio 2026)  
**Contesto implementativo reale:** `cartesian_velocity_controller` (pipeline esistente: `RepulsionDataManager` → `LocalPlanner` + `MarkerPublisher`)

---

### Scopo di questa analisi

Valutare la proposta implementativa “mappa 3D per obstacle avoidance” ed evidenziare:

- **criticità tecniche** (bug probabili / mismatch con il codice attuale)
- **assunzioni implicite** (che vanno rese esplicite prima di implementare)
- **punti aperti** (decisioni ancora non chiuse)
- **raccomandazioni concrete** (scelte e convenzioni che evitano ambiguità)

Nota: altri `.md` nel repo potrebbero essere non aggiornati; qui faccio riferimento al codice attuale del pacchetto.

---

### Riassunto esecutivo (high-signal)

- **Architettura generale valida**: griglia locale + EDT + double buffer è una buona base per query a 100 Hz con update a 10 Hz.
- **Rischio principale**: **ambiguità su frame e semantica dei vettori/distanze**. Se non fissata, porta a repulsioni invertite, marker sbagliati, tuning “saltato”.
- **Secondo rischio**: **PlanningScene non è solo primitive** (mesh frequenti). La proposta copre solo `sphere/box/cylinder`: se non definisci una strategia, “spariscono” ostacoli.
- **Terzo rischio**: la proposta introduce campi non presenti nelle strutture dati reali (`ObstacleInfo`, `LinkPOI`): serve una “spec” di interfaccia compatibile con `LocalPlanner` e `MarkerPublisher`.

---

## 1) Mismatch con il codice esistente (API e semantica)

### 1.1. `RepulsionDataManager` oggi non calcola distanze: le riceve già “pronte”

Nell’attuale implementazione (`src/components/repulsion_data_manager.cpp` + header):

- **sorgente dati**: `scene_builder/RobotPointsInfo` su topic (cache con staleness)
- per ogni POI abilitato:
  - legge `pt_info.distance` e `pt_info.distance_vector`
  - costruisce:
    - `ObstacleInfo` (per POI TCP, `config.is_tcp == true`)
    - `LinkPOI` (per gli altri POI)
  - applica correzione “superficie-superficie”:
    - $ d_\text{eff} = \max(0.001,\ d_\text{raw} - r_\text{poi} - r_\text{obj}) $

**Implicazione**: passare alla mappa 3D significa spostare nel controller responsabilità che oggi sono “esterne” (scene_builder). Va definito cosa si vuole preservare:

- **stessa semantica di distanza** (per non cambiare tuning)
- **stessa semantica di `distance_vector`** (per repulsione + marker)

**Risposta**: Non mi interessa retrocompatibilità, voglio la migliore soluzione per gestire e utilizzare la mappa.

### 1.2. Campi e flag citati nella proposta non esistono nelle strutture reali

La proposta cita:

- `ObstacleInfo::is_from_map`
- `ObstacleInfo::closest_point_on_obstacle`

Nel codice attuale (`include/cartesian_velocity_controller/types/pipeline_types.hpp`) questi campi **non esistono**.

**Conseguenze**:

- o si estendono le struct (impatti su debug/marker/feedback e possibili ROS message),
- o si rimane compatibili e si codifica la provenienza altrove (es. `id` o namespace), evitando modifiche “a cascata”.

### 1.3. `MarkerPublisher` assume una specifica semantica di `position` e `distance_vector`

In `src/components/marker_publisher.cpp`:

- per TCP obstacles: disegna una linea `tcp_position -> obs.position`
  - quindi `obs.position` è trattato come **punto “sull’ostacolo”** (o almeno “posizione ostacolo”)
- per Link POI: ricava la posizione ostacolo come:
  - `obs_pos = poi.position_world + poi.distance_vector`
  - quindi `poi.distance_vector` è trattato come **POI -> ostacolo** (in world frame)

**Punto critico**: un’EDT produce naturalmente “direzione verso libero” e “closest point on obstacle” nel frame della mappa (es. `base_link`). Se non converti correttamente in world frame e non rispetti queste convenzioni, i marker diventano fuorvianti e il comportamento repulsivo può cambiare segno/direzione.

**Risposta**: Al momento il world frame e il base_link coincidono, quindi passando al nuovo frame non ci dovrebbero essere problemi. Per quanto riguarda i vecchi marker e la vecchia metodologia si può rimuovere tutto.

---

## 2) Criticità di frame / TF (bug probabile se non si fissa una convenzione)

### 2.1. Situazione attuale

- `LocalPlanner` opera su vettori in **world frame** (o comunque nel “global frame” del controller).
- `RepulsionDataManager` attuale fornisce:
  - `LinkPOI.position_world` e vettori coerenti per repulsione + debug.

### 2.2. Proposta: mappa in `base_link`

La proposta raccomanda una mappa solidale al robot (`base_link` / `base_footprint`).
Questo è sensato per rolling window “gratis”, ma introduce un requisito:

- ogni quantità derivata dalla mappa (distance/gradient/closest point) nasce in **map frame**
- il controller e i marker vogliono informazioni in **world frame**

**Risposta**: Si può usare il base_link del robot, è lo stesso frame al momento

### 2.3. Errore tipico da evitare (mix di frame)

Nella proposta, l’idea “gradient * distance” viene poi sottratta a `position_world` per ricostruire `closest_point_on_obstacle`.
Questo è sbagliato se:

- `gradient` è in map frame
- `position_world` è in world frame

**Regola**: le operazioni di somma/sottrazione tra vettori/posizioni devono essere nello **stesso frame**.

### 2.4. Decisione da chiudere: in che frame pubblichiamo `ObstacleInfo` e `LinkPOI`?

Per minimizzare cambi:

- **Raccomandazione**: mantenere `ObstacleInfo` e `LinkPOI` in **world frame**, come oggi li consuma il sistema (planner + marker).
- Implementare in `Map3DManager` funzioni che:
  - trasformano il punto world -> map frame per la query,
  - trasformano **il vettore** map -> world usando solo la rotazione $R$ (no traslazione),
  - ricostruiscono `distance_vector` coerente (vedi §3).

**Risposta**: Finché non aggiungo la base mobile il world frame e il base_link coincidono e la mappa 3D è fissa rispetto al mondo

---

## 3) Semantica di distanza e vettori (EDT vs geometria attuale)

### 3.1. Oggi: distanza “superficie-superficie” controllata da due raggi

Il controller sottrae due inflazioni:

- `poi_radius` (config)
- `object_characteristic_radius` (dal messaggio)

Questo aiuta a stabilizzare e “dare margine” senza cambiare oggetti.

**Risposta**: Nella mappa io calcolerei un margine di sicurezza rispetto all'ostacolo che sia fisso, e considererei il robot puntiforme, ma definirei un raggio intorno al POI, che può variare da POI a POI, e che va sottratta alla distanza letta sulla mappa.

### 3.2. Con EDT: che distanza è davvero?

Un EDT su occupancy voxel restituisce distanza (approx) tra:

- un punto continuo (interpolato) e
- l’insieme delle celle occupate (definizione “discreta” della superficie)

**Punti critici**:

- la distanza dipende da come voxelizzi (centro voxel vs voxel volume)
- una mesh voxelizzata grossolanamente produce distanze “a scalini”
- il “punto più vicino” non coincide con una superficie geometrica perfetta

### 3.3. POI radius: sottrazione a runtime o inflazione della mappa?

Scelte possibili:

- **A) Sottrazione a runtime (consigliata)**:
  - $ d_\text{eff} = \max(\epsilon,\ d_\text{edt} - r_\text{poi}) $
  - pro: semplice, conserva la logica attuale dei POI
  - contro: non include margine “ostacolo” (vedi sotto)

- **B) Inflazione ostacoli in voxelization**:
  - marcando occupati voxel in un raggio (dilatazione)
  - pro: incorpora margini statici globali
  - contro: costoso e/o complesso, e non gestisce POI diversi per raggio senza rifare la mappa

**Punto aperto**: la proposta parla di `object_radius` ma con EDT non esiste naturalmente. Devi decidere un equivalente:

- “radius globale” (safety margin fisso)
- o per-oggetto (solo se PlanningScene fornisce metadati)
- o nessuno (ma allora cambi tuning e safety)

**Risposta**: Ti ho risposto sopra a questo punto.

---

## 4) Voxelizzazione: copertura reale della PlanningScene

### 4.1. Limite attuale della proposta

La proposta include esempi per:

- sphere
- box (anche ruotato, con logica conservativa)
- cylinder

### 4.2. Mesh e casi frequenti

In MoveIt, i `CollisionObject` possono includere:

- `shape_msgs/Mesh`
- primitive multiple nello stesso oggetto
- pose multiple

**Punto aperto bloccante**: come gestire mesh?

**Risposta**: Gestisci solo le sfere, tralascia il resto

Opzioni:

- **Fallback conservative**: voxelizza AABB della mesh (sicuro ma molto restrittivo)
- **Sampling**: campiona triangoli/punti e marca voxel (precisione migliore, costo variabile)
- **Conversione**: pre-process offline in primitive (non sempre possibile)
- **Non supportare**: ma devi loggare chiaramente e accettare che ostacoli “spariscono”

---

## 5) EDT e gradienti: stabilità numerica, jitter e coerenza con interpolazione

### 5.1. Gradienti vicino a distanza 0

In cella occupata:

- $d = 0$
- gradiente “non definito” / rumoroso

**Implicazioni pratiche**:

- se un POI entra in contatto (o quasi), la direzione di repulsione può “impazzire” a causa di discretizzazione
- si rischia jitter ad alta frequenza (che poi viene filtrato, ma può comunque destabilizzare o generare marker incoerenti)

**Mitigazioni tipiche**:

- **Clamp**: se $d < d_\text{min}$ usa una direzione fallback stabile (es. ultimo gradiente valido, o direzione verso centro mappa / outward)
- **Smoothing**: filtro low-pass sul gradiente per ogni POI (attenzione a introdurre latenza)
- **Derivata della distanza interpolata**: preferire gradiente derivato dalla distanza (interpolata) invece di un `gradient_grid_` discreto, se vuoi coerenza matematica.

**Risposta**: Il clamp mi piace, anche con fallback su ultimo gradiente valido

### 5.2. Coerenza “distanza interpolata” vs “gradiente precomputato”

La proposta prevede:

- query distanza con interpolazione trilineare
- gradienti precomputati (opzionali)

**Criticità**: se la distanza viene interpolata ma il gradiente è un campo discreto (precomputato per voxel), la direzione potrebbe non essere la derivata del valore che stai effettivamente usando → repulsione non “conservativa” e potenziale jitter.

**Raccomandazione**:

- prima versione: implementare `getDistance()` + `getGradient()` derivando il gradiente con differenze finite *sulla stessa `distance_grid_`* (e, se fai interpolazione, valutare gradienti coerenti localmente)
- ottimizzazione successiva: precompute gradiente solo se profilazione mostra che serve, e definire chiaramente come viene calcolato (stessa metrica e stessa risoluzione della distanza).

**Risposta**: qui non saprei cosa è meglio, bisogna fare delle prove

---

## 6) Gestione bordi, “unknown” e failure modes

### 6.1. Il problema reale

Quando il robot (o un POI) esce dal volume mappa:

- la distanza non è definita (fisicamente: “non sappiamo”)
- qualunque scelta influenza il comportamento (sicurezza vs permissività)

**Risposta**: La mappa è più grande dello spazio utile del robot. Non esce dai limiti

### 6.2. Scelte possibili (e loro effetti)

- **Hard wall (conservativa)**: fuori mappa → distanza 0 (repulsione massima)
  - pro: molto safe
  - contro: può bloccare movimenti leciti vicino ai bordi, specie se la mappa è piccola o il TCP si avvicina ai limiti

- **Ignore (ottimistica)**: fuori mappa → distanza `max_distance`
  - pro: non introduce “muri invisibili”
  - contro: pericolosa se i sensori non coprono davvero fuori mappa o se la scena non è completa

- **Gradual margin (ibrida)**: zona cuscinetto → blend che riduce la distanza man mano che ti avvicini al bordo
  - pro: evita discontinuità
  - contro: va definita bene anche la **direzione** (gradiente) quando sei “quasi fuori” o “fuori”

### 6.3. Punto aperto: cosa restituisce `getGradient()` fuori bounds?

Se fuori bounds ritorni “distanza 0”, ma il gradiente è zero o random:

- `LocalPlanner` può non avere direzione repulsiva coerente (repulsione nulla o instabile)

**Raccomandazione**:

- definire un gradiente di bordo deterministico:
  - outward/inward rispetto al box della mappa (normale del bordo più vicino)
  - oppure un vettore “towards center” per rientrare rapidamente

**Risposta**: Va bene il gradiente verso il centro

### 6.4. Failure modes operativi

Oltre ai bordi, la proposta deve coprire esplicitamente:

- **TF unavailable**: cosa succede se non hai TF map<->world? (oggi `RepulsionDataManager` semplicemente non produce dati se stali)
- **Map stale**: come degradi? (aumentare safety margin? ridurre velocità? disabilitare repulsione?)
- **Map build overrun**: se la generazione mappa supera il budget (es. 150–200ms), cosa fai? (skip update, riduci risoluzione, riduci forme supportate)

**Risposta**: Io aggiornerei la mappa il più velocemente possibile e poi controllerei l'effettiva frequenza di aggiornamento

---

## 7) Threading, MoveIt PlanningScene e lock

### 7.1. Dato che la sorgente è PlanningScene (non sensori), il “costo” vero è la lettura + conversione

La proposta prevede `PlanningSceneReader` che legge collision objects dal `PlanningSceneMonitor`.

**Punto aperto**: come si legge in modo thread-safe e consistente?

- `PlanningSceneMonitor` e `PlanningScene` usano lock interni. Se prendi lock lunghi o fai conversioni pesanti mentre hai il lock, puoi:
  - bloccare update scene
  - prendere snapshot inconsistente

**Raccomandazione**:

- prendere uno snapshot dei dati necessari il più rapidamente possibile (sotto lock)
- fare voxelization/EDT fuori dal lock

**Risposta**: Va bene la tua proposta

---

## 8) Performance e budget temporale (10 Hz → 100 Hz)

### 8.1. Il dimensionamento di base è OK, ma il costo dipende dalla voxelizzazione

Con 3×3×2 m a 5 cm: ~144k voxel (buono).

Costi dominanti tipici:

- voxelizzazione di box ruotati (test “point-in-box” su tanti voxel)
- gestione mesh (se implementata)
- EDT + gradienti (EDT O(N) può essere ok, ma attenzione alle allocazioni e ai passaggi colonna/righe)

### 8.2. Double buffering: bene per lock-free read, ma attenzione a copie/allocazioni

La proposta mostra `std::array<VoxelGrid3D,2>`: ok se `VoxelGrid3D` gestisce bene memoria interna.

**Punto aperto**:

- `VoxelGrid3D` deve evitare re-allocazioni ad ogni update (`clear()` deve “fillare” e non ridimensionare)
- l’EDT e l’estrazione colonne (passi Y/Z) devono evitare allocazioni ripetute (riuso buffer temporanei)

### 8.3. Profilazione obbligatoria

Prima di “chiudere” risoluzione/strategie, serve telemetria:

- tempo di `PlanningSceneReader`
- tempo di voxelization (per forma)
- tempo di EDT
- tempo di gradienti
- tempo totale ciclo mappa

Senza questi numeri, la stima “10–20ms” può essere ottimistica (specialmente con forme complesse o tante collision objects).

---

## 9) Staleness: allineare `MapMetadata` con l’attuale `stale_timeout_`

Oggi `RepulsionDataManager` ha `stale_timeout_` e se il topic è stalo ritorna senza dati.

Con la mappa:

- serve qualcosa di analogo (timestamp ultimo swap, age)
- va deciso **cosa fa il controller** quando la mappa è stala:
  - fallback a scene_builder se disponibile?
  - riduzione velocità / aumento min_safe?
  - stop (comportamento safe)

---

## 10) Fusione `BOTH`: “min delle distanze” è solo metà del problema

La proposta suggerisce `BOTH` = min(distanze).

**Criticità**:

- il “min” ha senso solo se le distanze sono comparabili (stessa semantica: superficie-superficie, stessi margini)
- la direzione (vettore) può essere diversa:
  - scene_builder: vettore verso centro oggetto (poi corretto con raggio)
  - EDT: vettore verso closest point su superficie voxelizzata

**Rischio**: switching tra sorgenti produce cambi bruschi di direzione.

**Raccomandazione**:

- se fai fusione, fondi anche la direzione in modo coerente (es. scegli la sorgente che ha la distanza min e usa il suo vettore, oppure blend continuo con hysteresis)
- introdurre hysteresis sulla sorgente scelta per evitare “flicker”.

---

## 11) “Spec” consigliata per rendere l’integrazione non ambigua (frame + segni)

Qui propongo una convenzione compatibile col comportamento attuale di `LocalPlanner` + `MarkerPublisher`, minimizzando refactor.

### 11.1. Frame

- **Tutte le struct che escono da `RepulsionDataManager` devono essere in world frame**:
  - `ObstacleInfo.position` (punto ostacolo) in world
  - `ObstacleInfo.distance_vector` in world
  - `LinkPOI.position_world` in world
  - `LinkPOI.distance_vector` in world
  - `LinkPOI.repulsive_direction` in world

La mappa può vivere in `base_link`, ma il suo output deve essere convertito prima di entrare in queste struct.

### 11.2. Segni / semantica vettori

Per compatibilità con l’attuale visualizzazione e logica:

- **Per `LinkPOI`**:
  - `distance_vector` = vettore **POI → closest point on obstacle** (world)
  - quindi `closest_point_world = poi.position_world + poi.distance_vector`
  - e `repulsive_direction = -distance_vector.normalized()` (away from obstacle)

- **Per `ObstacleInfo` (TCP)**:
  - `position` = **closest point on obstacle** (world) (o “posizione ostacolo” usata per marker)
  - `distance_vector` = vettore **obstacle → TCP** (world) con norma = `distance` effettiva
  - così `ObstacleInfo.getRepulsiveDirection()` restituisce una direzione già “away from obstacle” (coerente con `LocalPlanner`)

### 11.3. Distanze (effettive)

Decidere una regola unica:

- `distance` deve essere **superficie-superficie** coerente col resto del controller
- con EDT:
  - `d_edt` = distanza POI → superficie voxel (world via trasformazioni, ma numericamente nasce in map frame)
  - `distance = max(eps, d_edt - poi_radius - obstacle_margin)` dove `obstacle_margin` è una scelta progettuale (0 o safety margin globale)

---

## 12) Checklist di decisioni da chiudere (prima di scrivere codice)

- **Frame**: output del Map3D verso il controller in world frame? (raccomandato: sì)
- **Semantica dei vettori**: definizione esatta di `distance_vector` per `ObstacleInfo` e `LinkPOI` (vedi §11)
- **Semantica distanze**: come sostituire `object_characteristic_radius` quando la sorgente è EDT?
- **Supporto forme**: mesh supportate? fallback? policy di logging e safety
- **Bordi/unknown**: cosa restituisce `getDistance()` e soprattutto `getGradient()` fuori bounds?
- **Staleness/fallback**: cosa fa il controller se mappa stala o TF manca?
- **Fusion BOTH**: come evitare flicker e incoerenza di direzione?
- **Budget**: definire soglie e telemetria (timing per step) + degrade strategy

---

## 13) Test plan minimo (per validare senza “sorprese”)

- **Unit test matematici (offline)**:
  - `worldToVoxel/voxelToWorld` + bounds
  - distanza EDT su configurazioni note (singolo voxel occupato, box axis-aligned, ecc.)
  - gradiente coerente (direzione verso libero) e stabile vicino a contatto (clamp)

- **Test integrato in RViz**:
  - 1 ostacolo statico davanti al robot: marker direzioni coerenti e distanza monotona
  - 1 ostacolo che attraversa il bordo mappa: comportamento boundary (hard/gradual) previsto e stabile
  - “TF drop” simulato: degradazione safe (nessun comportamento erratico)

- **Test regressione**:
  - confrontare output repulsivo (modulo/direzione) tra `scene_builder` e `map3d` su scene semplici, per verificare che tuning non venga stravolto.


