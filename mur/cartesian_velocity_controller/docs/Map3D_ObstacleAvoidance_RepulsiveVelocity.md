# Map3D e “velocità repulsiva” in `cartesian_velocity_controller`

Questo documento spiega **se** il robot sta usando la nuova **mappa 3D** (distance field + gradiente) per l’evitamento ostacoli e **come** viene calcolata l’**intensità** (modulo) della velocità repulsiva a partire da:

- una **mappa di distanza** (EDT: Euclidean Distance Transform)
- una **mappa di gradiente** (direzione “verso spazio libero”)

> Nota terminologica: nel codice si parla di **velocità repulsiva** (non di “forza”). È equivalente a un *campo di velocità* tipo potential field.

---

## 1) Il robot usa già la Map3D per evitare ostacoli?

### Condizioni necessarie nel codice
Nel loop di controllo, i dati di repulsione vengono richiesti così:

- in `CartesianVelocityController::executePipeline()`:
  - se `repulsive_enabled_ == true` **e** `repulsion_manager_` esiste
  - allora viene chiamato `repulsion_manager_->getRepulsionData(obstacles, link_pois, ...)`
  - e quei vettori vengono passati a `LocalPlanner::compute(..., obstacles, link_pois, ...)`

Quindi: **la Map3D influisce sul moto solo se la repulsione è abilitata e il manager riesce a produrre almeno un `ObstacleInfo` (TCP) e/o un `LinkPOI`.**

### Perché spesso “sembra abilitata” ma non produce repulsione
Nella configurazione tipica (`config/controller_params.yaml`) trovi:

- `local_planner/repulsive_enabled: true`  → il controller prova a usare la repulsione
- ma in `repulsion/points/*/enabled` sono **false** per default → **nessun POI attivo**

Se **tutti** i POI sono disabilitati, `RepulsionDataManager` filtra tutto e ritorna:

- `obstacles_out = []`
- `link_pois_out = []`

Con liste vuote, il `LocalPlanner` calcola repulsione **zero** (quindi la mappa non “spinge” il robot).

> Nota pratica: all’avvio, anche la GUI/`dynamic_reconfigure` viene inizializzata leggendo lo stato corrente dei POI dal `RepulsionDataManager`.  
> Quindi se il YAML ha `repulsion/points/*/enabled: false`, anche i toggle `poi_*_enabled` partiranno “spenti” finché non li abiliti esplicitamente.

### Altra condizione: “stale timeout”
Anche con POI abilitati, la repulsione può essere disattivata *di fatto* se la mappa è considerata “vecchia”:

- parametro `repulsion/stale_timeout` (default ~0.3s)
- `RepulsionDataManager::hasValidData()` verifica che `Map3DManager` abbia `stamp` recente

Se la Map3D non si aggiorna (PlanningScene non disponibile, TF non ok, ecc.), la repulsione viene ignorata.

---

## 2) Che cosa sono “mappa distanza” e “mappa gradiente” nella Map3D

La `Map3DManager` costruisce una griglia voxel 3D:

1. legge ostacoli dalla **MoveIt PlanningScene** (in questa implementazione: **sfere**)
2. voxelizza (con una **inflazione globale** `map3d/obstacle_margin`)
3. calcola una **EDT**: per ogni voxel “libero” memorizza la distanza (in metri) dalla superficie ostacolo più vicina
4. calcola/fornisce il **gradiente** $∇d$: un vettore che punta verso distanze maggiori, quindi **verso spazio libero**

La query usata dalla repulsione è `Map3DManager::queryWorld(p_world, world_frame)` e restituisce:

- `distance = d` : distanza (m) alla **superficie** dell’ostacolo (già “inflated” dalla `obstacle_margin`)
- `gradient = g` : (idealmente) gradiente **normalizzato**, orientato verso spazio libero

---

## 3) Come la Map3D entra nel controller: POI → query → `ObstacleInfo` / `LinkPOI`

Il ponte tra mappa e planner è `RepulsionDataManager`:

### 3.1 POI (Points Of Interest)
I POI sono definiti come (link + offset) via parametri:

- `repulsion/robot_points_of_interest/<name>/link`
- `repulsion/robot_points_of_interest/<name>/offset: [x,y,z]`

Ogni POI ha anche una config runtime:

- `repulsion/points/<name>/enabled`
- `repulsion/points/<name>/radius` (inflazione del “corpo robot” in quel punto)
- `repulsion/points/<name>/weight` (peso solo per Link POIs)
- `repulsion/points/<name>/is_tcp` (se true genera repulsione “TCP-style”)

### 3.2 Query e vettori geometrici
Per ogni POI abilitato:

1. si calcola la posizione $p$ in world frame con MoveIt `RobotState`:
   - $p = T_{world\_link} \cdot offset$
2. si fa la query Map3D in quel punto:
   - $d_{edt} = \text{Map3D}(p)$
   - $g = ∇d(p)$ (direzione verso spazio libero)
3. si calcola una distanza “effettiva” che considera il raggio del POI:

$$
d_{eff} = \max(\varepsilon,\; d_{edt} - r_{poi})
$$

dove $\varepsilon$ è `map3d/min_distance_eps`.

4. si stima un “closest point” sulla superficie ostacolo (approssimato col gradiente):

$$
p_{closest} = p - g \cdot d_{edt}
$$

### 3.3 Stabilizzazione del gradiente vicino al contatto
Vicino all’ostacolo, il gradiente può essere rumoroso o nullo (discretizzazione / saturazione). Per questo:

- se $||g||$ è troppo piccolo (`map3d/gradient_eps`) **oppure** se $d_{edt}$ è sotto `map3d/gradient_clamp_distance`,
  allora `RepulsionDataManager` usa l’**ultimo gradiente valido** memorizzato per quel POI come fallback.

Questa parte influenza **solo la direzione** della repulsione, non la sua intensità.

---

## 4) Come viene calcolata l’intensità (modulo) della velocità repulsiva

Qui è il punto chiave della tua domanda.

### 4.1 Direzione vs intensità
Nel design attuale:

- **Direzione**: viene dal gradiente (o da `distance_vector` nel caso TCP), cioè “via dall’ostacolo”.
- **Intensità**: **non** usa il modulo del gradiente.
  - viene calcolata come funzione **solo della distanza** $d$ rispetto a due soglie:
    - `influence_distance` (oltre: repulsione 0)
    - `min_safe_distance` (sotto: repulsione massima)

### 4.2 TCP repulsion (`ObstacleInfo`) — formula implementata
In `LocalPlanner::computeRepulsiveVelocityFromObstacle()`:

- input: `ObstacleInfo.distance = d`
- `d_infl = influence_distance`
- `d_min  = min_safe_distance`
- `v_max  = max_linear_velocity` del LocalPlanner

#### 4.2.1 Caso “fuori influenza”
Se $d \ge d_{infl}$ allora:

$$
||v_{rep}|| = 0
$$

#### 4.2.2 Caso “troppo vicino” (zona di sicurezza)
Se $d \le d_{min}$ allora:

$$
||v_{rep}|| = v_{max}
$$

#### 4.2.3 Caso intermedio (tra $d_{min}$ e $d_{infl}$)
Si definisce:

$$
r = \frac{d_{infl} - d}{d_{infl} - d_{min}}
\qquad\text{con}\quad r \in (0,1)
$$

Poi il modulo è:

- **LINEAR**:

$$
||v_{rep}|| = v_{max}\, r
$$

- **QUADRATIC** (default nel `LocalPlanner`):

$$
||v_{rep}|| = v_{max}\, r^2
$$

> Nota: nel codice `RepulsiveVelocityMode::QUADRATIC` è implementato come **parabola** $r^2$ (0 a $d_{infl}$, massimo a $d_{min}$).  
> Il commento “$k/d^2$” presente nell’enum non riflette questa implementazione.

#### 4.2.4 Direzione e guadagno
La direzione viene da `ObstacleInfo.getRepulsiveDirection()`, cioè:

$$
\hat{n} = \frac{distance\_vector}{||distance\_vector||}
$$

Quindi (per ogni ostacolo):

$$
v_{rep,i} = \hat{n}_i \, ||v_{rep,i}||
$$

e il totale TCP è la somma:

$$
v_{rep,tcp} = k_{rep,tcp}\,\sum_i v_{rep,i}
$$

dove `k_rep,tcp` è `local_planner/repulsive_obstacle_gain`.

**Importante:** la repulsione TCP viene poi sommata alla velocità attrattiva e l’intero vettore risultante viene limitato in modulo da `local_planner/max_linear_velocity` (clamp finale sul “combined”).

### 4.3 Come `ObstacleInfo` viene costruito con Map3D (cosa è “d” nel caso Map3D)
Nel caso Map3D, `RepulsionDataManager` genera un solo `ObstacleInfo` con id `"map3d"` per il POI TCP (se abilitato).

- `d_{edt}`: distanza Map3D alla **superficie ostacolo** (già con `map3d/obstacle_margin`)
- `d = d_eff = max(min_distance_eps, d_edt - r_poi)` è la distanza “superficie–superficie” (approssimata) che entra nella formula sopra
- la direzione è coerente con il gradiente perché `distance_vector` viene costruito come:
  - $p_{closest} = p - g\, d_{edt}$
  - `distance_vector = (p - p_closest)` normalizzato e scalato a $d_{eff}$

In pratica: **il gradiente fornisce la normale/verso di fuga**, ma **il modulo dipende solo da $d_{eff}$**.

---

## 5) Repulsione sui link (`LinkPOI`): intensità e proiezione in spazio giunti

Quando un POI non è TCP (`is_tcp: false`), `RepulsionDataManager` genera un `LinkPOI` con:

- `distance_to_closest_obstacle = d_eff`
- `repulsive_direction = g` (gradiente verso spazio libero)
- `weight` (da config POI)

### 5.1 Intensità per LinkPOI (stessa legge del TCP)
In `LocalPlanner::computeRepulsiveLinkJointVelocity()` si usa la stessa identica legge del §4.2, ma con:

- $d =$ `poi.distance_to_closest_obstacle`
- $d_{infl}=$ `local_planner/influence_distance`
- $d_{min}=$ `local_planner/min_safe_distance`
- $v_{max}=$ `local_planner/max_linear_velocity`

e in più si applica il peso del POI:

$$
||v_{rep,poi}|| \leftarrow ||v_{rep,poi}|| \cdot w_{poi}
$$

Infine la velocità cartesiana del POI è:

$$
v_{rep,poi} = \hat{g}\,||v_{rep,poi}||
$$

dove $\hat{g}$ è la direzione (gradiente) verso spazio libero.

### 5.2 Dalla velocità cartesiana del POI alla velocità di giunto
Per ogni POI (raggruppato per `point_name`) si calcola il Jacobiano del link (offset incluso) e si usa **solo la parte lineare**:

- $J_{lin} \in \mathbb{R}^{3 \times N}$

Si calcola una pseudo-inversa “3xN” senza damping:

$$
J_{lin}^+ = J_{lin}^T\,(J_{lin}\,J_{lin}^T)^{+}
$$

e quindi:

$$
\dot{q}_{rep,poi} = J_{lin}^+\, v_{rep,poi}
$$

Le componenti dei vari POI vengono sommate:

$$
\dot{q}_{rep,links} = \sum_{poi}\dot{q}_{rep,poi}
$$

e infine scalate con:

$$
\dot{q}_{rep,links} \leftarrow k_{rep,links}\,\dot{q}_{rep,links}
$$

dove `k_rep,links` è `local_planner/repulsive_link_gain`.

### 5.3 Nota importante su “come entra nel comando finale”
Nel codice corrente:

- `LocalPlanner` **converte** $\dot{q}_{rep,links}$ in una velocità cartesiana equivalente al TCP tramite il Jacobiano del TCP, e la **somma** alla componente lineare (così influenza l’integrazione del “virtual target”).
- **Non** viene sommata direttamente $\dot{q}_{rep,links}$ al comando di giunto finale nel controller (è usata principalmente per debug/visualizzazione e per costruire $v_{rep,links}$ al TCP).

Quindi la repulsione link agisce **indirettamente** (modificando il target integrato), non come termine additivo “hard” sui giunti nel passo finale.

---

## 6) Riassunto “1 riga” alla tua domanda sull’intensità

La **mappa di gradiente** serve a dare la **direzione** (via dall’ostacolo).  
La **mappa di distanza** fornisce $d$.  
L’**intensità** della repulsione è una funzione **solo di $d$**:

- $0$ se $d \ge d_{infl}$
- $v_{max}$ se $d \le d_{min}$
- altrimenti cresce con una rampa **lineare** $r$ o **quadratica** $r^2$ tra $d_{infl}$ e $d_{min}$,
  poi moltiplicata per i guadagni (`repulsive_obstacle_gain` / `repulsive_link_gain`) e, per i link, per `weight`.

---

## 7) Parametri che governano “quanto spinge”

### Local planner (modulo base)
- `local_planner/max_linear_velocity` → $v_{max}$ (scala massima della repulsione, prima dei guadagni)
- `local_planner/influence_distance` → $d_{infl}$
- `local_planner/min_safe_distance` → $d_{min}$
- `local_planner/repulsive_obstacle_gain` → $k_{rep,tcp}$
- `local_planner/repulsive_link_gain` → $k_{rep,links}$

### POI (solo link e distanza effettiva)
- `repulsion/points/<name>/radius` → sottratto a $d_{edt}$ (distanza effettiva più “conservativa”)
- `repulsion/points/<name>/weight` → moltiplicatore del modulo (solo LinkPOI)
- `repulsion/points/<name>/enabled` → se falso, quel POI non esiste per la repulsione

### Map3D (geometria degli ostacoli)
- `map3d/obstacle_margin` → inflazione globale degli ostacoli nella mappa (riduce le distanze $d_{edt}$)
- `map3d/gradient_*` → stabilità direzione vicino al contatto (non cambia il modulo)


