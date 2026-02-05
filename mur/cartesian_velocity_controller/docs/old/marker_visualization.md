# Documentazione Visualization Markers

Questo documento descrive tutti i marker pubblicati da `MarkerPublisher` in `marker_publisher.cpp`, i loro namespace, colori e punti di applicazione.

## Overview Topic

| Topic | Descrizione | Frequenza |
|-------|-------------|-----------|
| `velocity_markers` | Vettori delle componenti di velocità (attrattive/repulsive) | 10 Hz (approx) |
| `command_markers` | Vettori del comando di velocità finale inviato al robot | 10 Hz (approx) |
| `target_markers` | Sfere che indicano i target (raw, filtrato, attivo) | 10 Hz (approx) |
| `repulsion_markers` | Dettagli sulla repulsione (POI, sfere di influenza, vettori locali) | 10 Hz (approx) |

---

## Dettaglio Markers

### 1. Velocity Markers (Topic: `velocity_markers`)
Visualizzano le forze/velocità componenti calcolate dal planner locale **applicate al TCP**.

| Namespace | Tipo | Colore | Punto di Applicazione | Descrizione |
|-----------|------|--------|-----------------------|-------------|
| `v_goal_linear` | Arrow | Ciano (Cyan) | **TCP** | Componente lineare attrattiva verso il target. |
| `v_goal_angular` | Arrow | Magenta | **TCP** | Componente angolare attrattiva per l'orientamento. |
| `v_obs_linear` | Arrow | Rosso/Arancio | **TCP** | Velocità repulsiva generata dagli ostacoli vicini al **TCP**. |
| `v_link_linear` | Arrow | Giallo | **TCP** | Velocità repulsiva generata dai **Link** (proiettata al TCP). <br>⚠️ **Nota:** Attualmente nel codice principale viene passato come `Vector3d::Zero()`, quindi non viene visualizzato. |

### 2. Command Markers (Topic: `command_markers`)
Visualizzano il comando di velocità cartesiana finale risultante (somma di attrattive + repulsive + scaling).

| Namespace | Tipo | Colore | Punto di Applicazione | Descrizione |
|-----------|------|--------|-----------------------|-------------|
| `cartesian_cmd` (ID 0) | Arrow | Bianco | **TCP** | Vettore velocità lineare finale. |
| `cartesian_cmd` (ID 1) | Text | Bianco | **TCP** (fine freccia) | Magnitudine velocità lineare (`|v|=...`). |
| `cartesian_cmd` (ID 2) | Arrow | Viola Chiaro | **TCP** | Vettore velocità angolare finale. |
| `cartesian_cmd` (ID 3) | Text | Viola Chiaro | **TCP** (fine freccia) | Magnitudine velocità angolare (`|w|=...`). |

### 3. Target Markers (Topic: `target_markers`)
Visualizzano i punti target nello spazio.

| Namespace | Tipo | Colore | Punto di Applicazione | Descrizione |
|-----------|------|--------|-----------------------|-------------|
| `active_waypoint` | Sphere | Verde | **Posizione Waypoint** | Il waypoint corrente verso cui il robot sta navigando. |
| `target_raw` | Sphere | Arancione | **Target Input** | Il target grezzo ricevuto dall'interfaccia/topic esterno. |
| `target_filtered` | Sphere | Blu | **Target Filtrato** | Il target dopo il filtraggio/interpolazione (se attivo). |

### 4. Repulsion Markers (Topic: `repulsion_markers`)
Visualizzano i dettagli geometrici della repulsione su tutto il corpo del robot.

| Namespace | Tipo | Colore | Punto di Applicazione | Descrizione |
|-----------|------|--------|-----------------------|-------------|
| `repulsion_tcp` | Sphere | Arancio Scuro | **TCP Surface** | Punto sulla collision geometry del TCP più vicino all'ostacolo. Raggio sfera = raggio POI TCP. |
| `repulsion_link` | Sphere | Oro/Giallo | **Link POI** | Posizione del Point of Interest (POI) sul link (es. gomito) che è in pericolo. |
| `poi_repulsive_vel` | Arrow | Rosso Chiaro | **Link POI** | Vettore velocità repulsiva generato dallo specifico POI del link (prima della proiezione Jacobiana). |
| `poi_vel_text` | Text | Bianco | **Link POI** | Magnitudine della repulsione su quel POI. |

---

## Analisi Anomalie / Mancanze

### 1. `v_link_linear` è sempre Zero
Nel file `cartesian_velocity_controller.cpp`, la funzione `publishVelocityMarkers` viene chiamata passando `Eigen::Vector3d::Zero()` come ultimo argomento (`v_link_linear`).
```cpp
marker_publisher_->publishVelocityMarkers(..., Eigen::Vector3d::Zero()); // v_link
```
Di conseguenza, la freccia gialla che dovrebbe rappresentare l'effetto repulsivo dei link sul TCP non appare mai.

### 2. Namespace "puliti" ma non usati
Nel metodo `clear()`, vengono cancellati i seguenti namespace che però **non vengono mai pubblicati** nel metodo `publishRepulsionMarkers`:
*   `repulsion_tcp_dir`
*   `repulsion_link_dir`

Questo suggerisce che in una versione precedente (o futura) c'era l'intenzione di visualizzare il **vettore direzione distanza** (la linea che collega l'ostacolo al robot), che è un debug visivo molto utile per capire "cosa" il robot sta evitando.

### 3. Visualizzazione Ostacoli
Non vengono pubblicati marker per la posizione degli ostacoli stessi (es. il punto dell'ostacolo più vicino), ma solo il punto corrispondente sul robot (`repulsion_tcp` o `repulsion_link`).

