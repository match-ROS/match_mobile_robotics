# Piano di Integrazione: Velocità Repulsiva con POI Dinamici

## Indice
1. [Obiettivo](#obiettivo)
2. [Stato Attuale del Codice](#stato-attuale-del-codice)
3. [Architettura Proposta](#architettura-proposta)
4. [Dettagli Implementativi](#dettagli-implementativi)
5. [Step di Implementazione](#step-di-implementazione)
6. [Punti Aperti e Dubbi](#punti-aperti-e-dubbi)
7. [Test Plan](#test-plan)

---

## Obiettivo

Implementare la velocità repulsiva nel `cartesian_velocity_controller` utilizzando **POI (Points of Interest)** definiti sul robot, con:

- **Raggio di influenza dinamico** per ogni POI
- **Peso dinamico** per ogni POI
- Modificabilità runtime sia da **logica interna C++** che da **rqt_reconfigure**
- Sorgente dati: topic `/robot_points_info` pubblicato da `scene_builder`
- **Nessuna rimozione del codice legacy** (topic `/distance_info`), ma non verrà usato attivamente

---

## Stato Attuale del Codice

### scene_builder

Il `RobotPointTracker` già:
- Traccia POI definiti in `scene_builder_params.yaml`
- Calcola posizione, velocità e distanze al centro degli oggetti
- Pubblica su `/robot_points_info` (`scene_builder/RobotPointsInfo`)

**Messaggio `RobotPointDistanceInfo`:**
```
string point_name                       # es. "elbow", "wrist"
string link_name                        # Link del robot
geometry_msgs/Point position            # Posizione nel world frame
geometry_msgs/Vector3 linear_velocity   # Velocità lineare
string object_id                        # ID del collision object
geometry_msgs/Vector3 distance_vector   # Vettore dal POI al centro dell'oggetto
float64 distance                        # Distanza scalare
geometry_msgs/Vector3 object_velocity   # Velocità dell'oggetto
```

**Importante:** `distance` è la distanza dal POI al **centro** dell'oggetto (non alla superficie). Questa approssimazione è accettata per ora.

### cartesian_velocity_controller

Il `LocalPlanner` già supporta:
- `std::vector<ObstacleInfo> obstacles` → repulsione TCP
- `std::vector<LinkPOI> link_pois` → repulsione link (via Jacobiano parziale)

**Problema attuale:** in `executePipeline()` (riga 779-780):
```cpp
std::vector<ObstacleInfo> obstacles;
std::vector<LinkPOI> link_pois;
```
I vettori sono **sempre vuoti** → la repulsione non entra mai in gioco.

### Strutture Dati Esistenti

**`ObstacleInfo`** (per repulsione TCP):
```cpp
struct ObstacleInfo {
  std::string id;
  Eigen::Vector3d position;      // Closest point on obstacle
  double distance;
  Eigen::Vector3d distance_vector;  // obstacle → TCP
  double influence_distance{1.0};
  double min_safe_distance{0.05};
};
```

**`LinkPOI`** (per repulsione link):
```cpp
struct LinkPOI {
  std::string link_name;
  Eigen::Vector3d position_world;
  Eigen::Vector3d position_link;     // Offset in link frame (per Jacobiano)
  double distance_to_closest_obstacle;
  Eigen::Vector3d repulsive_direction;
  std::string closest_obstacle_id;
  double weight{1.0};
};
```

---

## Architettura Proposta

### Idea Centrale

Creare un **componente "RepulsionManager"** (o estendere il controller) che:

1. **Sottoscrive** `/robot_points_info`
2. **Mantiene una configurazione** per ogni POI (peso, raggio, enabled)
3. **Converte** i messaggi in `LinkPOI` e/o `ObstacleInfo`
4. **Espone** i parametri a dynamic_reconfigure

### Schema a Blocchi

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        cartesian_velocity_controller                     │
│                                                                          │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                      RepulsionDataManager                         │   │
│  │                                                                   │   │
│  │  ┌─────────────────┐    ┌─────────────────────────────────────┐  │   │
│  │  │   Subscriber    │    │      POI Config Map                 │  │   │
│  │  │ /robot_points_  │───▶│  "elbow"  → {weight, radius, en}    │  │   │
│  │  │     info        │    │  "wrist"  → {weight, radius, en}    │  │   │
│  │  └─────────────────┘    │  "tcp"    → {weight, radius, en}    │  │   │
│  │                         │  ...                                 │  │   │
│  │                         └─────────────────────────────────────┘  │   │
│  │                                      │                            │   │
│  │                                      ▼                            │   │
│  │                         ┌─────────────────────────────────────┐  │   │
│  │                         │  Conversion Logic                   │  │   │
│  │                         │  msg → LinkPOI / ObstacleInfo       │  │   │
│  │                         └─────────────────────────────────────┘  │   │
│  │                                      │                            │   │
│  └──────────────────────────────────────┼────────────────────────────┘   │
│                                         │                                │
│                                         ▼                                │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                        executePipeline()                          │   │
│  │                                                                   │   │
│  │   obstacles ◀──── (TCP POI)                                      │   │
│  │   link_pois ◀──── (Link POIs: elbow, wrist, ...)                 │   │
│  │                                                                   │   │
│  │   local_planner_->compute(..., obstacles, link_pois, ...)        │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                    dynamic_reconfigure                            │   │
│  │   - poi_tcp_weight, poi_tcp_radius, poi_tcp_enabled              │   │
│  │   - poi_elbow_weight, poi_elbow_radius, poi_elbow_enabled        │   │
│  │   - poi_wrist_weight, poi_wrist_radius, poi_wrist_enabled        │   │
│  │   - poi_forearm_weight, poi_forearm_radius, poi_forearm_enabled  │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

### Configurazione POI

Ogni POI avrà questi parametri dinamici:

| Parametro | Tipo | Descrizione | Range |
|-----------|------|-------------|-------|
| `weight` | double | Peso della repulsione (0 = disabilitato) | 0.0 - 2.0 |
| `radius` | double | Raggio di influenza del POI (inflazione) | 0.0 - 0.5m |
| `enabled` | bool | Abilita/disabilita questo POI | true/false |

**Nota:** `radius` rappresenta la "sfera virtuale" attorno al POI. La distanza effettiva per il calcolo della repulsione sarà:
```
d_effective = d_raw - radius
```

---

## Dettagli Implementativi

### 1. Struttura `RepulsivePointConfig`

Nuova struttura per la configurazione runtime di ogni POI:

```cpp
// In types/config_types.hpp o nuovo file
struct RepulsivePointConfig {
  std::string name;           // "tcp", "elbow", "wrist", "forearm"
  double weight{1.0};         // Peso della repulsione
  double radius{0.05};        // Raggio di influenza (inflazione)
  bool enabled{true};         // POI attivo
  bool is_tcp{false};         // true = genera ObstacleInfo, false = genera LinkPOI
};
```

### 2. Classe `RepulsionDataManager`

Propongo una classe dedicata (non componente separato, ma integrata nel controller):

```cpp
// In include/cartesian_velocity_controller/components/repulsion_data_manager.hpp
class RepulsionDataManager {
public:
  RepulsionDataManager(ros::NodeHandle& nh);
  
  // === Configurazione POI ===
  void setPointConfig(const std::string& name, const RepulsivePointConfig& config);
  RepulsivePointConfig getPointConfig(const std::string& name) const;
  std::vector<std::string> getPointNames() const;
  
  // === Update runtime (chiamato da logica interna C++) ===
  void setPointWeight(const std::string& name, double weight);
  void setPointRadius(const std::string& name, double radius);
  void setPointEnabled(const std::string& name, bool enabled);
  
  // === Conversione dati ===
  // Chiamato da executePipeline() per ottenere i vettori pronti per LocalPlanner
  void getRepulsionData(
    std::vector<ObstacleInfo>& obstacles_out,
    std::vector<LinkPOI>& link_pois_out,
    const Eigen::Isometry3d& current_tcp_pose,
    const std::string& global_frame);

  // === Validità dati ===
  bool hasValidData() const;
  ros::Time getLastDataTimestamp() const;

private:
  void robotPointsCallback(const scene_builder::RobotPointsInfo::ConstPtr& msg);
  
  // Trasforma un punto/vettore dal frame del messaggio al global_frame
  bool transformToGlobalFrame(
    const std::string& source_frame,
    const Eigen::Vector3d& point_in,
    Eigen::Vector3d& point_out) const;

  ros::Subscriber robot_points_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  // Cache ultimo messaggio
  scene_builder::RobotPointsInfo last_msg_;
  ros::Time last_msg_stamp_;
  mutable std::mutex msg_mutex_;
  
  // Configurazione POI
  std::map<std::string, RepulsivePointConfig> point_configs_;
  mutable std::mutex config_mutex_;
  
  // Parametri
  double stale_timeout_{0.3};  // Timeout dati (secondi)
  std::string tcp_point_name_{"tcp"};  // Nome del POI che funge da TCP
};
```

### 3. Logica di Conversione

La conversione `RobotPointsInfo` → `LinkPOI` / `ObstacleInfo`:

```cpp
void RepulsionDataManager::getRepulsionData(
    std::vector<ObstacleInfo>& obstacles_out,
    std::vector<LinkPOI>& link_pois_out,
    const Eigen::Isometry3d& current_tcp_pose,
    const std::string& global_frame)
{
  obstacles_out.clear();
  link_pois_out.clear();
  
  std::lock_guard<std::mutex> lock(msg_mutex_);
  
  // Check stale
  if ((ros::Time::now() - last_msg_stamp_).toSec() > stale_timeout_) {
    return;  // Dati troppo vecchi
  }
  
  // Raggruppa per point_name → trova l'oggetto più vicino per ogni POI
  std::map<std::string, scene_builder::RobotPointDistanceInfo> closest_per_poi;
  
  for (const auto& pt_info : last_msg_.points) {
    auto config_it = point_configs_.find(pt_info.point_name);
    if (config_it == point_configs_.end() || !config_it->second.enabled) {
      continue;  // POI non configurato o disabilitato
    }
    
    // Trova il più vicino
    auto it = closest_per_poi.find(pt_info.point_name);
    if (it == closest_per_poi.end() || pt_info.distance < it->second.distance) {
      closest_per_poi[pt_info.point_name] = pt_info;
    }
  }
  
  // Converti in ObstacleInfo o LinkPOI
  for (const auto& [poi_name, pt_info] : closest_per_poi) {
    const auto& config = point_configs_.at(poi_name);
    
    // Trasforma posizione e vettore nel global_frame (se necessario)
    Eigen::Vector3d position(pt_info.position.x, pt_info.position.y, pt_info.position.z);
    Eigen::Vector3d dist_vec(pt_info.distance_vector.x, pt_info.distance_vector.y, pt_info.distance_vector.z);
    
    // TODO: trasformazione TF se msg.header.frame_id != global_frame
    
    // Calcola distanza effettiva (con inflazione raggio)
    double d_effective = std::max(0.001, pt_info.distance - config.radius);
    
    if (config.is_tcp) {
      // === Genera ObstacleInfo per TCP ===
      ObstacleInfo obs;
      obs.id = pt_info.object_id;
      obs.position = position + dist_vec;  // Centro oggetto
      obs.distance = d_effective;
      obs.distance_vector = -dist_vec.normalized() * d_effective;  // oggetto → POI
      obstacles_out.push_back(obs);
    } else {
      // === Genera LinkPOI ===
      LinkPOI poi;
      poi.link_name = pt_info.link_name;
      poi.position_world = position;
      poi.distance_to_closest_obstacle = d_effective;
      poi.repulsive_direction = -dist_vec.normalized();  // Away from object
      poi.closest_obstacle_id = pt_info.object_id;
      poi.weight = config.weight;
      
      // position_link: offset nel frame del link (da configurazione scene_builder)
      // Per ora lo mettiamo a zero - verrà calcolato internamente dal LocalPlanner
      poi.position_link = Eigen::Vector3d::Zero();
      
      link_pois_out.push_back(poi);
    }
  }
}
```

### 4. Dynamic Reconfigure

Estendere `ControllerTuning.cfg` con un gruppo per i POI:

```python
# =========================
# Repulsive POI Configuration
# =========================
poi_grp = gen.add_group("repulsive_pois")

# TCP POI (speciale - genera ObstacleInfo)
poi_grp.add("poi_tcp_enabled", bool_t, 0, "Enable TCP repulsion point", False)
poi_grp.add("poi_tcp_weight", double_t, 0, "TCP repulsion weight", 1.0, 0.0, 2.0)
poi_grp.add("poi_tcp_radius", double_t, 0, "TCP influence radius (m)", 0.05, 0.0, 0.5)

# Elbow POI
poi_grp.add("poi_elbow_enabled", bool_t, 0, "Enable elbow repulsion point", False)
poi_grp.add("poi_elbow_weight", double_t, 0, "Elbow repulsion weight", 1.0, 0.0, 2.0)
poi_grp.add("poi_elbow_radius", double_t, 0, "Elbow influence radius (m)", 0.05, 0.0, 0.5)

# Wrist POI
poi_grp.add("poi_wrist_enabled", bool_t, 0, "Enable wrist repulsion point", False)
poi_grp.add("poi_wrist_weight", double_t, 0, "Wrist repulsion weight", 1.0, 0.0, 2.0)
poi_grp.add("poi_wrist_radius", double_t, 0, "Wrist influence radius (m)", 0.05, 0.0, 0.5)

# Forearm POI
poi_grp.add("poi_forearm_enabled", bool_t, 0, "Enable forearm repulsion point", False)
poi_grp.add("poi_forearm_weight", double_t, 0, "Forearm repulsion weight", 1.0, 0.0, 2.0)
poi_grp.add("poi_forearm_radius", double_t, 0, "Forearm influence radius (m)", 0.05, 0.0, 0.5)
```

### 5. Integrazione in executePipeline()

```cpp
void CartesianVelocityController::executePipeline(double dt)
{
  // ... codice esistente fino a Level B ...

  // ========== Level B: Local Planner ==========
  std::vector<ObstacleInfo> obstacles;
  std::vector<LinkPOI> link_pois;
  
  // Popola i vettori dal RepulsionDataManager
  if (repulsive_enabled_ && repulsion_manager_) {
    repulsion_manager_->getRepulsionData(
      obstacles, link_pois, current_tcp_pose, global_frame_);
  }

  LocalPlannerOutput local_output = local_planner_->compute(
      current_tcp_pose, waypoint, obstacles, link_pois, dt);
  
  // ... resto del codice ...
}
```

---

## Step di Implementazione

### Step 1: Preparazione (Non modifica comportamento)

1. Creare `RepulsivePointConfig` in `types/config_types.hpp`
2. Creare `RepulsionDataManager` (header + cpp)
3. Aggiungere subscriber a `/robot_points_info` nel manager
4. Aggiungere include di `scene_builder/RobotPointsInfo.h` nel CMakeLists

**File coinvolti:**
- `types/config_types.hpp` (modifica)
- `components/repulsion_data_manager.hpp` (nuovo)
- `components/repulsion_data_manager.cpp` (nuovo)
- `CMakeLists.txt` (modifica per dipendenza scene_builder msgs)

### Step 2: Integrazione base nel Controller

1. Aggiungere `RepulsionDataManager` come membro del controller
2. Inizializzarlo in `initializeComponents()`
3. Caricare configurazione POI da parametri yaml
4. Chiamare `getRepulsionData()` in `executePipeline()`

**File coinvolti:**
- `cartesian_velocity_controller.hpp` (modifica)
- `cartesian_velocity_controller.cpp` (modifica)
- `config/controller_params.yaml` (modifica)

### Step 3: Dynamic Reconfigure

1. Estendere `ControllerTuning.cfg` con parametri POI
2. Gestire callback in `dynamicReconfigureCallback()`
3. Propagare modifiche al `RepulsionDataManager`

**File coinvolti:**
- `cfg/ControllerTuning.cfg` (modifica)
- `cartesian_velocity_controller.cpp` (modifica callback)

### Step 4: Gestione Frame TF

1. Implementare trasformazione TF nel `RepulsionDataManager`
2. Aggiungere gestione errori (fallback repulsione = 0 se TF manca)
3. Log throttled in caso di mismatch frame

**File coinvolti:**
- `components/repulsion_data_manager.cpp` (modifica)

### Step 5: Debug e Visualizzazione

1. Aggiungere marker RViz per i POI attivi
2. Aggiungere info nel `PipelineDebug` message
3. Log diagnostici

**File coinvolti:**
- `components/marker_publisher.cpp` (modifica)
- `msg/PipelineDebug.msg` (modifica opzionale)

---

## Punti Aperti e Dubbi

### 1. **Calcolo di `position_link` per il Jacobiano**

**Problema:** Il `LocalPlanner` usa `position_link` (offset nel frame del link) per calcolare il Jacobiano parziale. Il messaggio `RobotPointDistanceInfo` fornisce solo `position` (world frame).

**Opzioni:**
- **A)** Calcolare `position_link` nel controller: `position_link = T_link_world^{-1} * position_world`
- **B)** Modificare `scene_builder` per includere `position_link` nel messaggio
- **C)** Ignorare l'offset (usare `position_link = [0,0,0]`) - approssimazione accettabile se i POI sono sul centro del link

**Mia raccomandazione:** Opzione A o C per semplicità iniziale.

**Risposta:** Usa l'opzione A

### 2. **Gestione del "TCP POI"**

**Problema:** Il TCP è speciale: deve generare `ObstacleInfo` (non `LinkPOI`) perché la repulsione TCP agisce direttamente sulla velocità lineare, non attraverso il Jacobiano.

**Domande:**
- Vuoi definire un POI chiamato "tcp" in `scene_builder_params.yaml` e usarlo come sorgente per la repulsione TCP?
- Oppure preferisci usare la posizione TCP calcolata direttamente dal controller?

**Mia raccomandazione:** Definire "tcp" nel scene_builder per coerenza (tutti i POI da un'unica sorgente).

**Risposta:** Va bene come suggerisci

### 3. **Nomi dei POI**

**Domanda:** Quali nomi vuoi usare per i 4 POI iniziali? Proposta:
- `tcp` - end effector
- `elbow` - gomito (forearm_link)
- `wrist` - polso (wrist_3_link)
- `forearm_mid` - punto intermedio sull'avambraccio (opzionale)

**Questi nomi devono matchare** quelli definiti in `scene_builder_params.yaml`.

**Risposta:** Va bene come suggerisci, anche quello opzionale

### 4. **Comportamento quando `/robot_points_info` non arriva**

**Opzioni:**
- **A)** Repulsione = 0 (sicuro, il robot si muove normalmente)
- **B)** Fermare il robot (troppo conservativo?)
- **C)** Usare ultimi dati validi (rischioso se vecchi)

**Mia raccomandazione:** Opzione A con log warning throttled.

**Risposta:** Va bene come suggerisci

### 5. **Trasformazione Frame**

**Scenario:** `scene_builder` pubblica in `world`, controller usa `world` → nessun problema.

**Ma se divergono?** Serve TF2. Costo computazionale basso, ma aggiunge complessità.

**Domanda:** Possiamo assumere che entrambi usino lo stesso frame (es. `world`)? Se sì, semplifichiamo l'implementazione iniziale.

**Risposta:** Per il momento facciamo questa assunzione, modificheremo in seguito se necessario

### 6. **Prestazioni a 100 Hz**

Il controller gira a ~100 Hz. L'operazione in `getRepulsionData()` deve essere leggera:
- Nessun lookup TF nel loop critico (cache transform)
- Nessuna allocazione dinamica (pre-allocare vettori)
- Mutex lock breve

**Valutazione:** OK, l'approccio proposto è leggero.

**Risposta:** Va benissimo

### 7. **Interazione repulsione TCP vs repulsione Link**

Attualmente il `LocalPlanner`:
- Repulsione TCP → modifica velocità lineare direttamente
- Repulsione Link → genera velocità joint, poi convertita in Cartesian via Jacobiano

**Domanda:** Questo comportamento è quello che desideri? Oppure vuoi che anche il "TCP POI" passi per il Jacobiano come gli altri?

**Mia opinione:** L'approccio attuale ha senso - TCP diretto è più reattivo.

**Risposta:** Va benissimo

---

## Test Plan

### Test 1: Cablaggio Base
- Avviare `scene_builder` + controller
- Verificare che il callback riceva messaggi (log)
- Verificare timestamp aggiornato

### Test 2: Repulsione TCP
- Definire POI "tcp" in scene_builder
- Abilitare `poi_tcp_enabled = true` via rqt_reconfigure
- Impostare `repulsive_obstacle_gain > 0`
- Avvicinare un ostacolo e verificare deviazione

### Test 3: Repulsione Link
- Abilitare `poi_elbow_enabled = true`
- Impostare `repulsive_link_gain > 0`
- Avvicinare ostacolo al gomito e verificare reazione

### Test 4: Modifica Runtime
- Durante il movimento, cambiare `poi_elbow_weight` via rqt_reconfigure
- Verificare che la forza repulsiva cambi immediatamente

### Test 5: Raggio Dinamico
- Impostare `poi_elbow_radius = 0.1`
- Verificare che la repulsione inizi prima (a distanza maggiore)

### Test 6: Dati Stale
- Fermare `scene_builder`
- Verificare che dopo `stale_timeout` la repulsione vada a zero (non blocchi il robot)

---

## Configurazione YAML Proposta

### controller_params.yaml (aggiunta)

```yaml
# ============================================================================
# REPULSIVE POI CONFIGURATION
# ============================================================================

repulsion:
  enabled: false                        # Master enable
  robot_points_topic: "/robot_points_info"
  stale_timeout: 0.3                    # seconds
  tcp_point_name: "tcp"                 # POI che funge da TCP

  # Configurazione iniziale POI (sovrascrivibile da dynamic_reconfigure)
  points:
    tcp:
      weight: 1.0
      radius: 0.05
      enabled: false
      is_tcp: true
    elbow:
      weight: 1.0
      radius: 0.05
      enabled: false
      is_tcp: false
    wrist:
      weight: 1.0
      radius: 0.05
      enabled: false
      is_tcp: false
    forearm_mid:
      weight: 0.8
      radius: 0.03
      enabled: false
      is_tcp: false
```

### scene_builder_params.yaml (aggiunta)

```yaml
robot_points_of_interest:
  tcp:
    link: "tool0"
    offset: [0.0, 0.0, 0.0]
  elbow:
    link: "forearm_link"
    offset: [0.0, 0.0, 0.0]
  wrist:
    link: "wrist_3_link"
    offset: [0.0, 0.0, 0.0]
  forearm_mid:
    link: "forearm_link"
    offset: [0.0, 0.0, 0.3]  # 30cm lungo l'avambraccio
```

---

## Riassunto Decisioni Prese

| Aspetto | Decisione |
|---------|-----------|
| Sorgente dati | Solo `/robot_points_info` |
| Codice legacy `/distance_info` | Mantenuto ma non usato |
| Modifica runtime | C++ interno + rqt_reconfigure |
| POI iniziali | 4 (tcp, elbow, wrist, forearm_mid) |
| Frame | Assumere stesso frame (semplificazione iniziale) |
| Dati stale | Repulsione = 0 + warning |
| `position_link` | Calcolare nel controller (opzione A) |

---

## Prossimi Passi

1. **Conferma i punti aperti** sopra elencati
2. **Definisci i nomi esatti** dei POI che vuoi usare
3. **Conferma la struttura** dei parametri yaml
4. Una volta confermato, procediamo con l'implementazione Step 1

---

*Documento generato il: 17 Dicembre 2024*
