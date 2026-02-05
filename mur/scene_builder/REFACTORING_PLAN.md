# 🔧 Piano di Rifattorizzazione - scene_builder

## 📋 Indice

1. [Analisi dello Stato Attuale](#1-analisi-dello-stato-attuale)
2. [Obiettivi della Rifattorizzazione](#2-obiettivi-della-rifattorizzazione)
3. [Nuova Architettura Proposta](#3-nuova-architettura-proposta)
4. [Piano di Implementazione Dettagliato](#4-piano-di-implementazione-dettagliato)
5. [Ordine di Esecuzione](#5-ordine-di-esecuzione)
6. [Test e Validazione](#6-test-e-validazione)
7. [Note per l'Implementazione](#7-note-per-limplementazione)

---

## 1. Analisi dello Stato Attuale

### 1.1 Struttura File Corrente

```
scene_builder/
├── include/scene_builder/
│   ├── distance_monitor.hpp
│   ├── distance_query_manager.hpp
│   ├── object_command_node.hpp
│   ├── object_manager.hpp
│   └── internal/  (vuota)
├── src/
│   ├── distance_monitor.cpp
│   ├── distance_monitor_node.cpp
│   ├── distance_query_manager.cpp
│   ├── object_command_node.cpp  (contiene anche main)
│   └── object_manager.cpp
├── scripts/
│   └── scene_interactive_control.py  (1705 linee)
├── msg/
│   ├── DistanceContact.msg
│   ├── DistanceInfo.msg
│   ├── ObjectCommand.msg
│   └── ObjectVelocityCommand.msg
├── srv/
│   ├── ClearMotionSequence.srv
│   ├── GetMotionSequence.srv
│   ├── ListObjects.srv
│   └── SetMotionSequence.srv
├── config/
│   ├── scene_builder_params.yaml
│   └── scene_config.yaml
└── launch/
    ├── distance_monitor.launch
    ├── object_command.launch
    ├── ur10e_complete_bringup.launch
    └── ur10e_complete_with_distance_monitor.launch
```

### 1.2 Problemi Identificati

#### 🔴 Critici

| # | Problema | File | Impatto |
|---|----------|------|---------|
| P1 | **Codice duplicato** - Funzioni `identityPose()`, `sanitizePose()`, `interpolatePose()`, `integrateTwist()` definite 2 volte | `object_manager.cpp` | Manutenzione, Bug |
| P2 | **God Class** - `ObjectManager` gestisce troppe responsabilità | `object_manager.hpp/cpp` | Estendibilità |
| P3 | **PlanningSceneMonitor duplicata** - Creata sia in `DistanceMonitor` che in `DistanceQueryManager` | Entrambi | Performance, Memoria |
| P4 | **CMakeLists obsoleto** - Riferimento a file eliminato `move_to_pose.py` | `CMakeLists.txt` | Build warning |
| P5 | **Script Python monolitico** - 1700+ linee in un singolo file | `scene_interactive_control.py` | Manutenibilità |

#### 🟡 Importanti

| # | Problema | File | Impatto |
|---|----------|------|---------|
| P6 | **Nessuna libreria condivisa** - Codice duplicato negli eseguibili | `CMakeLists.txt` | Dimensione binari |
| P7 | **Mix italiano/inglese** nei commenti | Vari | Consistenza |
| P8 | **Nessun test** - Sezione testing commentata | `CMakeLists.txt` | Qualità |
| P9 | **ObjectCommandNode troppo grande** - 750+ linee con main incluso | `object_command_node.cpp` | Leggibilità |
| P10 | **Parametri hardcoded** sparsi nel codice | Vari | Configurabilità |

#### 🟢 Minori

| # | Problema | File | Impatto |
|---|----------|------|---------|
| P11 | **Cartella internal/ vuota** | `include/scene_builder/internal/` | Pulizia |
| P12 | **Commenti template catkin** non rimossi | `CMakeLists.txt`, `package.xml` | Pulizia |

---

## 2. Obiettivi della Rifattorizzazione

### 2.1 Obiettivi Primari

1. **Separazione delle responsabilità** - Ogni classe ha una singola responsabilità
2. **Eliminazione duplicazioni** - DRY (Don't Repeat Yourself)
3. **Libreria condivisa** - Codice comune compilato una volta
4. **Estendibilità** - Facile aggiungere nuovi tipi di oggetti/primitive
5. **Testabilità** - Componenti testabili in isolamento

### 2.2 Obiettivi Secondari

1. **Consistenza** - Commenti tutti in inglese
2. **Documentazione** - README aggiornato con nuova architettura
3. **Performance** - Singola PlanningSceneMonitor condivisa
4. **Script Python modulare** - Suddivisione in moduli

---

## 3. Nuova Architettura Proposta

### 3.1 Nuova Struttura Directory

```
scene_builder/
├── include/scene_builder/
│   │
│   ├── core/                          # Componenti core riusabili
│   │   ├── pose_utils.hpp             # [NUOVO] Utility per pose/quaternion
│   │   ├── yaml_parser.hpp            # [NUOVO] Parsing YAML oggetti
│   │   └── planning_scene_holder.hpp  # [NUOVO] Wrapper per PlanningSceneMonitor condivisa
│   │
│   ├── objects/                       # Gestione oggetti
│   │   ├── collision_object_factory.hpp  # [NUOVO] Factory per creare oggetti
│   │   ├── object_state.hpp           # [NUOVO] Stato singolo oggetto (estratto da ObjectManager)
│   │   └── object_manager.hpp         # [REFACTOR] Solo gestione oggetti, senza distanze
│   │
│   ├── animation/                     # Sistema animazioni
│   │   ├── waypoint_animator.hpp      # [NUOVO] Gestione animazioni waypoint
│   │   ├── velocity_controller.hpp    # [NUOVO] Controllo velocità oggetti
│   │   └── motion_sequence.hpp        # [NUOVO] Sequenze di movimento
│   │
│   ├── distance/                      # Calcolo distanze
│   │   ├── distance_calculator.hpp    # [REFACTOR da DistanceQueryManager]
│   │   └── distance_visualizer.hpp    # [NUOVO] Creazione marker RViz
│   │
│   └── nodes/                         # Interfacce nodo ROS
│       ├── distance_monitor_node.hpp  # [REFACTOR] Solo interfaccia ROS
│       └── object_command_node.hpp    # [REFACTOR] Solo interfaccia ROS
│
├── src/
│   ├── core/
│   │   ├── pose_utils.cpp
│   │   ├── yaml_parser.cpp
│   │   └── planning_scene_holder.cpp
│   │
│   ├── objects/
│   │   ├── collision_object_factory.cpp
│   │   ├── object_state.cpp
│   │   └── object_manager.cpp
│   │
│   ├── animation/
│   │   ├── waypoint_animator.cpp
│   │   ├── velocity_controller.cpp
│   │   └── motion_sequence.cpp
│   │
│   ├── distance/
│   │   ├── distance_calculator.cpp
│   │   └── distance_visualizer.cpp
│   │
│   └── nodes/
│       ├── distance_monitor_node.cpp
│       ├── distance_monitor_main.cpp  # [NUOVO] Separazione main
│       ├── object_command_node.cpp
│       └── object_command_main.cpp    # [NUOVO] Separazione main
│
├── scripts/
│   ├── scene_interactive_control.py   # [REFACTOR] Entry point
│   └── scene_control/                 # [NUOVO] Modulo Python
│       ├── __init__.py
│       ├── controller.py              # Logica controller principale
│       ├── ros_interface.py           # Comunicazione ROS
│       ├── menu_ui.py                 # Interfaccia utente terminale
│       ├── object_manager.py          # Gestione stato oggetti
│       └── utils.py                   # Utility (colori, input, ecc.)
│
├── test/                              # [NUOVO] Test directory
│   ├── test_pose_utils.cpp
│   ├── test_yaml_parser.cpp
│   ├── test_waypoint_animator.cpp
│   └── test_collision_object_factory.cpp
│
├── msg/                               # Invariato
├── srv/                               # Invariato
├── config/                            # Invariato
└── launch/                            # Invariato
```

### 3.2 Diagramma delle Dipendenze

```
                    ┌──────────────────────────────────────────┐
                    │           ROS Interface Layer            │
                    │  ┌─────────────────┐ ┌────────────────┐  │
                    │  │DistanceMonitor  │ │ObjectCommand   │  │
                    │  │     Node        │ │    Node        │  │
                    │  └────────┬────────┘ └───────┬────────┘  │
                    └───────────┼──────────────────┼───────────┘
                                │                  │
                    ┌───────────┼──────────────────┼───────────┐
                    │           │   Business Logic │           │
                    │           ▼                  ▼           │
                    │  ┌─────────────────┐ ┌────────────────┐  │
                    │  │   Distance      │ │  Object        │  │
                    │  │  Calculator     │ │  Manager       │  │
                    │  └────────┬────────┘ └───────┬────────┘  │
                    │           │                  │           │
                    │           │    ┌─────────────┤           │
                    │           │    │             │           │
                    │           ▼    ▼             ▼           │
                    │  ┌─────────────────┐ ┌────────────────┐  │
                    │  │   Distance      │ │   Animation    │  │
                    │  │  Visualizer     │ │   System       │  │
                    │  └─────────────────┘ │ ┌────────────┐ │  │
                    │                      │ │  Waypoint  │ │  │
                    │                      │ │  Animator  │ │  │
                    │                      │ ├────────────┤ │  │
                    │                      │ │ Velocity   │ │  │
                    │                      │ │ Controller │ │  │
                    │                      │ └────────────┘ │  │
                    │                      └────────────────┘  │
                    └──────────────────────────────────────────┘
                                        │
                    ┌───────────────────┼───────────────────────┐
                    │                   │      Core Layer       │
                    │   ┌───────────────┴───────────────┐       │
                    │   │    PlanningSceneHolder        │       │
                    │   │   (Shared PlanningScene)      │       │
                    │   └───────────────────────────────┘       │
                    │                                           │
                    │   ┌─────────────┐  ┌─────────────┐        │
                    │   │ PoseUtils   │  │ YamlParser  │        │
                    │   └─────────────┘  └─────────────┘        │
                    │                                           │
                    │   ┌─────────────────────────────────┐     │
                    │   │   CollisionObjectFactory        │     │
                    │   └─────────────────────────────────┘     │
                    └───────────────────────────────────────────┘
```

### 3.3 Classi Principali

#### 3.3.1 Core Layer

```cpp
// pose_utils.hpp - Utility per pose e quaternion
namespace scene_builder::core {

geometry_msgs::Pose identityPose();
geometry_msgs::Pose sanitizePose(const geometry_msgs::Pose& pose);
geometry_msgs::Pose interpolatePose(const geometry_msgs::Pose& a,
                                    const geometry_msgs::Pose& b,
                                    double t);
geometry_msgs::Pose integrateTwist(const geometry_msgs::Pose& pose,
                                   const geometry_msgs::Twist& twist,
                                   double dt);
double quaternionNorm(const geometry_msgs::Quaternion& q);
void normalizeQuaternion(geometry_msgs::Quaternion& q);

}  // namespace scene_builder::core
```

```cpp
// planning_scene_holder.hpp - Wrapper per PlanningSceneMonitor condivisa
namespace scene_builder::core {

class PlanningSceneHolder {
public:
  static PlanningSceneHolder& instance();
  
  void initialize(const ros::NodeHandle& nh,
                  const std::string& robot_description = "robot_description");
  
  planning_scene_monitor::PlanningSceneMonitorPtr getMonitor();
  planning_scene_monitor::LockedPlanningSceneRO getSceneRO();
  planning_scene_monitor::LockedPlanningSceneRW getSceneRW();
  
  bool isInitialized() const;
  std::string getPlanningFrame() const;
  
private:
  PlanningSceneHolder() = default;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  std::mutex mutex_;
  bool initialized_ = false;
};

}  // namespace scene_builder::core
```

```cpp
// yaml_parser.hpp - Parsing YAML per oggetti e sequenze
namespace scene_builder::core {

struct ObjectDefinition {
  std::string id;
  std::string frame_id;
  std::string primitive_type;  // "box", "sphere", "cylinder"
  std::vector<double> dimensions;
  geometry_msgs::Pose pose;
};

struct WaypointDefinition {
  geometry_msgs::Pose pose;
  double duration;
};

struct MotionSequenceDefinition {
  std::string object_id;
  bool loop;
  std::vector<WaypointDefinition> waypoints;
};

class YamlParser {
public:
  static std::vector<ObjectDefinition> parseObjects(
      const ros::NodeHandle& nh,
      const std::string& param_name);
  
  static std::vector<MotionSequenceDefinition> parseMotionSequences(
      const ros::NodeHandle& nh,
      const std::string& param_name);
  
  static ObjectDefinition parseObjectEntry(
      const std::string& id,
      const XmlRpc::XmlRpcValue& entry);
};

}  // namespace scene_builder::core
```

#### 3.3.2 Objects Layer

```cpp
// collision_object_factory.hpp - Factory per creare CollisionObject
namespace scene_builder::objects {

class CollisionObjectFactory {
public:
  static moveit_msgs::CollisionObject createBox(
      const std::string& id,
      const std::string& frame_id,
      const geometry_msgs::Pose& pose,
      double size_x, double size_y, double size_z);
  
  static moveit_msgs::CollisionObject createSphere(
      const std::string& id,
      const std::string& frame_id,
      const geometry_msgs::Pose& pose,
      double radius);
  
  static moveit_msgs::CollisionObject createCylinder(
      const std::string& id,
      const std::string& frame_id,
      const geometry_msgs::Pose& pose,
      double height, double radius);
  
  static moveit_msgs::CollisionObject createFromDefinition(
      const core::ObjectDefinition& def);
  
  static moveit_msgs::CollisionObject createMoveCommand(
      const std::string& id,
      const std::string& frame_id,
      const geometry_msgs::Pose& new_pose);
  
  static moveit_msgs::CollisionObject createRemoveCommand(
      const std::string& id,
      const std::string& frame_id);
};

}  // namespace scene_builder::objects
```

```cpp
// object_state.hpp - Stato di un singolo oggetto
namespace scene_builder::objects {

struct ObjectState {
  std::string id;
  geometry_msgs::Pose last_pose;
  bool has_pose = false;
  
  // Animation state
  std::deque<animation::WaypointCommand> waypoint_queue;
  std::optional<animation::VelocityCommand> active_velocity;
  
  // Sequence state
  std::vector<animation::WaypointCommand> sequence_template;
  bool loop_sequence = false;
  std::size_t next_sequence_index = 0;
  bool sequence_active = false;
};

}  // namespace scene_builder::objects
```

```cpp
// object_manager.hpp - Gestione oggetti (semplificata)
namespace scene_builder::objects {

class ObjectManager {
public:
  explicit ObjectManager(const std::string& planning_frame);
  
  // Object CRUD
  bool addObject(const moveit_msgs::CollisionObject& object, std::string& error);
  bool removeObject(const std::string& object_id, std::string& error);
  bool moveObject(const std::string& object_id, const geometry_msgs::Pose& pose, std::string& error);
  
  // Query
  bool getObjectPose(const std::string& object_id, geometry_msgs::Pose& pose_out) const;
  std::vector<std::string> listObjectIds() const;
  std::vector<moveit_msgs::CollisionObject> listObjects() const;
  bool objectExists(const std::string& object_id) const;
  
  // State access (for animation system)
  ObjectState* getObjectState(const std::string& object_id);
  const ObjectState* getObjectState(const std::string& object_id) const;
  ObjectState& getOrCreateState(const std::string& object_id);
  
  // Bulk operations
  void loadFromDefinitions(const std::vector<core::ObjectDefinition>& definitions);
  
private:
  moveit::planning_interface::PlanningSceneInterface scene_interface_;
  std::string planning_frame_;
  std::unordered_map<std::string, ObjectState> states_;
  mutable std::mutex mutex_;
};

}  // namespace scene_builder::objects
```

#### 3.3.3 Animation Layer

```cpp
// waypoint_animator.hpp - Gestione animazioni waypoint
namespace scene_builder::animation {

struct WaypointCommand {
  geometry_msgs::Pose target_pose;
  ros::Duration duration_hint{0.0};
  ros::Time start_time;
  geometry_msgs::Pose start_pose;
  bool started = false;
};

class WaypointAnimator {
public:
  explicit WaypointAnimator(double default_duration = 0.5);
  
  // Queue management
  void queueWaypoint(objects::ObjectState& state, const WaypointCommand& cmd);
  void clearQueue(objects::ObjectState& state);
  bool hasActiveAnimation(const objects::ObjectState& state) const;
  
  // Update (returns new pose if animation is active)
  std::optional<geometry_msgs::Pose> update(
      objects::ObjectState& state,
      const ros::Time& now);
  
private:
  double default_duration_;
};

}  // namespace scene_builder::animation
```

```cpp
// velocity_controller.hpp - Controllo velocità oggetti
namespace scene_builder::animation {

struct VelocityCommand {
  geometry_msgs::Twist twist;
  std::string reference_frame;
  ros::Duration timeout{0.0};
  ros::Time stamp;
};

class VelocityController {
public:
  explicit VelocityController(double max_velocity = 1.0, double default_timeout = 1.0);
  
  // Apply velocity
  void applyVelocity(objects::ObjectState& state, const VelocityCommand& cmd);
  void stopVelocity(objects::ObjectState& state);
  bool hasActiveVelocity(const objects::ObjectState& state) const;
  
  // Update (returns new pose if velocity is active)
  std::optional<geometry_msgs::Pose> update(
      objects::ObjectState& state,
      const ros::Time& now,
      double dt);
  
  // Configuration
  void setMaxVelocity(double max_vel) { max_velocity_ = max_vel; }
  void setDefaultTimeout(double timeout) { default_timeout_ = timeout; }
  
private:
  geometry_msgs::Twist clampVelocity(const geometry_msgs::Twist& twist) const;
  
  double max_velocity_;
  double default_timeout_;
};

}  // namespace scene_builder::animation
```

```cpp
// motion_sequence.hpp - Gestione sequenze di movimento
namespace scene_builder::animation {

class MotionSequenceManager {
public:
  void setSequence(objects::ObjectState& state,
                   const std::vector<WaypointCommand>& sequence,
                   bool loop);
  
  void clearSequence(objects::ObjectState& state);
  
  bool isSequenceActive(const objects::ObjectState& state) const;
  bool isLooping(const objects::ObjectState& state) const;
  
  std::size_t getCurrentIndex(const objects::ObjectState& state) const;
  const std::vector<WaypointCommand>& getSequence(const objects::ObjectState& state) const;
  
  // Called when waypoint queue is empty to enqueue next
  void enqueueNextIfNeeded(objects::ObjectState& state);
};

}  // namespace scene_builder::animation
```

#### 3.3.4 Distance Layer

```cpp
// distance_calculator.hpp - Calcolo distanze
namespace scene_builder::distance {

struct DistanceResult {
  double distance = std::numeric_limits<double>::infinity();
  std::string link_name;
  std::string object_id;
  Eigen::Vector3d robot_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d object_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d distance_vector = Eigen::Vector3d::Zero();
};

class DistanceCalculator {
public:
  explicit DistanceCalculator(const std::string& move_group_name);
  
  // Compute all distances (for visualization)
  std::vector<DistanceResult> computeAllDistances();
  
  // Compute minimum distance only
  std::optional<DistanceResult> computeMinimumDistance(
      const std::vector<std::string>& link_filter = {});
  
  // Configuration
  void setDistanceThreshold(double threshold);
  void setMaxContactsPerBody(int max_contacts);
  
private:
  void configureDistanceRequest();
  DistanceResult processDistanceData(
      const collision_detection::DistanceResultsData& data,
      const robot_state::RobotState& state,
      const planning_scene::PlanningSceneConstPtr& scene);
  
  std::string move_group_name_;
  collision_detection::DistanceRequest request_;
};

}  // namespace scene_builder::distance
```

```cpp
// distance_visualizer.hpp - Visualizzazione marker RViz
namespace scene_builder::distance {

struct VisualizationConfig {
  double arrow_length = 0.15;
  double arrow_shaft_diameter = 0.01;
  double arrow_head_diameter = 0.02;
  double arrow_head_length = 0.03;
  double point_radius = 0.015;
};

class DistanceVisualizer {
public:
  explicit DistanceVisualizer(const VisualizationConfig& config = {});
  
  visualization_msgs::MarkerArray createMarkers(
      const std::vector<DistanceResult>& distances,
      const std::string& frame_id,
      const ros::Time& stamp);
  
  void setConfig(const VisualizationConfig& config) { config_ = config; }
  
private:
  visualization_msgs::Marker createPointMarker(
      const Eigen::Vector3d& point,
      int id, const std::string& ns,
      const std::string& frame_id, const ros::Time& stamp,
      double r, double g, double b);
  
  visualization_msgs::Marker createArrowMarker(
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      int id, const std::string& ns,
      const std::string& frame_id, const ros::Time& stamp);
  
  VisualizationConfig config_;
};

}  // namespace scene_builder::distance
```

---

## 4. Piano di Implementazione Dettagliato

### Fase 1: Preparazione e Cleanup (1-2 ore)

#### Task 1.1: Pulizia CMakeLists.txt
**File:** `CMakeLists.txt`
**Azioni:**
1. Rimuovere riferimento a `move_to_pose.py` (linea 191)
2. Rimuovere commenti template catkin non necessari
3. Preparare struttura per libreria condivisa

```cmake
# Aggiungere dopo le dipendenze
add_library(${PROJECT_NAME}_core SHARED
  src/core/pose_utils.cpp
  src/core/yaml_parser.cpp
  src/core/planning_scene_holder.cpp
)
```

#### Task 1.2: Pulizia package.xml
**File:** `package.xml`
**Azioni:**
1. Aggiornare descrizione
2. Rimuovere commenti template
3. Aggiungere licenza appropriata

#### Task 1.3: Rimuovere cartella vuota
**Azione:** Eliminare `include/scene_builder/internal/`

---

### Fase 2: Creazione Core Layer (2-3 ore)

#### Task 2.1: Creare pose_utils
**File da creare:**
- `include/scene_builder/core/pose_utils.hpp`
- `src/core/pose_utils.cpp`

**Contenuto:** Estrarre da `object_manager.cpp`:
- `identityPose()`
- `sanitizePose()`
- `interpolatePose()`
- `integrateTwist()`
- Aggiungere funzioni utility per quaternion

#### Task 2.2: Creare yaml_parser
**File da creare:**
- `include/scene_builder/core/yaml_parser.hpp`
- `src/core/yaml_parser.cpp`

**Contenuto:** Estrarre da `object_manager.cpp`:
- `loadObjectsFromParameter()` → `parseObjects()`
- Da `object_command_node.cpp`:
  - `loadMotionSequences()` → `parseMotionSequences()`
  - Funzioni helper: `xmlRpcToDouble()`, `xmlRpcToBool()`, `readVector()`, `parsePose()`

#### Task 2.3: Creare planning_scene_holder
**File da creare:**
- `include/scene_builder/core/planning_scene_holder.hpp`
- `src/core/planning_scene_holder.cpp`

**Contenuto:** Singleton che gestisce una singola PlanningSceneMonitor:
- Inizializzazione lazy
- Accesso thread-safe
- Metodi per ottenere lock RO/RW

---

### Fase 3: Creazione Objects Layer (2-3 ore)

#### Task 3.1: Creare collision_object_factory
**File da creare:**
- `include/scene_builder/objects/collision_object_factory.hpp`
- `src/objects/collision_object_factory.cpp`

**Contenuto:** Factory methods per creare CollisionObject:
- `createBox()`, `createSphere()`, `createCylinder()`
- `createFromDefinition()`
- `createMoveCommand()`, `createRemoveCommand()`

#### Task 3.2: Creare object_state
**File da creare:**
- `include/scene_builder/objects/object_state.hpp`

**Contenuto:** Estrarre struct `ObjectState` da `ObjectManager`

#### Task 3.3: Rifattorizzare object_manager
**File da modificare:**
- `include/scene_builder/object_manager.hpp`
- `src/object_manager.cpp`

**Azioni:**
1. Rimuovere codice duplicato (usare pose_utils)
2. Rimuovere parsing YAML (usare yaml_parser)
3. Rimuovere `DistanceQueryManager` (calcolo distanze va in DistanceCalculator)
4. Spostare in `objects/` namespace
5. Semplificare responsabilità

---

### Fase 4: Creazione Animation Layer (2-3 ore)

#### Task 4.1: Creare waypoint_animator
**File da creare:**
- `include/scene_builder/animation/waypoint_animator.hpp`
- `src/animation/waypoint_animator.cpp`

**Contenuto:** Estrarre da `ObjectManager::update()`:
- Logica gestione waypoint queue
- Interpolazione pose
- Gestione timing

#### Task 4.2: Creare velocity_controller
**File da creare:**
- `include/scene_builder/animation/velocity_controller.hpp`
- `src/animation/velocity_controller.cpp`

**Contenuto:** Estrarre da `ObjectManager`:
- `applyVelocity()`
- Logica update velocità
- Clamping velocità

#### Task 4.3: Creare motion_sequence
**File da creare:**
- `include/scene_builder/animation/motion_sequence.hpp`
- `src/animation/motion_sequence.cpp`

**Contenuto:** Estrarre da `ObjectManager`:
- `setWaypointSequence()`
- `enqueueNextSequenceWaypoint()`
- `getSequenceState()`

---

### Fase 5: Creazione Distance Layer (2-3 ore)

#### Task 5.1: Creare distance_calculator
**File da creare:**
- `include/scene_builder/distance/distance_calculator.hpp`
- `src/distance/distance_calculator.cpp`

**Contenuto:** Unire e rifattorizzare:
- Da `DistanceQueryManager`: logica query distanze
- Da `DistanceMonitor::timerCallback()`: elaborazione risultati

#### Task 5.2: Creare distance_visualizer
**File da creare:**
- `include/scene_builder/distance/distance_visualizer.hpp`
- `src/distance/distance_visualizer.cpp`

**Contenuto:** Estrarre da `DistanceMonitor`:
- `makeSphereMarker()`
- `makePointMarker()`
- `makeArrowMarker()`
- Logica creazione MarkerArray

#### Task 5.3: Eliminare distance_query_manager
**File da eliminare:**
- `include/scene_builder/distance_query_manager.hpp`
- `src/distance_query_manager.cpp`

**Nota:** Funzionalità assorbita da `DistanceCalculator`

---

### Fase 6: Rifattorizzazione Nodi ROS (3-4 ore)

#### Task 6.1: Rifattorizzare distance_monitor_node
**File da modificare:**
- `include/scene_builder/distance_monitor.hpp` → `include/scene_builder/nodes/distance_monitor_node.hpp`
- `src/distance_monitor.cpp` → `src/nodes/distance_monitor_node.cpp`

**File da creare:**
- `src/nodes/distance_monitor_main.cpp` (solo main)

**Azioni:**
1. Usare `PlanningSceneHolder` invece di creare PlanningSceneMonitor
2. Usare `DistanceCalculator` per calcoli
3. Usare `DistanceVisualizer` per marker
4. Usare `YamlParser` per caricare oggetti
5. Semplificare la classe a sola interfaccia ROS

#### Task 6.2: Rifattorizzare object_command_node
**File da modificare:**
- `include/scene_builder/object_command_node.hpp` → `include/scene_builder/nodes/object_command_node.hpp`
- `src/object_command_node.cpp` → `src/nodes/object_command_node.cpp`

**File da creare:**
- `src/nodes/object_command_main.cpp` (solo main)

**Azioni:**
1. Usare nuovo `ObjectManager` semplificato
2. Usare `WaypointAnimator`, `VelocityController`, `MotionSequenceManager`
3. Usare `YamlParser` per caricamento
4. Estrarre main in file separato
5. Semplificare a sola interfaccia ROS

---

### Fase 7: Rifattorizzazione Script Python (2-3 ore)

#### Task 7.1: Creare struttura modulo
**Directory da creare:** `scripts/scene_control/`

**File da creare:**
- `scripts/scene_control/__init__.py`
- `scripts/scene_control/utils.py` (Colors, print functions, input functions)
- `scripts/scene_control/ros_interface.py` (Publishers, subscribers, service clients)
- `scripts/scene_control/object_manager.py` (ObjectInfo, gestione stato)
- `scripts/scene_control/menu_ui.py` (Tutti i menu handlers)
- `scripts/scene_control/controller.py` (SceneInteractiveController semplificato)

#### Task 7.2: Rifattorizzare script principale
**File:** `scripts/scene_interactive_control.py`

**Nuova struttura:**
```python
#!/usr/bin/env python3
"""Scene Interactive Control - Entry Point"""

from scene_control.controller import SceneInteractiveController

def main():
    controller = SceneInteractiveController()
    controller.run()

if __name__ == '__main__':
    main()
```

---

### Fase 8: Aggiornamento Build System (1-2 ore)

#### Task 8.1: Aggiornare CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(scene_builder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf2_geometry_msgs
  visualization_msgs
)

# Messages
add_message_files(FILES
  DistanceContact.msg
  DistanceInfo.msg
  ObjectCommand.msg
  ObjectVelocityCommand.msg
)

# Services
add_service_files(FILES
  SetMotionSequence.srv
  GetMotionSequence.srv
  ClearMotionSequence.srv
  ListObjects.srv
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core ${PROJECT_NAME}_objects ${PROJECT_NAME}_animation ${PROJECT_NAME}_distance
  CATKIN_DEPENDS geometry_msgs message_runtime moveit_core moveit_msgs moveit_ros_planning_interface roscpp rospy std_msgs tf2_geometry_msgs visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Core library
add_library(${PROJECT_NAME}_core
  src/core/pose_utils.cpp
  src/core/yaml_parser.cpp
  src/core/planning_scene_holder.cpp
)
add_dependencies(${PROJECT_NAME}_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES})

# Objects library
add_library(${PROJECT_NAME}_objects
  src/objects/collision_object_factory.cpp
  src/objects/object_manager.cpp
)
add_dependencies(${PROJECT_NAME}_objects ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_objects ${PROJECT_NAME}_core ${catkin_LIBRARIES})

# Animation library
add_library(${PROJECT_NAME}_animation
  src/animation/waypoint_animator.cpp
  src/animation/velocity_controller.cpp
  src/animation/motion_sequence.cpp
)
add_dependencies(${PROJECT_NAME}_animation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_animation ${PROJECT_NAME}_core ${catkin_LIBRARIES})

# Distance library
add_library(${PROJECT_NAME}_distance
  src/distance/distance_calculator.cpp
  src/distance/distance_visualizer.cpp
)
add_dependencies(${PROJECT_NAME}_distance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_distance ${PROJECT_NAME}_core ${catkin_LIBRARIES})

# Executables
add_executable(distance_monitor_node
  src/nodes/distance_monitor_node.cpp
  src/nodes/distance_monitor_main.cpp
)
add_dependencies(distance_monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(distance_monitor_node
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_objects
  ${PROJECT_NAME}_distance
  ${catkin_LIBRARIES}
)

add_executable(object_command_node
  src/nodes/object_command_node.cpp
  src/nodes/object_command_main.cpp
)
add_dependencies(object_command_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_command_node
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_objects
  ${PROJECT_NAME}_animation
  ${catkin_LIBRARIES}
)

# Python
catkin_install_python(PROGRAMS
  scripts/scene_interactive_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install
install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_objects
  ${PROJECT_NAME}_animation
  ${PROJECT_NAME}_distance
  distance_monitor_node
  object_command_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Testing
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_pose_utils test/test_pose_utils.cpp)
  target_link_libraries(test_pose_utils ${PROJECT_NAME}_core ${catkin_LIBRARIES})
  
  catkin_add_gtest(test_yaml_parser test/test_yaml_parser.cpp)
  target_link_libraries(test_yaml_parser ${PROJECT_NAME}_core ${catkin_LIBRARIES})
endif()
```

---

### Fase 9: Creazione Test (2-3 ore)

#### Task 9.1: Test pose_utils
**File:** `test/test_pose_utils.cpp`

```cpp
#include <gtest/gtest.h>
#include "scene_builder/core/pose_utils.hpp"

using namespace scene_builder::core;

TEST(PoseUtilsTest, IdentityPose) {
  auto pose = identityPose();
  EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.w, 1.0);
}

TEST(PoseUtilsTest, SanitizeZeroQuaternion) {
  geometry_msgs::Pose pose;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  
  auto sanitized = sanitizePose(pose);
  EXPECT_DOUBLE_EQ(sanitized.orientation.w, 1.0);
}

TEST(PoseUtilsTest, InterpolatePoseMiddle) {
  geometry_msgs::Pose a = identityPose();
  geometry_msgs::Pose b = identityPose();
  b.position.x = 2.0;
  
  auto mid = interpolatePose(a, b, 0.5);
  EXPECT_DOUBLE_EQ(mid.position.x, 1.0);
}

// ... altri test
```

#### Task 9.2: Test yaml_parser
**File:** `test/test_yaml_parser.cpp`

#### Task 9.3: Test collision_object_factory
**File:** `test/test_collision_object_factory.cpp`

---

### Fase 10: Documentazione e Cleanup Finale (1-2 ore)

#### Task 10.1: Aggiornare README.md
- Documentare nuova architettura
- Aggiornare esempi di utilizzo
- Aggiungere sezione sviluppo/contributi

#### Task 10.2: Convertire commenti in inglese
- Uniformare tutti i commenti in inglese

#### Task 10.3: Cleanup finale
- Rimuovere file obsoleti
- Verificare che tutti i test passino
- Verificare che il build sia pulito

---

## 5. Ordine di Esecuzione

```
Settimana 1:
├── Fase 1: Preparazione e Cleanup (Task 1.1 - 1.3)
├── Fase 2: Core Layer (Task 2.1 - 2.3)
└── Fase 3: Objects Layer (Task 3.1 - 3.3)

Settimana 2:
├── Fase 4: Animation Layer (Task 4.1 - 4.3)
├── Fase 5: Distance Layer (Task 5.1 - 5.3)
└── Fase 6: Rifattorizzazione Nodi (Task 6.1 - 6.2)

Settimana 3:
├── Fase 7: Script Python (Task 7.1 - 7.2)
├── Fase 8: Build System (Task 8.1)
├── Fase 9: Test (Task 9.1 - 9.3)
└── Fase 10: Documentazione (Task 10.1 - 10.3)
```

**Tempo totale stimato: 18-26 ore**

---

## 6. Test e Validazione

### 6.1 Test Unitari
- `test_pose_utils.cpp` - Test funzioni pose/quaternion
- `test_yaml_parser.cpp` - Test parsing YAML
- `test_collision_object_factory.cpp` - Test creazione oggetti
- `test_waypoint_animator.cpp` - Test animazioni

### 6.2 Test di Integrazione
1. Verificare che `distance_monitor_node` funzioni come prima
2. Verificare che `object_command_node` funzioni come prima
3. Verificare che lo script Python funzioni come prima
4. Verificare che i launch file funzionino

### 6.3 Test di Regressione
Eseguire i seguenti scenari:
1. Caricare oggetti da YAML
2. Muovere oggetti con waypoints
3. Controllare oggetti con velocity
4. Avviare/fermare loop di movimento
5. Visualizzare distanze in RViz

---

## 7. Note per l'Implementazione

### 7.1 Principi Guida
1. **Incrementale**: Ogni fase deve produrre codice compilabile
2. **Backward Compatible**: I topic/servizi ROS non cambiano
3. **Test First**: Scrivere test prima di rifattorizzare

### 7.2 Rischi e Mitigazioni

| Rischio | Mitigazione |
|---------|-------------|
| Breaking changes | Mantenere API esterne identiche |
| Bug di regressione | Test di integrazione ad ogni fase |
| Tempo eccessivo | Prioritizzare fasi critiche (1-5) |

### 7.3 Metriche di Successo
- [ ] Nessuna duplicazione di codice
- [ ] Tutte le classi < 500 linee
- [ ] Copertura test > 60% su core
- [ ] Build senza warning
- [ ] Documentazione completa

---

## Appendice A: File da Eliminare

```
include/scene_builder/distance_query_manager.hpp
src/distance_query_manager.cpp
include/scene_builder/internal/  (directory)
```

## Appendice B: File da Spostare

```
include/scene_builder/object_manager.hpp    → include/scene_builder/objects/object_manager.hpp
include/scene_builder/distance_monitor.hpp  → include/scene_builder/nodes/distance_monitor_node.hpp
include/scene_builder/object_command_node.hpp → include/scene_builder/nodes/object_command_node.hpp
src/object_manager.cpp                      → src/objects/object_manager.cpp
src/distance_monitor.cpp                    → src/nodes/distance_monitor_node.cpp
src/object_command_node.cpp                 → src/nodes/object_command_node.cpp
```

## Appendice C: Nuovi File da Creare

```
# Core
include/scene_builder/core/pose_utils.hpp
include/scene_builder/core/yaml_parser.hpp
include/scene_builder/core/planning_scene_holder.hpp
src/core/pose_utils.cpp
src/core/yaml_parser.cpp
src/core/planning_scene_holder.cpp

# Objects
include/scene_builder/objects/collision_object_factory.hpp
include/scene_builder/objects/object_state.hpp
src/objects/collision_object_factory.cpp

# Animation
include/scene_builder/animation/waypoint_animator.hpp
include/scene_builder/animation/velocity_controller.hpp
include/scene_builder/animation/motion_sequence.hpp
src/animation/waypoint_animator.cpp
src/animation/velocity_controller.cpp
src/animation/motion_sequence.cpp

# Distance
include/scene_builder/distance/distance_calculator.hpp
include/scene_builder/distance/distance_visualizer.hpp
src/distance/distance_calculator.cpp
src/distance/distance_visualizer.cpp

# Nodes
src/nodes/distance_monitor_main.cpp
src/nodes/object_command_main.cpp

# Python
scripts/scene_control/__init__.py
scripts/scene_control/utils.py
scripts/scene_control/ros_interface.py
scripts/scene_control/object_manager.py
scripts/scene_control/menu_ui.py
scripts/scene_control/controller.py

# Test
test/test_pose_utils.cpp
test/test_yaml_parser.cpp
test/test_collision_object_factory.cpp
test/test_waypoint_animator.cpp
```

