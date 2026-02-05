# LocalPlanner - Documentazione Tecnica

## Panoramica

Il `LocalPlanner` è un componente fondamentale del controller di velocità cartesiana che implementa un sistema di pianificazione locale basato sul metodo dei **campi potenziali artificiali** (Artificial Potential Fields - APF). Questo approccio combina forze attrattive verso l'obiettivo con forze repulsive dagli ostacoli per generare traiettorie sicure in tempo reale.

---

## Architettura

### Dipendenze
- `RobotStateManager`: gestione dello stato del robot (posizioni giunti, Jacobiano)
- `JacobianSolver`: risoluzione cinematica inversa tramite Jacobiano

### Thread Safety
Il componente è **thread-safe** grazie a due mutex:
- `params_mutex_`: protegge i parametri dinamici
- `state_mutex_`: protegge lo stato interno (virtual target)

---

## Funzionalità Principali

### 1. Velocità Attrattiva (Attractive Velocity)

#### Componente Lineare
```cpp
Eigen::Vector3d computeAttractiveLinearVelocity(current_pos, waypoint_pos)
```
- Calcola il vettore differenza tra la posizione corrente e il waypoint
- Restituisce un vettore **proporzionale all'errore** (controllo proporzionale)
- Formula: `v_att = k_att * (waypoint - current)`

#### Componente Angolare
```cpp
Eigen::Vector3d computeAttractiveAngularVelocity(current_orientation, waypoint_orientation)
```
- Usa quaternioni per calcolare l'errore di orientamento
- Sceglie automaticamente il percorso più breve (quaternion flip handling)
- Converte la rotazione relativa in rappresentazione asse-angolo
- Formula: `ω_att = k_att * angle * axis`

---

### 2. Velocità Repulsiva - Ostacoli TCP/Payload

```cpp
Eigen::Vector3d computeRepulsiveVelocityFromObstacle(point, obstacle)
Eigen::Vector3d computeRepulsiveVelocityTotal(point, obstacles)
```

Calcola forze repulsive per ogni ostacolo vicino al TCP (Tool Center Point) o payload.

#### Modalità di Calcolo

| Modalità | Formula | Caratteristiche |
|----------|---------|-----------------|
| **LINEAR** | `V = V_max * (d_inf - d) / (d_inf - d_min)` | Decadimento lineare, comportamento legacy |
| **QUADRATIC** | `V = V_max * (d_min / d)²` | Decadimento quadratico, più reattivo vicino agli ostacoli |

#### Parametri Chiave
- `influence_distance`: distanza oltre la quale l'ostacolo non ha effetto
- `min_safe_distance`: distanza minima di sicurezza (velocità = max)
- `taper_start`: 80% della distanza di influenza (per smooth transition)

---

### 3. Velocità Repulsiva - Link POI (Points of Interest)

```cpp
Eigen::VectorXd computeRepulsiveLinkJointVelocity(link_pois)
```

Gestisce l'evitamento ostacoli per punti specifici lungo i link del robot (non solo TCP).

#### Processo di Calcolo
1. Per ogni POI con repulsione attiva (`distance < influence_distance`)
2. Calcola la magnitudine repulsiva (LINEAR o QUADRATIC)
3. Applica il peso del POI (`poi.weight`)
4. Ottiene il Jacobiano parziale per quel link
5. Calcola la pseudo-inversa: `J⁺ = Jᵀ * (J * Jᵀ)⁻¹`
6. Converte in velocità giunti: `q̇ = J⁺ * v_cartesian`

#### Conversione a Cartesiano
Le velocità giunti dei link POI vengono poi riconvertite in velocità cartesiana al TCP tramite il Jacobiano completo per essere integrate nel virtual target.

---

### 4. Virtual Target Integration

```cpp
void integrateTarget(velocity_linear, velocity_angular, dt)
```

Mantiene un **target virtuale** che viene integrato nel tempo:

- **Posizione**: `P_new = P_old + V * dt`
- **Orientamento**: Integrazione esponenziale tramite quaternioni
  ```cpp
  δq = Quaternion(AngleAxis(ω * dt, axis))
  q_new = δq * q_current
  ```

Questo approccio permette un movimento fluido e previene discontinuità.

---

### 5. Limitazione Velocità

```cpp
Eigen::Vector3d limitVelocity(velocity, max_magnitude)
```

Applica un hard limit alla magnitudine del vettore velocità:
- Se `||v|| > max` → `v_limited = v * (max / ||v||)`
- Preserva la direzione del vettore

---

## Pipeline di Calcolo Principale

```
┌─────────────────────────────────────────────────────────────────┐
│                     compute(current_pose, waypoint, ...)        │
└─────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
         ┌────────────────────────┴────────────────────────┐
         │                                                 │
         ▼                                                 ▼
┌─────────────────────┐                      ┌─────────────────────┐
│ Attractive Velocity │                      │ Repulsive Velocity  │
│   (Linear + Angular)│                      │   (Obstacles + POI) │
│      × k_att        │                      │   × k_rep / k_rep_l │
└─────────────────────┘                      └─────────────────────┘
         │                                                 │
         └───────────────────┬─────────────────────────────┘
                             │
                             ▼
                 ┌───────────────────────┐
                 │   Combine Velocities  │
                 │ v_lin = v_att + v_rep │
                 │ v_ang = v_att_ang     │
                 └───────────────────────┘
                             │
                             ▼
                 ┌───────────────────────┐
                 │   Apply Velocity      │
                 │      Limits           │
                 └───────────────────────┘
                             │
                             ▼
                 ┌───────────────────────┐
                 │  Integrate Virtual    │
                 │       Target          │
                 └───────────────────────┘
                             │
                             ▼
                 ┌───────────────────────┐
                 │  Return Output with   │
                 │     Diagnostics       │
                 └───────────────────────┘
```

---

## Struttura Output

```cpp
struct LocalPlannerOutput {
    // Componenti velocità
    Eigen::Vector3d attractive_linear;
    Eigen::Vector3d attractive_angular;
    Eigen::Vector3d repulsive_obstacle_linear;
    Eigen::VectorXd repulsive_links_joint;
    
    // Velocità combinate
    Eigen::Vector3d combined_linear;
    Eigen::Vector3d combined_angular;
    
    // Virtual target
    Eigen::Isometry3d target_raw;
    
    // Diagnostica
    double distance_to_waypoint;
    double angular_distance_to_waypoint;
    double closest_obstacle_distance;
};
```

---

## Parametri Configurabili

| Parametro | Tipo | Default | Descrizione |
|-----------|------|---------|-------------|
| `k_attractive_` | double | - | Guadagno forza attrattiva |
| `k_repulsive_obstacle_` | double | - | Guadagno repulsione ostacoli TCP |
| `k_repulsive_link_` | double | - | Guadagno repulsione link POI |
| `max_linear_velocity_` | double | - | Limite velocità lineare (m/s) |
| `max_angular_velocity_` | double | - | Limite velocità angolare (rad/s) |
| `influence_distance_` | double | - | Distanza di influenza ostacoli (m) |
| `min_safe_distance_` | double | - | Distanza minima di sicurezza (m) |
| `repulsive_mode_` | enum | - | LINEAR o QUADRATIC |

### Setter/Getter Disponibili
Tutti i parametri hanno metodi pubblici thread-safe:
- `setAttractiveGain()` / `getAttractiveGain()`
- `setRepulsiveObstacleGain()` / `getRepulsiveObstacleGain()`
- `setRepulsiveLinkGain()` / `getRepulsiveLinkGain()`
- `setMaxLinearVelocity()` / `getMaxLinearVelocity()`
- `setMaxAngularVelocity()` / `getMaxAngularVelocity()`
- `setInfluenceDistance()` / `getInfluenceDistance()`
- `setMinSafeDistance()` / `getMinSafeDistance()`
- `setRepulsiveMode()` / `getRepulsiveMode()`

---

## Gestione Stato

### Reset
```cpp
void reset()                              // Reset completo
void resetToPosition(const Eigen::Isometry3d& pose)  // Reset a posizione specifica
```

### Query
```cpp
bool isInitialized() const                // Verifica se il target è inizializzato
Eigen::Isometry3d getVirtualTarget() const // Ottiene il target virtuale corrente
```

---

## Costanti

```cpp
static constexpr double kEpsilon = 1e-9;  // Soglia numerica per evitare divisioni per zero
```

---

## Note Implementative

1. **Quaternion Flip Handling**: Il codice gestisce il caso in cui `q.dot(q_target) < 0` invertendo il quaternion target per garantire il percorso più breve.

2. **Pseudo-Inversa senza Damping**: La pseudo-inversa del Jacobiano per i link POI non usa damping (`J⁺ = Jᵀ(JJᵀ)⁻¹`), il che può causare problemi vicino a singolarità.

3. **Remapping Colonne Jacobiano**: Gestisce il caso in cui le colonne del Jacobiano MoveIt non corrispondono all'ordine dei giunti del controller.

4. **Smooth Transition**: Il taper al 80% della distanza di influenza garantisce una transizione graduale verso zero repulsione.

