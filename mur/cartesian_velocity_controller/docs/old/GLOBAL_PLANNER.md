# GlobalPlanner - Documentazione

## Panoramica

Il `GlobalPlanner` è un componente del `cartesian_velocity_controller` responsabile della gestione e navigazione attraverso una sequenza di waypoint. Fornisce funzionalità per definire percorsi, monitorare il progresso e gestire la transizione automatica tra waypoint.

## Caratteristiche Principali

- **Thread-safe**: Tutte le operazioni sono protette da mutex per garantire la sicurezza in ambienti multi-thread
- **Waypoint personalizzabili**: Ogni waypoint può avere parametri di switch personalizzati
- **Sistema di callback**: Notifiche per eventi come raggiungimento waypoint e completamento percorso
- **Configurazione dinamica**: Parametri modificabili a runtime

---

## Funzionalità

### 1. Gestione Waypoint

| Metodo | Descrizione |
|--------|-------------|
| `setWaypoints(waypoints)` | Imposta l'intera lista di waypoint (accetta sia `Eigen::Isometry3d` che `WaypointInfo`) |
| `addWaypoint(waypoint)` | Aggiunge un waypoint alla fine della lista |
| `insertWaypoint(index, waypoint)` | Inserisce un waypoint in una posizione specifica |
| `removeWaypoint(index)` | Rimuove un waypoint dalla posizione specificata |
| `clearWaypoints()` | Cancella tutti i waypoint e resetta lo stato |
| `getWaypointCount()` | Restituisce il numero totale di waypoint |
| `hasWaypoints()` | Verifica se esistono waypoint nella lista |
| `getWaypoints()` | Restituisce una copia di tutti i waypoint |

### 2. Navigazione

| Metodo | Descrizione |
|--------|-------------|
| `getCurrentWaypoint()` | Restituisce la posa (`Eigen::Isometry3d`) del waypoint corrente |
| `getCurrentWaypointInfo()` | Restituisce tutte le informazioni (`WaypointInfo`) del waypoint corrente |
| `getCurrentWaypointIndex()` | Restituisce l'indice del waypoint corrente |
| `advanceToNextWaypoint()` | Avanza manualmente al prossimo waypoint |
| `jumpToWaypoint(index)` | Salta direttamente a un waypoint specifico |
| `isAtFinalWaypoint()` | Verifica se il waypoint corrente è l'ultimo |
| `resetToStart()` | Torna al primo waypoint della lista |

### 3. Aggiornamento Posizione e Logica di Switch

| Metodo | Descrizione |
|--------|-------------|
| `updateCurrentPosition(current_pose)` | Aggiorna la posizione corrente del robot e verifica automaticamente se passare al prossimo waypoint |
| `getDistanceToCurrentWaypoint()` | Restituisce la distanza lineare (metri) dal waypoint corrente |
| `getAngularDistanceToCurrentWaypoint()` | Restituisce la distanza angolare (radianti) dal waypoint corrente |

### 4. Parametri Dinamici

| Metodo | Descrizione | Default |
|--------|-------------|---------|
| `setWaypointSwitchDistance(distance)` | Imposta la distanza di switch tra waypoint | Min: 0.001m |
| `getWaypointSwitchDistance()` | Ottiene la distanza di switch corrente | - |
| `setUseOrientationForSwitch(use)` | Abilita/disabilita il controllo dell'orientamento per lo switch | - |
| `getUseOrientationForSwitch()` | Verifica se l'orientamento è usato per lo switch | - |
| `setOrientationSwitchThreshold(threshold)` | Imposta la soglia angolare per lo switch | Min: 0.001 rad |
| `getOrientationSwitchThreshold()` | Ottiene la soglia angolare corrente | - |

### 5. Sistema di Callback

| Metodo | Descrizione |
|--------|-------------|
| `setWaypointReachedCallback(callback)` | Registra una callback invocata quando si raggiunge un waypoint. Firma: `void(std::size_t index, const Eigen::Isometry3d& pose)` |
| `setPathCompletedCallback(callback)` | Registra una callback invocata quando il percorso è completato (raggiunto ultimo waypoint). Firma: `void()` |

### 6. Reset

| Metodo | Descrizione |
|--------|-------------|
| `reset()` | Resetta lo stato interno: indice waypoint a 0, distanze a infinito/zero |

---

## Logica di Switch tra Waypoint

Il metodo `updateCurrentPosition()` implementa la logica automatica di transizione:

```
1. Calcola distanza lineare (posizione) dal waypoint corrente
2. Calcola distanza angolare (orientamento) dal waypoint corrente
3. Verifica condizioni di switch:
   - Distanza lineare < switch_distance
   - SE use_orientation_for_switch:
     - Distanza angolare < orientation_threshold
4. Se condizioni soddisfatte:
   - Se ultimo waypoint → invoca path_completed_callback (una sola volta)
   - Altrimenti → avanza al prossimo waypoint e invoca waypoint_reached_callback
```

### Parametri Personalizzati per Waypoint

Ogni `WaypointInfo` può definire parametri di switch personalizzati:
- `switch_distance`: Distanza di switch specifica per quel waypoint
- `orientation_threshold`: Soglia angolare specifica
- `use_orientation_for_switch`: Se usare l'orientamento per lo switch

Se non specificati (valore 0), vengono usati i parametri globali.

---

## Calcolo delle Distanze

### Distanza Lineare
```cpp
double computePositionDistance(a, b) {
  return (a.translation() - b.translation()).norm();
}
```

### Distanza Angolare
```cpp
double computeOrientationDistance(a, b) {
  // 1. Estrae quaternioni dalle pose
  // 2. Assicura il percorso più breve (dot product check)
  // 3. Calcola rotazione relativa: q_diff = q_a^-1 * q_b
  // 4. Estrae angolo: 2 * acos(|q_diff.w|)
  // Risultato in [0, π] radianti
}
```

---

## Thread Safety

Il componente utilizza mutex separati per proteggere diversi gruppi di dati:

| Mutex | Protezione |
|-------|------------|
| `waypoints_mutex_` | Lista waypoint e indice corrente |
| `state_mutex_` | Distanze calcolate (posizione e angolare) |
| `params_mutex_` | Parametri di configurazione |
| `callback_mutex_` | Funzioni callback |

---

## Esempio di Utilizzo

```cpp
#include "cartesian_velocity_controller/components/global_planner.hpp"

// Creazione
GlobalPlanner planner;

// Configurazione parametri
planner.setWaypointSwitchDistance(0.05);  // 5 cm
planner.setUseOrientationForSwitch(true);
planner.setOrientationSwitchThreshold(0.1);  // ~5.7 gradi

// Registrazione callback
planner.setWaypointReachedCallback([](size_t idx, const Eigen::Isometry3d& pose) {
  ROS_INFO("Raggiunto waypoint %zu", idx);
});

planner.setPathCompletedCallback([]() {
  ROS_INFO("Percorso completato!");
});

// Impostazione waypoint
std::vector<Eigen::Isometry3d> waypoints;
// ... popola waypoints ...
planner.setWaypoints(waypoints);

// Nel loop di controllo
void controlLoop(const Eigen::Isometry3d& current_pose) {
  // Aggiorna posizione e verifica switch automatico
  bool switched = planner.updateCurrentPosition(current_pose);
  
  // Ottieni target corrente
  Eigen::Isometry3d target = planner.getCurrentWaypoint();
  
  // ... usa target per il controllo ...
}
```

---

## Struttura Dati

### WaypointInfo
Struttura che contiene informazioni estese su un waypoint:
- `pose`: `Eigen::Isometry3d` - Posa del waypoint
- `switch_distance`: `double` - Distanza di switch personalizzata (0 = usa default)
- `orientation_threshold`: `double` - Soglia orientamento personalizzata (0 = usa default)
- `use_orientation_for_switch`: `bool` - Flag per uso orientamento

---

## Note Implementative

1. **Callback path completed**: Viene invocata una sola volta tramite una variabile statica `path_completed_called`
2. **Inserimento waypoint**: Se si inserisce un waypoint prima o all'indice corrente, l'indice viene incrementato per mantenere il riferimento allo stesso waypoint
3. **Rimozione waypoint**: Se l'indice corrente diventa invalido dopo la rimozione, viene aggiustato all'ultimo waypoint valido
4. **Valori minimi**: Le distanze e soglie hanno valori minimi per evitare comportamenti instabili (1mm per distanze, 0.001 rad per angoli)

