# Mitigazione della Discontinuità nella Velocità Repulsiva

Questo documento analizza le **cause delle discontinuità** nei comandi di velocità repulsiva generati dalla mappa 3D (EDT) e propone **strategie di mitigazione** per rendere il comando più fluido.

---

## Indice

1. [Analisi del Problema](#1-analisi-del-problema)
2. [Fonti di Discontinuità Identificate](#2-fonti-di-discontinuità-identificate)
3. [Strategie di Mitigazione](#3-strategie-di-mitigazione)
4. [Raccomandazioni e Priorità](#4-raccomandazioni-e-priorità)
5. [Parametri da Tuning](#5-parametri-da-tuning)

---

## 1. Analisi del Problema

### 1.1 Architettura Attuale

Il flusso attuale della velocità repulsiva è:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         LOOP DI AGGIORNAMENTO MAPPA                      │
│                              (Thread separato)                           │
└─────────────────────────────────────────────────────────────────────────┘
                                     ↓
     ┌─────────────────────────────────────────────────────────────────┐
     │  Map3DManager::updateOnce()  (@ 15 Hz, configurabile)           │
     │  1. Leggi sfere dalla PlanningScene                             │
     │  2. Voxelizza + inflazione                                      │
     │  3. Calcola EDT (Euclidean Distance Transform)                  │
     │  4. Swap double-buffer (atomico)                                │
     └─────────────────────────────────────────────────────────────────┘
                                     ↓
┌─────────────────────────────────────────────────────────────────────────┐
│                         LOOP DI CONTROLLO                                │
│                              (100 Hz)                                    │
└─────────────────────────────────────────────────────────────────────────┘
                                     ↓
     ┌─────────────────────────────────────────────────────────────────┐
     │  RepulsionDataManager::getRepulsionData()                       │
     │  - Per ogni POI: queryWorld() → (distance, gradient)            │
     │  - Costruisce ObstacleInfo / LinkPOI                            │
     └─────────────────────────────────────────────────────────────────┘
                                     ↓
     ┌─────────────────────────────────────────────────────────────────┐
     │  LocalPlanner::compute()                                        │
     │  - computeRepulsiveVelocityFromObstacle()                       │
     │  - computeRepulsiveLinkJointVelocity()                          │
     │  - Combina con velocità attrattiva                              │
     └─────────────────────────────────────────────────────────────────┘
                                     ↓
     ┌─────────────────────────────────────────────────────────────────┐
     │  Output: v_combined → Motion Generator → IK → Joint Command     │
     └─────────────────────────────────────────────────────────────────┘
```

### 1.2 Sintomi Osservati

- **Jerk** (scatti) nel movimento quando la mappa si aggiorna
- Variazioni improvvise di direzione della repulsione
- Possibili oscillazioni quando il robot è vicino al confine dell'influence zone

---

## 2. Fonti di Discontinuità Identificate

### 2.1 Asincronia Mappa/Controllo (CRITICO)

| Aspetto | Dettaglio |
|---------|-----------|
| **Frequenza Mappa** | 15 Hz (configurabile via `map3d/update_rate_hz`) |
| **Frequenza Controllo** | 100 Hz |
| **Rapporto** | ~6.7 cicli di controllo per ogni aggiornamento mappa |

**Problema**: Il double-buffer swap avviene istantaneamente (`front_index_.store(back, std::memory_order_release)`), causando un cambio discreto nei valori di distanza/gradiente tra un ciclo di controllo e il successivo.

**Esempio numerico**:
- Ciclo N: `distance = 0.35 m`, `gradient = (0.8, 0.6, 0)`
- Ciclo N+1 (dopo swap): `distance = 0.33 m`, `gradient = (0.75, 0.65, 0.1)`
- Questo salto discreto si traduce in un **salto nella velocità repulsiva**.

### 2.2 Discretizzazione della Mappa EDT

```
        resolution = 0.03 m (3 cm)
        
        ┌───┬───┬───┐
        │0.03│0.06│0.09│  ← Distanze discrete a step di ~resolution
        ├───┼───┼───┤
        │0.06│0.09│0.12│
        └───┴───┴───┘
```

Anche con interpolazione trilineare (`getDistanceInterpolated`), i valori di distanza possono avere variazioni discrete quando il POI attraversa i confini dei voxel.

### 2.3 Gradiente Rumoroso Vicino agli Ostacoli

Il gradiente calcolato dalla EDT diventa **degenerato o rumoroso** vicino agli ostacoli (quando `distance < resolution`):

- Il gradiente può oscillare tra voxel adiacenti
- La normalizzazione di gradienti piccoli amplifica il rumore
- Il fallback al "last valid gradient" può causare salti quando si esce/entra nella zona di clamping

### 2.4 Profilo di Velocità Non-Smooth ai Boundary

La formula attuale per la velocità repulsiva è:

$$
v_{rep} = v_{max} \cdot \left(\frac{d_{infl} - d}{d_{infl} - d_{min}}\right)^2
$$

Questo profilo è **C¹ continuo** (derivata prima continua) ma **non C² continuo**:

- A `d = d_infl`: passaggio brusco da `v = 0` a ramping
- A `d = d_min`: saturazione a `v_max`

### 2.5 Transizione On/Off della Repulsione

Quando un POI entra/esce dalla zona di influenza (`d < influence_distance`), la velocità repulsiva passa istantaneamente da 0 a un valore finito (o viceversa).

### 2.6 Variazioni di Postura del Robot

Durante il movimento, la posizione dei POI varia anche se l'ostacolo è fermo. Questo causa variazioni nelle query alla mappa che si propagano al comando.

---

## 3. Strategie di Mitigazione

### 3.1 Filtro Passa-Basso sulla Velocità Repulsiva (RACCOMANDATO #1)

**Concetto**: Applicare un filtro EMA (Exponential Moving Average) alla velocità repulsiva calcolata prima di combinarla con la velocità attrattiva.

**Implementazione suggerita** in `LocalPlanner`:

```cpp
// Nuovo membro in LocalPlanner
Eigen::Vector3d v_repulsive_filtered_{Eigen::Vector3d::Zero()};
double repulsive_filter_tau_{0.05};  // Costante temporale [s]

// In compute():
Eigen::Vector3d v_repulsive_obstacle = computeRepulsiveVelocityTotal(...);

// Filtro EMA
double alpha = dt / (repulsive_filter_tau_ + dt);
v_repulsive_filtered_ = alpha * v_repulsive_obstacle + (1.0 - alpha) * v_repulsive_filtered_;

// Usa v_repulsive_filtered_ invece di v_repulsive_obstacle
output.repulsive_obstacle_linear = v_repulsive_filtered_ * k_rep;
```

**Pro**:
- Semplice da implementare
- Molto efficace per smorzare salti discreti
- Parametro `tau` facilmente tunabile via dynamic_reconfigure

**Contro**:
- Introduce un leggero ritardo nella risposta
- Non risolve la causa root ma tratta i sintomi

**Parametro raccomandato**: `tau = 0.03 - 0.08 s` (trade-off reattività/smoothness)

---

### 3.2 Interpolazione Temporale tra Buffer della Mappa (RACCOMANDATO #2)

**Concetto**: Invece di uno swap istantaneo, interpolare gradualmente tra la mappa "vecchia" e quella "nuova".

**Implementazione suggerita** in `Map3DManager`:

```cpp
// Nuovi membri
std::atomic<double> blend_alpha_{1.0};  // 0 = old, 1 = new
double blend_duration_{0.1};            // durata transizione [s]

// In updateOnce(), dopo il calcolo della nuova mappa:
blend_alpha_.store(0.0);  // Inizia transizione

// In queryWorld():
const double alpha = blend_alpha_.load();
const double d_old = grids_[1 - front_idx].getDistanceInterpolated(p_map, inside_old);
const double d_new = grids_[front_idx].getDistanceInterpolated(p_map, inside_new);
const double d = alpha * d_new + (1.0 - alpha) * d_old;
// Simile per il gradiente

// In un timer o nel loop del controller:
// blend_alpha_ += dt / blend_duration_;  // Clamp a [0, 1]
```

**Pro**:
- Transizione smooth tra aggiornamenti della mappa
- Mantiene la coerenza temporale

**Contro**:
- Complessità implementativa maggiore
- Richiede mantenere il buffer precedente valido
- Gradiente interpolato può essere meno preciso

---

### 3.3 Rate-Limiter sulla Variazione di Velocità (Jerk Limit)

**Concetto**: Limitare la velocità di variazione (derivata) del comando repulsivo.

**Implementazione**:

```cpp
// In LocalPlanner
Eigen::Vector3d v_repulsive_prev_{Eigen::Vector3d::Zero()};
double max_repulsive_jerk_{5.0};  // m/s² - massima variazione per ciclo

// In compute():
Eigen::Vector3d dv = v_repulsive_raw - v_repulsive_prev_;
double dv_norm = dv.norm();
double max_dv = max_repulsive_jerk_ * dt;

if (dv_norm > max_dv) {
    dv = dv.normalized() * max_dv;
}

v_repulsive_out = v_repulsive_prev_ + dv;
v_repulsive_prev_ = v_repulsive_out;
```

**Pro**:
- Controllo diretto sulla "smoothness" del comando
- Può essere combinato con il filtro EMA

**Contro**:
- Rischio di ritardo eccessivo in situazioni critiche (ostacolo in avvicinamento rapido)

---

### 3.4 Profilo di Velocità C² Continuo (Smoothstep/Hermite)

**Concetto**: Sostituire il profilo quadratico con una funzione che garantisce C² continuità ai bordi.

**Formula Smoothstep** (C¹):
$$
r = \frac{d_{infl} - d}{d_{infl} - d_{min}}, \quad r \in [0, 1]
$$
$$
v_{rep} = v_{max} \cdot (3r^2 - 2r^3)
$$

**Formula Smootherstep** (C²):
$$
v_{rep} = v_{max} \cdot (6r^5 - 15r^4 + 10r^3)
$$

```cpp
double smootherstep(double r) {
    r = std::clamp(r, 0.0, 1.0);
    return r * r * r * (r * (r * 6.0 - 15.0) + 10.0);
}
```

**Pro**:
- Derivata nulla all'inizio e alla fine dell'influence zone
- Transizione più morbida

**Contro**:
- Risposta leggermente più lenta nella zona centrale
- Modifica comportamento esistente

---

### 3.5 Aumento della Frequenza di Aggiornamento della Mappa

**Concetto**: Ridurre il "salto" tra aggiornamenti aumentando la frequenza.

**Configurazione** in `controller_params.yaml`:

```yaml
map3d:
  update_rate_hz: 30.0   # Aumentato da 15 Hz
```

**Pro**:
- Transizioni più piccole per ogni update
- Migliore tracking in tempo reale

**Contro**:
- **Maggiore carico CPU** (EDT è O(n³) per griglia n×n×n)
- Non risolve il problema della discretizzazione

---

### 3.6 Stabilizzazione del Gradiente (già parzialmente implementato)

**Stato attuale**: `RepulsionDataManager` usa `last_valid_gradients_world_` come fallback.

**Miglioramento proposto**: Filtrare anche il gradiente con EMA independente:

```cpp
// In RepulsionDataManager
std::map<std::string, Eigen::Vector3d> smoothed_gradients_;
double gradient_filter_alpha_{0.3};

// Dopo ogni query valida:
Eigen::Vector3d g_raw = q.gradient.normalized();
auto& g_smooth = smoothed_gradients_[poi_name];
g_smooth = gradient_filter_alpha_ * g_raw + (1.0 - gradient_filter_alpha_) * g_smooth;
grad = g_smooth.normalized();
```

---

### 3.7 Predictive Query (Compensazione Latenza Mappa)

**Concetto**: Predire la posizione del POI al prossimo update della mappa e usare quella per la query.

**Implementazione**:

```cpp
// Stima della latenza mappa
double map_latency = 1.0 / update_rate_hz;  // ~0.067s @ 15Hz

// Predizione posizione POI
Eigen::Vector3d poi_velocity = (position_world - previous_position_) / dt;
Eigen::Vector3d predicted_position = position_world + poi_velocity * map_latency;

// Query alla posizione predetta
QueryResult q = map3d_manager_->queryWorld(predicted_position, global_frame);
```

**Pro**:
- Compensa il ritardo intrinseco della mappa
- Migliore reattività

**Contro**:
- Errori di predizione possono causare instabilità
- Più complesso da implementare correttamente

---

## 4. Raccomandazioni e Priorità

### Priorità ALTA (implementare subito)

| # | Strategia | Impatto | Complessità | Note |
|---|-----------|---------|-------------|------|
| 1 | **Filtro EMA sulla velocità repulsiva** | ★★★★★ | Bassa | Aggiungere ~20 righe in `LocalPlanner` |
| 2 | **Filtro EMA sul gradiente** | ★★★★☆ | Bassa | Modificare `RepulsionDataManager` |

### Priorità MEDIA (secondo step)

| # | Strategia | Impatto | Complessità | Note |
|---|-----------|---------|-------------|------|
| 3 | Rate-limiter (jerk limit) | ★★★☆☆ | Media | Complementare al filtro |
| 4 | Profilo smootherstep | ★★★☆☆ | Bassa | Cambia comportamento esistente |
| 5 | Aumento frequenza mappa (25-30 Hz) | ★★★☆☆ | Nulla | Solo tuning parametri |

### Priorità BASSA (ottimizzazione fine)

| # | Strategia | Impatto | Complessità | Note |
|---|-----------|---------|-------------|------|
| 6 | Interpolazione temporale buffer | ★★★★☆ | Alta | Richiede refactoring mappa |
| 7 | Predictive query | ★★☆☆☆ | Media | Rischio instabilità |

---

## 5. Parametri da Tuning

### 5.1 Nuovi Parametri Suggeriti (da aggiungere)

```yaml
local_planner:
  # Smoothing della velocità repulsiva
  repulsive_velocity_filter_tau: 0.05   # [s] EMA time constant (0 = disable)
  repulsive_max_jerk: 10.0              # [m/s²] max variation rate (0 = disable)
  repulsive_velocity_mode: "QUADRATIC"  # LINEAR, QUADRATIC, SMOOTHSTEP, SMOOTHERSTEP

repulsion:
  # Smoothing del gradiente
  gradient_filter_alpha: 0.3            # EMA blend factor (0 = no filter, 1 = full filter)
```

### 5.2 Parametri Esistenti da Considerare

| Parametro | Default | Raccomandato | Effetto |
|-----------|---------|--------------|---------|
| `map3d/update_rate_hz` | 15 | 20-30 | Più alto = meno discreto |
| `map3d/resolution` | 0.03 | 0.02-0.03 | Più fine = gradiente più smooth |
| `map3d/gradient_clamp_distance` | 0.02 | 0.03-0.05 | Più alto = fallback più frequente |
| `local_planner/influence_distance` | 0.8 | 0.5-1.0 | Zona di transizione più ampia |
| `local_planner/min_safe_distance` | 0.1 | 0.05-0.15 | Saturazione più graduale |

### 5.3 Trade-off Reattività vs Smoothness

```
        Più Reattivo                              Più Smooth
        ◄────────────────────────────────────────────────►
        
        tau = 0.01s                              tau = 0.10s
        jerk = 20 m/s²                           jerk = 2 m/s²
        update_rate = 50 Hz                      update_rate = 10 Hz
        
        PRO: Risposta rapida                     PRO: Movimento fluido
        CONTRO: Jitter visibile                  CONTRO: Ritardo risposta
```

---

## 6. Esempio di Implementazione Rapida

### Modifica minimale in `LocalPlanner` (30 minuti)

```cpp
// local_planner.hpp - Nuovi membri privati
Eigen::Vector3d v_repulsive_filtered_{Eigen::Vector3d::Zero()};
Eigen::Vector3d v_repulsive_links_filtered_{Eigen::Vector3d::Zero()};
double repulsive_filter_tau_{0.05};  // Configurabile

// local_planner.cpp - In compute()
// Dopo: Eigen::Vector3d v_repulsive_obstacle = computeRepulsiveVelocityTotal(...);
// Aggiungere:
{
    const double alpha = (repulsive_filter_tau_ > 1e-6) 
        ? std::clamp(dt / (repulsive_filter_tau_ + dt), 0.0, 1.0)
        : 1.0;
    v_repulsive_filtered_ = alpha * v_repulsive_obstacle 
                          + (1.0 - alpha) * v_repulsive_filtered_;
    v_repulsive_obstacle = v_repulsive_filtered_;
}
```

Questo singolo cambiamento dovrebbe mitigare significativamente il problema.

---

## 7. Conclusioni

La **discontinuità nella velocità repulsiva** è principalmente causata da:

1. **Asincronia** tra update della mappa (15 Hz) e controllo (100 Hz)
2. **Discretizzazione** della griglia voxel
3. **Gradiente rumoroso** vicino agli ostacoli

Le **soluzioni più efficaci** e a basso costo sono:

1. ✅ **Filtro EMA sulla velocità repulsiva** (impatto immediato)
2. ✅ **Filtro EMA sul gradiente** (complementare)
3. ⚠️ Aumentare la frequenza mappa a 25-30 Hz (se CPU lo permette)

Queste modifiche dovrebbero ridurre significativamente il jitter percepito senza sacrificare la sicurezza del sistema di obstacle avoidance.

---

*Documento creato il 2026-01-23 - cartesian_velocity_controller*
