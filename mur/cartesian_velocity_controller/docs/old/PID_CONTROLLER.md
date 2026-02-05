# PID Controller - Documentazione

## Panoramica

Il `PIDController` è un'implementazione generica di un controllore **PID (Proporzionale-Integrale-Derivativo)** N-dimensionale con caratteristiche avanzate:

- Supporto multi-dimensionale (vettoriale)
- Termine feedforward
- Saturazione dell'output
- Anti-windup dinamico
- Filtro passa-basso sulla derivata
- Thread-safety

---

## Struttura di Configurazione

```cpp
struct PIDConfig
{
  double kp{1.0};                    // Guadagno proporzionale
  double ki{0.0};                    // Guadagno integrale
  double kd{0.0};                    // Guadagno derivativo
  double kff{0.0};                   // Guadagno feedforward [0..1]
  double output_limit{1.0};          // Limite saturazione (simmetrico)
  double derivative_filter_tau{0.1}; // Costante tempo filtro derivata (s)
  bool enabled{true};                // Abilita/disabilita controller
};
```

---

## Equazione del Controllore

L'output del controllore è calcolato come:

```
output = P + I + D + FF
```

Dove:
- **P** = `kp × error` (termine proporzionale)
- **I** = `ki × ∫error·dt` (termine integrale)
- **D** = `kd × d(error)/dt` (termine derivativo filtrato)
- **FF** = `kff × feedforward` (termine feedforward)

---

## Componenti del Controllore

### 1. Termine Proporzionale (P)

```cpp
Eigen::VectorXd p_term = config.kp * error;
```

- Risposta **immediata** e **proporzionale** all'errore corrente
- Più alto è `kp`, più aggressiva è la correzione
- Non elimina l'errore a regime (offset stazionario)

### 2. Termine Integrale (I) con Anti-Windup

```cpp
integral_ += error * dt;
i_term = config.ki * integral_;
applyAntiWindup(i_term, p_magnitude, config.output_limit);
```

**Funzione:**
- Accumula l'errore nel tempo
- Elimina l'errore stazionario (offset)
- Reagisce lentamente ma garantisce errore zero a regime

**Anti-Windup Dinamico:**

Il problema del "windup" si verifica quando l'output è saturato ma l'integrale continua a crescere. La soluzione implementata è un **anti-windup dinamico**:

```cpp
void applyAntiWindup(Eigen::VectorXd& integral_output,
                     double p_magnitude,
                     double output_limit)
{
  // Spazio disponibile per l'integrale = limite - |P|
  double integral_headroom = std::max(0.0, output_limit - p_magnitude);
  double i_magnitude = integral_output.norm();

  if (i_magnitude > integral_headroom && i_magnitude > kEpsilon)
  {
    integral_output *= (integral_headroom / i_magnitude);
  }
}
```

**Come funziona:**
1. Calcola lo "spazio disponibile" = `output_limit - |P|`
2. Se `|I| > spazio disponibile`, scala I per rientrare nel limite
3. Questo impedisce all'integrale di contribuire quando P già satura l'output

**Back-calculation:**
```cpp
integral_ = i_term / config.ki;
```
Dopo il clamping, l'accumulatore integrale viene aggiornato per riflettere il valore effettivamente utilizzato.

### 3. Termine Derivativo (D) con Filtro

```cpp
if (config.kd > kEpsilon && has_prev_error_)
{
  Eigen::VectorXd derivative_raw = (error - prev_error_) / dt;

  // Filtro passa-basso: alpha = dt / (tau + dt)
  double alpha = dt / (config.derivative_filter_tau + dt);
  derivative_filtered_ = alpha * derivative_raw + (1.0 - alpha) * derivative_filtered_;

  d_term = config.kd * derivative_filtered_;
}
```

**Funzione:**
- Anticipa le variazioni dell'errore
- Smorza le oscillazioni
- Migliora la stabilità

**Filtro Passa-Basso:**

La derivata pura amplifica il rumore. Per evitarlo, viene applicato un filtro passa-basso del primo ordine:

```
alpha = dt / (tau + dt)
y[k] = alpha × x[k] + (1 - alpha) × y[k-1]
```

Dove:
- `tau` = costante di tempo del filtro (`derivative_filter_tau`)
- Più alto è `tau`, più smussata è la derivata (ma più lenta)
- Con `tau = 0`, il filtro è disabilitato

### 4. Termine Feedforward (FF)

```cpp
if (config.kff > kEpsilon && feedforward.size() == dimensions_)
{
  ff_term = config.kff * feedforward;
}
```

**Funzione:**
- Aggiunge un contributo diretto dalla traiettoria desiderata
- Migliora il tracking dinamico
- Riduce il carico sul feedback loop

**Esempio d'uso:**
Se stai seguendo una traiettoria con velocità nota, puoi passare la velocità desiderata come feedforward:
```cpp
Eigen::Vector3d feedforward = desired_velocity;
output = pid.compute(position_error, feedforward, dt);
```

---

## Saturazione dell'Output

```cpp
void saturate(Eigen::VectorXd& output, double limit) const
{
  double magnitude = output.norm();
  if (magnitude > limit && magnitude > kEpsilon)
  {
    output *= (limit / magnitude);
  }
}
```

- Limita la **norma** del vettore output
- Mantiene la **direzione** originale
- Simmetrico: `[-limit, +limit]`

---

## Thread Safety

Il controller è **thread-safe** grazie a due mutex separati:

```cpp
mutable std::mutex config_mutex_;  // Protegge la configurazione
mutable std::mutex state_mutex_;   // Protegge lo stato interno
```

- `config_mutex_`: protegge lettura/scrittura della configurazione
- `state_mutex_`: protegge l'integrale, la derivata e gli errori precedenti

Questo permette di modificare i parametri PID in runtime da un thread diverso.

---

## Diagramma di Flusso

```
                    ┌─────────────┐
                    │   error     │
                    └──────┬──────┘
                           │
           ┌───────────────┼───────────────┐
           │               │               │
           ▼               ▼               ▼
      ┌────────┐      ┌────────┐      ┌────────┐
      │  × kp  │      │ ∫·dt   │      │ d/dt   │
      │   (P)  │      │  × ki  │      │ filter │
      └────┬───┘      │  (I)   │      │  × kd  │
           │          └────┬───┘      │  (D)   │
           │               │          └────┬───┘
           │               ▼               │
           │       ┌──────────────┐        │
           │       │ Anti-Windup │        │
           │       └──────┬───────┘        │
           │              │               │
           │   ┌──────────┴─────────────┐ │
           │   │                        │ │
           ▼   ▼                        ▼ ▼
         ┌───────────────────────────────────┐
         │            P + I + D + FF         │
         └───────────────┬───────────────────┘
                         │        ▲
                         │        │
                         │   ┌────┴────┐
                         │   │   FF    │◄── feedforward × kff
                         │   └─────────┘
                         ▼
                  ┌─────────────┐
                  │  Saturate   │
                  └──────┬──────┘
                         │
                         ▼
                  ┌─────────────┐
                  │   output    │
                  └─────────────┘
```

---

## Esempio d'Uso

```cpp
#include "cartesian_velocity_controller/components/pid_controller.hpp"

// Crea un controller 3D (es. per posizione XYZ)
PIDController pid(3);

// Configura i parametri
PIDConfig config;
config.kp = 2.0;
config.ki = 0.1;
config.kd = 0.05;
config.kff = 0.5;
config.output_limit = 0.5;
config.derivative_filter_tau = 0.05;
pid.setConfig(config);

// Nel loop di controllo
void controlLoop(double dt)
{
  Eigen::Vector3d error = target_position - current_position;
  Eigen::Vector3d feedforward = desired_velocity;
  
  Eigen::Vector3d velocity_command = pid.compute(error, feedforward, dt);
  
  // Invia velocity_command al robot
}

// Reset quando cambia il target
void onNewTarget()
{
  pid.reset();  // Azzera integrale e derivata
}
```

---

## Metodi Pubblici

| Metodo | Descrizione |
|--------|-------------|
| `compute(error, feedforward, dt)` | Calcola output PID completo |
| `compute(error, dt)` | Calcola output senza feedforward |
| `reset()` | Azzera integrale e stato derivata |
| `resetIntegral()` | Azzera solo l'integrale |
| `setConfig(config)` | Imposta configurazione (thread-safe) |
| `getConfig()` | Ottiene configurazione corrente |
| `getLastError()` | Ultimo errore elaborato |
| `getIntegral()` | Valore corrente accumulatore integrale |
| `setIntegral(value)` | Imposta manualmente l'integrale |
| `setEnabled(bool)` | Abilita/disabilita controller |
| `isEnabled()` | Stato abilitazione |
| `dimensions()` | Numero di dimensioni |

---

## Tuning dei Parametri

### Metodo Empirico (Ziegler-Nichols semplificato)

1. **Imposta `ki = 0`, `kd = 0`**
2. **Aumenta `kp`** fino a ottenere oscillazioni stabili
3. **Aggiungi `kd`** (circa 0.1× `kp`) per smorzare le oscillazioni
4. **Aggiungi `ki`** (circa 0.01× `kp`) per eliminare l'offset

### Valori Tipici per Controllo Robot

| Applicazione | kp | ki | kd | tau |
|--------------|----|----|----|----|
| Posizione lenta | 1.0-2.0 | 0.05-0.1 | 0.02-0.1 | 0.05-0.1 |
| Posizione veloce | 3.0-5.0 | 0.1-0.3 | 0.1-0.3 | 0.02-0.05 |
| Orientamento | 2.0-4.0 | 0.1 | 0.05 | 0.05 |

---

## Note Implementative

1. **Epsilon (1e-10):** Usato per evitare divisioni per zero
2. **Primo ciclo:** La derivata è zero finché non c'è un errore precedente (`has_prev_error_`)
3. **Dimensionalità:** Tutti i vettori devono avere la stessa dimensione specificata nel costruttore
4. **Output nullo:** Se `enabled = false` o `dt ≤ 0`, l'output è sempre zero

