# Uso dei nuovi parametri: `controller_joint_limits` + `elbow_injection`

Questo documento spiega **come configurare** e **cosa fanno** i parametri aggiunti per:

- **limiti giunti “controller-only”** (reachability IK + guardrail runtime)
- **elbow velocity injection** (anti-singolarità gomito esteso), filtrata jerk/acc-safe

I parametri sono stati aggiunti/attivati in questi file:

- `cartesian_velocity_controller/config/controller_joint_limits.yaml` (**nuovo**)
- `cartesian_velocity_controller/config/controller_params.yaml` (sezione **nuova** `elbow_injection`)
- `cartesian_velocity_controller/launch/cartesian_velocity_controller.launch` (carica anche `controller_joint_limits.yaml`)

## Principi importanti (prima di configurare)

- **Unità**: tutti i limiti/zone per i giunti sono in **radianti** (rad). Le velocità sono in **rad/s**.
- **Nomi giunti**: i nomi sotto `controller_joint_limits.limits` devono combaciare con i joint names del gruppo MoveIt/RobotModel del controller (di default `group_name: "manipulator"`).
- **Scope**: questi limiti sono **solo lato controller**. Non modificano URDF o MoveIt.
- **Filtro jerk/acc-safe**: se abiliti guardrail e/o elbow injection, il controller **forza ON** `joint_velocity_filter/enabled` (con warning), perché la specifica richiede comportamento continuo (no step).

---

## 1) `controller_joint_limits.yaml`: limiti controller-only + guardrail runtime

File: `cartesian_velocity_controller/config/controller_joint_limits.yaml`

### 1.1 Struttura dei parametri

```yaml
controller_joint_limits:
  enabled: true

  limits:
    <joint_name>:
      enabled: true
      min: <rad>
      max: <rad>

      runtime_guard:
        enabled: true
        soft_zone: <rad>
        margin: <rad>
        reentry_velocity: <rad/s>
```

### 1.2 `controller_joint_limits.enabled`

- **true**: abilita l’uso dei limiti definiti sotto `limits`.
- **false**: ignora completamente questo file (nessun check IK, nessun guardrail runtime).

### 1.3 `controller_joint_limits.limits.<joint>.{enabled,min,max}`

Per ogni giunto elencato:

- **enabled**:
  - `true`: il giunto entra nel sistema di limiti controller-only.
  - `false`: il giunto viene ignorato (anche se presente).
- **min/max**:
  - definiscono la finestra valida $[min, max]$ per quel giunto, in **rad**.

#### Effetto sui target/waypoint (Reachability IK)

Quando `global_planner/reachability_check_enabled` è `true` (oppure `block_unreachable_targets: true` legacy):

- il controller calcola IK per target/waypoint
- la soluzione IK viene accettata **solo se** rispetta i limiti controller-only dei giunti con `enabled: true`
- se non esiste una soluzione IK che rispetta i limiti → il target/waypoint viene **rifiutato** (log “REJECTED”)

Nota: la validazione usa una **margin** (vedi sotto) come tolleranza numerica: accetta solo se $q \in [min+margin, max-margin]$.

### 1.4 `runtime_guard` (guardrail durante l’esecuzione)

Abilitando `runtime_guard.enabled: true` su un giunto, il controller modifica il comando $\dot q$ **in tempo reale** per evitare di andare verso il limite.

Per ogni giunto:

- **soft_zone** (rad)
  - zona “morbida” vicino al limite: quando la distanza dal limite (nella direzione del moto) è `< soft_zone`, il comando verso il limite viene **scalato in modo continuo** (smoothstep).
  - più grande = frena prima; più piccolo = frena più tardi.
- **margin** (rad)
  - margine “hard”: se sei dentro `margin` e stai andando verso il limite, viene forzato un comportamento di contenimento (vedi reentry).
  - viene anche usato come “tolleranza” nella validazione IK (target/waypoint).
- **reentry_velocity** (rad/s)
  - se sei dentro `margin` e stai andando verso il limite, il controller forza una piccola velocità di rientro verso l’interno:
    - vicino al **max** → impone una velocità negativa
    - vicino al **min** → impone una velocità positiva
  - se `reentry_velocity: 0.0`, dentro `margin` viene forzato `0.0` verso il limite (stop) invece del rientro attivo.

#### Interazione con gli altri limiter

Ordine (semplificato) nel loop:

1. Jacobian solve → `joint_velocities`
2. **(nuovo)** elbow injection (se abilitata)
3. **(nuovo)** runtime guardrail su limiti posizione
4. `JointVelocityFilter` (jerk/acc-safe)
5. `JointSafetyLimiter` (scaling finale hard su vel/acc)

---

## 2) `controller_params.yaml`: `elbow_injection` (anti-singolarità gomito esteso)

File: `cartesian_velocity_controller/config/controller_params.yaml`

### 2.1 Parametri disponibili

```yaml
elbow_injection:
  enabled: false

  elbow_joint_name: "elbow_joint"
  elbow_index: 2

  critical_zone: 0.40
  target_near_limit_threshold: 0.25

  k: 1.0
  max_push_velocity: 0.20
```

### 2.2 Cosa fa (in breve)

Quando il gomito è vicino al suo **limite MAX controller-only** (zona di singolarità), il controller aggiunge un bias $\dot q_{bias}$ (rad/s) che spinge il gomito **via dal MAX** (verso una configurazione più flessa).

Questo bias è:

- **continuo** (smoothstep su distanza dal limite)
- **filtrato jerk/acc-safe** perché viene inserito **prima** del `JointVelocityFilter`
- **gated** usando la soluzione IK del target/waypoint:
  - **senza target** → gate = 1 (injection ON)
  - **con target**:
    - se l’IK del target “richiede” gomito vicino al MAX → gate = 0 (injection OFF)
    - altrimenti gate = 1

### 2.3 Requisiti per abilitarla davvero

Perché `elbow_injection` funzioni, il controller deve conoscere $q_{max}$ del gomito da `controller_joint_limits`.

Quindi serve:

- `controller_joint_limits.enabled: true`
- una entry in `controller_joint_limits.limits.<nome_gomito>.enabled: true` con `max: ...`
- `elbow_injection.enabled: true`

Se `elbow_injection.enabled` è `true` ma il gomito o `qmax` non sono risolvibili dai limiti controller-only, l’injection viene **disabilitata automaticamente** con warning (per evitare comportamento ambiguo).

### 2.4 `elbow_joint_name` vs `elbow_index`

- **elbow_joint_name** (consigliato):
  - più robusto rispetto a cambi ordine giunti (deriva l’indice dal mapping dei joint del gruppo).
- **elbow_index**:
  - fallback se il nome non è presente/non trovato.

### 2.5 `critical_zone` (rad)

Definisce quanto “lontano” dal MAX considerare la zona critica:

- se $q_{max} - q_{elbow} \ge critical\_zone$ → injection ~0
- se $q_{max} - q_{elbow} \to 0$ → injection cresce fino a `max_push_velocity` (saturata)

### 2.6 `target_near_limit_threshold` (rad)

Serve per **spegnere** l’injection quando il target ha davvero bisogno del gomito esteso:

- calcolo: $d_{target} = q_{max} - q^*_{elbow,target}$
- se $d_{target} < target\_near\_limit\_threshold$ → gate = 0
- altrimenti gate = 1

### 2.7 `k` e `max_push_velocity`

- **k**:
  - gain della spinta (moltiplica il profilo $w$).
- **max_push_velocity** (rad/s):
  - saturazione del bias aggiunto al gomito.

---

## 3) Parametri collegati (consigli pratici)

### 3.1 `joint_velocity_filter` (diventa “obbligatorio” se abiliti guardrail/injection)

File: `controller_params.yaml` → sezione `joint_velocity_filter`

Se abiliti:

- almeno un `controller_joint_limits.limits.<joint>.runtime_guard.enabled: true` **oppure**
- `elbow_injection.enabled: true`

allora il controller forza `joint_velocity_filter/enabled: true` (se era false).

Consiglio:

- imposta `joint_velocity_filter/tau` e i limiti `max_joint_acceleration` / `max_joint_jerk` in modo coerente con la tua dinamica, perché ora questo stadio diventa parte integrante del comportamento “continuo”.

### 3.2 Reachability: `global_planner/reachability_check_enabled` e legacy `block_unreachable_targets`

File: `controller_params.yaml`

- **Nuovo parametro**: `global_planner/reachability_check_enabled`
- **Legacy**: `block_unreachable_targets`

Policy implementata:

- se `global_planner/reachability_check_enabled` non è presente ma c’è `block_unreachable_targets`, viene usato quello
- se sono presenti entrambi, viene usato **il nuovo** e viene loggato un warning

---

## 4) Esempi di configurazione

### 4.1 Esempio minimo: rifiuta target fuori limite (reachability)

`controller_joint_limits.yaml`:

```yaml
controller_joint_limits:
  enabled: true
  limits:
    elbow_joint:
      enabled: true
      min: -1.0
      max:  1.0
      runtime_guard:
        enabled: false
        soft_zone: 0.30
        margin: 0.01
        reentry_velocity: 0.05
```

`controller_params.yaml`:

```yaml
global_planner:
  reachability_check_enabled: true
```

Atteso: target che richiede gomito oltre $[-1, 1]$ viene **REJECTED** dal controller.

### 4.2 Esempio: runtime guardrail attivo su gomito

```yaml
controller_joint_limits:
  enabled: true
  limits:
    elbow_joint:
      enabled: true
      min: -1.0
      max:  1.0
      runtime_guard:
        enabled: true
        soft_zone: 0.30
        margin: 0.02
        reentry_velocity: 0.05
```

Atteso: avvicinandosi al limite, la velocità verso il limite viene frenata in modo continuo; dentro `margin` viene forzato un piccolo rientro.

### 4.3 Esempio: elbow injection con gating sul target

`controller_joint_limits.yaml` (serve `max`):

```yaml
controller_joint_limits:
  enabled: true
  limits:
    elbow_joint:
      enabled: true
      min: -6.28
      max:  1.57
      runtime_guard:
        enabled: true
        soft_zone: 0.30
        margin: 0.01
        reentry_velocity: 0.05
```

`controller_params.yaml`:

```yaml
elbow_injection:
  enabled: true
  elbow_joint_name: "elbow_joint"
  critical_zone: 0.40
  target_near_limit_threshold: 0.25
  k: 1.0
  max_push_velocity: 0.20
```

Atteso:

- senza target (controller in “hold”) → se gomito entra in zona critica, appare una spinta filtrata che lo allontana dal limite
- con target che richiede gomito molto vicino al MAX → injection gate=0 (non disturba il target)

