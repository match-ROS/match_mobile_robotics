# Estensione del Cartesian Velocity Controller a un Manipolatore Mobile

**Data:** 26 Gennaio 2026  
**Versione:** 1.0  
**Stato:** Documento Concettuale  

---

## 📋 Indice

1. [Introduzione](#1-introduzione)
2. [Architettura Attuale: Braccio Singolo](#2-architettura-attuale-braccio-singolo)
3. [Sfide dell'Estensione a Manipolatore Mobile](#3-sfide-dellestensione-a-manipolatore-mobile)
4. [Architettura Proposta per Manipolatore Mobile](#4-architettura-proposta-per-manipolatore-mobile)
5. [Gestione della Cinematica Composita](#5-gestione-della-cinematica-composita)
6. [Coordinamento Base-Braccio](#6-coordinamento-base-braccio)
7. [Strategie di Controllo](#7-strategie-di-controllo)
8. [Considerazioni su Frame e Trasformazioni](#8-considerazioni-su-frame-e-trasformazioni)
9. [Impatto sulla Mappa 3D e Obstacle Avoidance](#9-impatto-sulla-mappa-3d-e-obstacle-avoidance)
10. [Piano di Estensione Incrementale](#10-piano-di-estensione-incrementale)

---

## 1. Introduzione

### 1.1. Contesto

Il **Cartesian Velocity Controller** attuale è progettato per controllare un **braccio robotico fisso** (es. UR10e) con base statica. L'obiettivo è estendere questa architettura per supportare un **manipolatore mobile**, ovvero un sistema composto da:

- **Base mobile**: Piattaforma con ruote (differenziale, omnidirezionale, o Ackermann)
- **Braccio manipolatore**: Montato sulla base mobile

### 1.2. Obiettivo del Documento

Questo documento descrive **concettualmente** le modifiche architetturali necessarie per estendere il controller, senza entrare nei dettagli implementativi. L'obiettivo è fornire una visione chiara del percorso di sviluppo.

### 1.3. Terminologia

| Termine | Descrizione |
|---------|-------------|
| **TCP** | Tool Center Point - Punto di riferimento dell'end-effector |
| **Base Mobile** | Piattaforma robotica con capacità di locomozione |
| **Mobile Manipulator** | Sistema composto da base mobile + braccio manipolatore |
| **Holonomic** | Base che può muoversi in qualsiasi direzione istantaneamente |
| **Non-holonomic** | Base con vincoli cinematici (es. auto, differenziale) |

---

## 2. Architettura Attuale: Braccio Singolo

### 2.1. Schema Attuale

L'architettura corrente gestisce **solo il braccio robotico**:

```
┌─────────────────────────────────────────────────────────────────────┐
│                ARCHITETTURA ATTUALE (BRACCIO SINGOLO)               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   World Frame (fisso)                                               │
│        │                                                            │
│        ▼                                                            │
│   ┌────────────────┐                                                │
│   │   base_link    │  ← FISSO nel mondo                             │
│   │   (UR10e base) │                                                │
│   └───────┬────────┘                                                │
│           │                                                         │
│           ▼ Catena cinematica (6 DOF)                               │
│   ┌────────────────┐                                                │
│   │   Joint 1-6    │  shoulder, elbow, wrist...                     │
│   └───────┬────────┘                                                │
│           │                                                         │
│           ▼                                                         │
│   ┌────────────────┐                                                │
│   │      TCP       │  End-Effector                                  │
│   └────────────────┘                                                │
│                                                                     │
│   CONTROLLO:                                                        │
│   • Input: Target Pose (in world o base_link)                       │
│   • Output: 6 velocità giunti [q̇₁, q̇₂, q̇₃, q̇₄, q̇₅, q̇₆]              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2. Flusso della Pipeline Attuale

```
Target Pose → GlobalPlanner → LocalPlanner → VelocityFilter → PID → Jacobian IK → Joint Velocities
                                   │
                                   ├─ Attractive Velocity (verso target)
                                   └─ Repulsive Velocity (evitamento ostacoli)
```

### 2.3. Limitazioni per un Manipolatore Mobile

1. **Workspace limitato**: Il braccio può raggiungere solo pose nel suo raggio d'azione
2. **Base statica assunta**: Lo Jacobiano considera solo i giunti del braccio
3. **Frame di riferimento fisso**: `base_link` non si muove nel mondo

---

## 3. Sfide dell'Estensione a Manipolatore Mobile

### 3.1. Nuovi Gradi di Libertà

Passando da braccio singolo a manipolatore mobile, introduciamo nuovi DOF:

| Configurazione | DOF Braccio | DOF Base | DOF Totali |
|----------------|-------------|----------|------------|
| **Braccio fisso** | 6 | 0 | 6 |
| **Base differenziale** | 6 | 2 (v, ω) | 8 |
| **Base omnidirezionale** | 6 | 3 (vx, vy, ω) | 9 |
| **Con piano verticale (ascensore)** | 6 | 3 + 1 | 10 |

### 3.2. Sfide Principali

```
┌─────────────────────────────────────────────────────────────────────┐
│                    SFIDE DEL MANIPOLATORE MOBILE                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1️⃣ CINEMATICA ESTESA                                              │
│     ┌────────────────────────────────────────────────────────────┐  │
│     │ • Lo Jacobiano deve includere i contributi della base      │  │
│     │ • Problema di ridondanza (più DOF che necessari)           │  │
│     │ • Gestione delle singolarità cambia                        │  │
│     └────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  2️⃣ COORDINAMENTO BASE-BRACCIO                                     │
│     ┌────────────────────────────────────────────────────────────┐  │
│     │ • Chi si muove: la base, il braccio, o entrambi?           │  │
│     │ • Come distribuire il "lavoro" tra i due sottosistemi?     │  │
│     │ • Dinamiche molto diverse (base lenta, braccio veloce)     │  │
│     └────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  3️⃣ VINCOLI NON-OLONOMI                                            │
│     ┌────────────────────────────────────────────────────────────┐  │
│     │ • Base differenziale: non può muoversi lateralmente        │  │
│     │ • Vincoli sulla velocità, non sulla posizione              │  │
│     │ • Pianificazione più complessa                             │  │
│     └────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  4️⃣ FRAME DI RIFERIMENTO DINAMICO                                  │
│     ┌────────────────────────────────────────────────────────────┐  │
│     │ • base_link ora si muove nel mondo                         │  │
│     │ • Trasformazioni TF più complesse                          │  │
│     │ • La mappa 3D si sposta con il robot                       │  │
│     └────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  5️⃣ OBSTACLE AVOIDANCE                                             │
│     ┌────────────────────────────────────────────────────────────┐  │
│     │ • La base stessa diventa un "POI" da proteggere            │  │
│     │ • Ostacoli a terra (solo per la base)                      │  │
│     │ • Volume di collisione della base vs braccio               │  │
│     └────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 4. Architettura Proposta per Manipolatore Mobile

### 4.1. Vista d'Insieme

L'architettura estesa mantiene la **struttura a pipeline**, ma introduce un livello aggiuntivo per gestire il coordinamento base-braccio:

```
┌─────────────────────────────────────────────────────────────────────┐
│              ARCHITETTURA ESTESA (MANIPOLATORE MOBILE)              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                     GLOBAL PLANNER                          │   │
│   │   (invariato: gestione waypoint e target)                   │   │
│   └──────────────────────────┬──────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                     LOCAL PLANNER                           │   │
│   │   (esteso: include velocità desiderata del TCP)             │   │
│   └──────────────────────────┬──────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│   ╔═════════════════════════════════════════════════════════════╗   │
│   ║           🆕 TASK ALLOCATION / COORDINATION                 ║   │
│   ╠═════════════════════════════════════════════════════════════╣   │
│   ║                                                             ║   │
│   ║   Decide COME raggiungere la velocità TCP desiderata:       ║   │
│   ║   • Solo braccio? Solo base? Combinazione?                  ║   │
│   ║                                                             ║   │
│   ║   Input:  v_tcp_desired (6D twist)                          ║   │
│   ║   Output: v_arm_task, v_base_task                           ║   │
│   ║                                                             ║   │
│   ╚═══════════════════════════════════════════════════════════════╝   │
│                              │                                      │
│            ┌─────────────────┼─────────────────┐                    │
│            ▼                                   ▼                    │
│   ┌────────────────────┐            ┌────────────────────┐          │
│   │   ARM CONTROLLER   │            │  BASE CONTROLLER   │          │
│   │                    │            │                    │          │
│   │  • Jacobian IK     │            │  • Cmd_vel         │          │
│   │  • Safety Limiter  │            │  • Non-holonomic   │          │
│   │  • 6 joint vel     │            │  • 2-3 DOF         │          │
│   └────────┬───────────┘            └────────┬───────────┘          │
│            │                                 │                      │
│            ▼                                 ▼                      │
│   ┌────────────────────┐            ┌────────────────────┐          │
│   │ /joint_velocities  │            │     /cmd_vel       │          │
│   └────────────────────┘            └────────────────────┘          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.2. Nuovo Componente: Task Allocation

Il cuore dell'estensione è il modulo di **Task Allocation**, che decide come distribuire il "compito" tra base e braccio:

```
┌─────────────────────────────────────────────────────────────────────┐
│                        TASK ALLOCATION                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   INPUT:                                                            │
│   • v_tcp_desired: Twist 6D desiderato del TCP                      │
│   • current_arm_config: Posizione corrente giunti braccio           │
│   • current_base_pose: Posizione corrente base                      │
│   • obstacles_info: Informazioni ostacoli                           │
│                                                                     │
│   DECISIONE BASATA SU:                                              │
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ 1. MANIPULABILITY INDEX                                     │   │
│   │    • Se il braccio è vicino a singolarità → usa la base     │   │
│   │    • Se il braccio ha buona manipolabilità → preferisci     │   │
│   │      movimento del braccio (più preciso)                    │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ 2. WORKSPACE LIMITS                                         │   │
│   │    • Se il target è fuori reach → muovi la base             │   │
│   │    • Se il target è nel "core" del workspace → solo braccio │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ 3. DYNAMICA / VELOCITÀ                                      │   │
│   │    • Movimenti fini e precisi → braccio                     │   │
│   │    • Grandi spostamenti → base                              │   │
│   │    • Task ad alta frequenza → solo braccio                  │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ 4. OBSTACLE CONSTRAINTS                                     │   │
│   │    • Se la base è bloccata da ostacoli → solo braccio       │   │
│   │    • Se il braccio è in zona pericolosa → muovi la base     │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   OUTPUT:                                                           │
│   • v_arm_contribution: Quanto del twist deve fare il braccio       │
│   • v_base_contribution: Quanto del twist deve fare la base         │
│   • α = arm_ratio ∈ [0, 1]: Peso relativo                           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 5. Gestione della Cinematica Composita

### 5.1. Jacobiano Esteso

Per un manipolatore mobile, il Jacobiano del TCP rispetto a tutti i DOF controllabili diventa:

```
┌─────────────────────────────────────────────────────────────────────┐
│                     JACOBIANO COMPOSITO                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   v_tcp = J_extended · q̇_extended                                   │
│                                                                     │
│   Dove:                                                             │
│   • v_tcp ∈ ℝ⁶: Twist del TCP (3 lineari + 3 angolari)              │
│   • q̇_extended ∈ ℝⁿ: Velocità di tutti i DOF                       │
│                                                                     │
│   Per una base differenziale + UR10e (6 DOF):                       │
│                                                                     │
│            ┌                                                    ┐   │
│            │  J_base (6×2)  │  J_arm (6×6)                      │   │
│   J_ext =  │                │                                   │   │
│            │  contributo    │  Jacobiano                        │   │
│            │  della base    │  standard UR10e                   │   │
│            └                                                    ┘   │
│                                                                     │
│   q̇_extended = [ v_base, ω_base, q̇₁, q̇₂, q̇₃, q̇₄, q̇₅, q̇₆ ]ᵀ         │
│                  └────┬────┘  └──────────┬──────────┘               │
│                   2 DOF base         6 DOF braccio                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.2. Calcolo del Jacobiano della Base

Il Jacobiano della base dipende dal tipo di base mobile:

```
┌─────────────────────────────────────────────────────────────────────┐
│                   JACOBIANO DELLA BASE                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   BASE DIFFERENZIALE (2 DOF: v_linear, ω_angular)                   │
│                                                                     │
│   L'effetto sul TCP dipende dalla posizione relativa base→TCP:      │
│                                                                     │
│   Sia p_tcp_base = posizione TCP nel frame base                     │
│   Sia R_base_world = rotazione base nel mondo                       │
│                                                                     │
│           ┌                 ┐                                       │
│           │  R · [1, 0, 0]ᵀ │                                       │
│   J_base =│  R · [0, 0, 0]ᵀ │   per movimento lineare              │
│           │  [0, 0, 1]ᵀ     │   per rotazione (attorno a Z)         │
│           │  + cross terms  │                                       │
│           └                 ┘                                       │
│                                                                     │
│   In pratica: quando la base si muove/ruota, il TCP si muove        │
│   di conseguenza, la relazione è geometrica.                        │
│                                                                     │
│   ─────────────────────────────────────────────────────────────     │
│                                                                     │
│   BASE OMNIDIREZIONALE (3 DOF: vx, vy, ω)                           │
│                                                                     │
│   Più semplice: ogni DOF della base ha un effetto "diretto"         │
│   sulla posizione del TCP.                                          │
│                                                                     │
│           ┌                     ┐                                   │
│           │  R · [1, 0, 0]ᵀ  vx │                                   │
│   J_base =│  R · [0, 1, 0]ᵀ  vy │                                   │
│           │  [0, 0, 1]ᵀ      ω  │                                   │
│           └                     ┘                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.3. Gestione della Ridondanza

Con più DOF del necessario (es. 8 DOF per 6 DOF richiesti), abbiamo ridondanza:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    GESTIONE RIDONDANZA                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   PROBLEMA:                                                         │
│   • 6 DOF richiesti (twist TCP)                                     │
│   • 8+ DOF disponibili (base + braccio)                             │
│   • Infinite soluzioni per lo stesso movimento TCP                  │
│                                                                     │
│   SOLUZIONE: Ottimizzazione con costi secondari                     │
│                                                                     │
│   q̇ = J⁺ · v_tcp + (I - J⁺J) · q̇_null                               │
│         └───┬───┘   └────────┬────────┘                             │
│      Soluzione         Proiezione nel                               │
│      minima norma      null-space                                   │
│                                                                     │
│   Il null-space può essere usato per:                               │
│                                                                     │
│   1. Mantenere una postura ottimale del braccio                     │
│   2. Evitare limiti giunti                                          │
│   3. Massimizzare manipolabilità                                    │
│   4. Minimizzare movimento della base (per stabilità)               │
│   5. Ottimizzare consumo energetico                                 │
│                                                                     │
│   ─────────────────────────────────────────────────────────────     │
│                                                                     │
│   ESEMPIO: Preferenza braccio vs base                               │
│                                                                     │
│   Con pesi diversi per i DOF:                                       │
│                                                                     │
│   W = diag(w_base, w_base, w_arm, w_arm, ...)                       │
│                                                                     │
│   • w_base alto → la base si muove poco                             │
│   • w_arm alto → il braccio si muove poco                           │
│                                                                     │
│   La pseudo-inversa pesata J_W⁺ = W⁻¹Jᵀ(JW⁻¹Jᵀ)⁻¹ produce          │
│   soluzioni che minimizzano ||Wq̇||²                                 │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 6. Coordinamento Base-Braccio

### 6.1. Modalità di Coordinamento

Esistono diverse strategie per decidere quando e come coinvolgere la base:

```
┌─────────────────────────────────────────────────────────────────────┐
│                  MODALITÀ DI COORDINAMENTO                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   🅰️ MODALITÀ "ARM FIRST" (conservativa)                            │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                                                             │   │
│   │   • Il braccio fa tutto il possibile                        │   │
│   │   • La base si muove SOLO se necessario:                    │   │
│   │     - Target fuori workspace                                │   │
│   │     - Braccio in singolarità                                │   │
│   │     - Limiti giunti                                         │   │
│   │                                                             │   │
│   │   PRO: Più preciso, meno vibrazioni, stabile                │   │
│   │   CONTRO: Può essere lento per grandi spostamenti           │   │
│   │                                                             │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   🅱️ MODALITÀ "COORDINATED" (simultanea)                            │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                                                             │   │
│   │   • Base e braccio si muovono insieme                       │   │
│   │   • Distribuzione basata su ottimizzazione                  │   │
│   │   • Tipicamente: spostamenti grandi → base                  │   │
│   │                   spostamenti fini → braccio                │   │
│   │                                                             │   │
│   │   PRO: Più veloce, workspace infinito                       │   │
│   │   CONTRO: Più complesso, possibili oscillazioni             │   │
│   │                                                             │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   🅲 MODALITÀ "BASE FIRST" (mobile priority)                        │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                                                             │   │
│   │   • La base si posiziona prima                              │   │
│   │   • Il braccio interviene per il fine-tuning                │   │
│   │   • Utile per task di navigazione + manipolazione           │   │
│   │                                                             │   │
│   │   PRO: Minimizza sforzo del braccio                         │   │
│   │   CONTRO: Più lento, richiede pianificazione base           │   │
│   │                                                             │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   🅳 MODALITÀ "DECOUPLED" (separata)                                 │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                                                             │   │
│   │   • Base e braccio controllati indipendentemente            │   │
│   │   • Utile per task di trasporto (base muove, braccio stabile)│   │
│   │                                                             │   │
│   │   PRO: Semplice, robusto                                    │   │
│   │   CONTRO: Non sfrutta la ridondanza                         │   │
│   │                                                             │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 6.2. Parametri di Decisione

Per implementare la task allocation, si possono usare diversi criteri:

```
┌─────────────────────────────────────────────────────────────────────┐
│                   CRITERI DI TASK ALLOCATION                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   1. MANIPULABILITY INDEX (µ)                                       │
│      ─────────────────────────                                      │
│      µ = √det(J·Jᵀ)                                                 │
│                                                                     │
│      • µ alto → braccio in buona posizione → preferisci braccio     │
│      • µ basso → vicino a singolarità → coinvolgi base              │
│                                                                     │
│      Se µ < µ_threshold:                                            │
│          α_base = (µ_threshold - µ) / µ_threshold                   │
│                                                                     │
│   ─────────────────────────────────────────────────────────────     │
│                                                                     │
│   2. DISTANCE TO WORKSPACE BOUNDARY                                 │
│      ────────────────────────────────                               │
│      d_ws = distanza del TCP dal limite del workspace               │
│                                                                     │
│      • d_ws grande → interno al workspace → solo braccio            │
│      • d_ws piccolo → near boundary → coinvolgi base                │
│                                                                     │
│   ─────────────────────────────────────────────────────────────     │
│                                                                     │
│   3. VELOCITY MAGNITUDE                                             │
│      ──────────────────────                                         │
│      ||v_tcp|| = norma della velocità richiesta                     │
│                                                                     │
│      • v alto → movimento grosso → base può contribuire             │
│      • v basso → movimento fine → preferisci braccio                │
│                                                                     │
│   ─────────────────────────────────────────────────────────────     │
│                                                                     │
│   4. DIRECTION ALIGNMENT                                            │
│      ─────────────────────                                          │
│      Allineamento tra direzione target e direzione "facile" base    │
│                                                                     │
│      • Se v_tcp è principalmente lungo X_world → base può aiutare   │
│      • Se v_tcp è verticale (Z) → solo braccio                      │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 7. Strategie di Controllo

### 7.1. Architettura di Controllo a Due Livelli

```
┌─────────────────────────────────────────────────────────────────────┐
│               CONTROLLO A DUE LIVELLI (PROPOSTO)                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│               ┌───────────────────────────────────┐                 │
│               │    WHOLE-BODY CONTROLLER          │                 │
│               │    (Livello Alto - 100 Hz)        │                 │
│               │                                   │                 │
│               │  • Riceve target pose             │                 │
│               │  • Calcola v_tcp desiderata       │                 │
│               │  • Esegue task allocation         │                 │
│               └─────────────────┬─────────────────┘                 │
│                                 │                                   │
│                    ┌────────────┴────────────┐                      │
│                    │                         │                      │
│                    ▼                         ▼                      │
│  ┌─────────────────────────┐   ┌─────────────────────────┐          │
│  │   ARM CONTROLLER        │   │   BASE CONTROLLER       │          │
│  │   (Livello Basso)       │   │   (Livello Basso)       │          │
│  │                         │   │                         │          │
│  │   • Input: v_arm_task   │   │   • Input: v_base_task  │          │
│  │   • Jacobian IK         │   │   • Kinematics model    │          │
│  │   • Safety limits       │   │   • Non-holonomic cnst  │          │
│  │   • Joint vel output    │   │   • cmd_vel output      │          │
│  │                         │   │                         │          │
│  │   Rate: 100-500 Hz      │   │   Rate: 20-50 Hz        │          │
│  └─────────────────────────┘   └─────────────────────────┘          │
│                                                                     │
│   NOTA: I due controller possono girare a frequenze diverse!        │
│   Tipicamente la base è più lenta (inerzia, ruote, ecc.)            │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 7.2. Gestione delle Dinamiche Diverse

Base e braccio hanno dinamiche molto diverse:

```
┌─────────────────────────────────────────────────────────────────────┐
│              DIFFERENZE DINAMICHE BASE vs BRACCIO                   │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│                          BRACCIO              BASE                  │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  Massa               5-30 kg             100-500 kg         │   │
│   │  Inerzia             Bassa               Alta               │   │
│   │  Vel. max lineare    ~1 m/s              ~0.5-2 m/s         │   │
│   │  Acc. max            ~10 m/s²            ~1-2 m/s²          │   │
│   │  Tempo risposta      10-50 ms            100-500 ms         │   │
│   │  Precisione          ~0.1 mm             ~1-10 mm           │   │
│   │  Freq. controllo     100-1000 Hz         20-50 Hz           │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   IMPLICAZIONI:                                                     │
│                                                                     │
│   • Il braccio può "compensare" i movimenti lenti della base        │
│   • Durante accelerazioni della base, il braccio può stabilizzare   │
│     il TCP                                                          │
│   • Mai affidarsi alla base per movimenti ad alta frequenza         │
│                                                                     │
│   STRATEGIA: Feedforward della velocità base nel controller braccio │
│                                                                     │
│   v_arm_compensated = v_tcp_desired - v_tcp_from_base               │
│                                                                     │
│   Dove v_tcp_from_base è il contributo "passivo" del movimento     │
│   base sulla posizione del TCP.                                     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 7.3. Vincoli Non-Olonomi

Per basi con vincoli cinematici (es. differenziale):

```
┌─────────────────────────────────────────────────────────────────────┐
│                   VINCOLI NON-OLONOMI                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   BASE DIFFERENZIALE:                                               │
│                                                                     │
│   Non può muoversi lateralmente (vy = 0 istantaneo)                 │
│                                                                     │
│       ┌─────────┐                                                   │
│       │         │                                                   │
│       │  [===]  │ ──▶ può muoversi in avanti/indietro               │
│       │  ○   ○  │                                                   │
│       └─────────┘     può ruotare                                   │
│           ↑                                                         │
│           │           NON può muoversi lateralmente!                │
│           ╳                                                         │
│                                                                     │
│   IMPLICAZIONE PER IL CONTROLLER:                                   │
│                                                                     │
│   Se v_tcp_desired ha componente Y (laterale nel frame base):       │
│   • Opzione 1: Il braccio compensa completamente                    │
│   • Opzione 2: La base ruota per allinearsi, poi avanza             │
│                                                                     │
│   ─────────────────────────────────────────────────────────────     │
│                                                                     │
│   PROIEZIONE DEI COMANDI:                                           │
│                                                                     │
│   v_base_feasible = project(v_base_desired, NonHolonomicSpace)      │
│                                                                     │
│   Per base differenziale:                                           │
│   [ v_linear ]   =  [ cos(θ)   sin(θ)   0 ] · v_desired             │
│   [ ω_angular]      [   0        0      1 ]                         │
│                                                                     │
│   La componente Y viene "persa" nella proiezione.                   │
│   Questo residuo deve essere gestito dal braccio o dalla            │
│   pianificazione.                                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 8. Considerazioni su Frame e Trasformazioni

### 8.1. Catena di Frame

Con una base mobile, la catena TF si allunga:

```
┌─────────────────────────────────────────────────────────────────────┐
│                       CATENA TF ESTESA                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   BRACCIO FISSO (attuale):                                          │
│                                                                     │
│   world ──── base_link ──── shoulder ──── ... ──── tcp              │
│         (fisso)                                                     │
│                                                                     │
│   ─────────────────────────────────────────────────────────────     │
│                                                                     │
│   MANIPOLATORE MOBILE (esteso):                                     │
│                                                                     │
│   map/odom ──── base_footprint ──── base_link ──── arm_base_link    │
│            (dinamico, dalla       (può includere   (base del        │
│             localizzazione)        z se elevatore)  braccio)        │
│                    │                                                │
│                    └──── sensor_frames (lidar, camera, ecc.)        │
│                                                                     │
│   arm_base_link ──── shoulder ──── ... ──── tcp                     │
│                 (catena cinematica invariata)                       │
│                                                                     │
│   ─────────────────────────────────────────────────────────────     │
│                                                                     │
│   FRAME DI RIFERIMENTO PER IL CONTROLLER:                           │
│                                                                     │
│   Per il BRACCIO: target in arm_base_link (o base_link)             │
│   Per la BASE: target in map/odom (navigazione globale)             │
│                                                                     │
│   Il controller deve essere "consapevole" di entrambi i frame       │
│   e trasformare correttamente.                                      │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 8.2. Scelta del Frame per la Mappa 3D

Con la base mobile, la scelta del frame della mappa diventa cruciale:

```
┌─────────────────────────────────────────────────────────────────────┐
│                   FRAME DELLA MAPPA 3D                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   OPZIONE A: Mappa in base_link (attuale)                           │
│   ───────────────────────────────────────                           │
│   • La mappa si muove con il robot                                  │
│   • Gli ostacoli devono essere ri-trasformati ad ogni update        │
│   • ✅ Ottimo per obstacle avoidance locale                         │
│   • ❌ Non adatto per ostacoli lontani/statici                      │
│                                                                     │
│   OPZIONE B: Mappa in map/odom (fissa nel mondo)                    │
│   ───────────────────────────────────────                           │
│   • La mappa è statica                                              │
│   • Il robot si muove nella mappa                                   │
│   • ✅ Ottimo per obstacle avoidance globale                        │
│   • ❌ Richiede mappa grande, computazionalmente costoso            │
│                                                                     │
│   OPZIONE C: DUAL MAP (consigliata per manipolatore mobile)         │
│   ───────────────────────────────────────────────────────           │
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                                                             │   │
│   │  MAPPA LOCALE (base_link)    +    MAPPA GLOBALE (map)       │   │
│   │  • Alta risoluzione          │    • Bassa risoluzione       │   │
│   │  • Range limitato (3m)       │    • Range ampio (10m+)      │   │
│   │  • Per il braccio            │    • Per la base (naviga.)   │   │
│   │  • Update 10-20 Hz           │    • Update 1-5 Hz           │   │
│   │                              │                              │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   Il controller del braccio usa la mappa locale.                    │
│   Il controller della base usa la mappa globale (o costmap di       │
│   move_base).                                                       │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 9. Impatto sulla Mappa 3D e Obstacle Avoidance

### 9.1. Nuovi POI da Proteggere

Con la base mobile, abbiamo nuovi punti da proteggere:

```
┌─────────────────────────────────────────────────────────────────────┐
│                  POI ESTESI PER MANIPOLATORE MOBILE                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   BRACCIO FISSO (attuale):                                          │
│   • TCP                                                             │
│   • Link intermedi (shoulder, elbow, wrist)                         │
│                                                                     │
│   MANIPOLATORE MOBILE (esteso):                                     │
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                                                             │   │
│   │          [TCP] ─── wrist ─── elbow ─── shoulder             │   │
│   │                                          │                  │   │
│   │                                    ┌─────┴─────┐            │   │
│   │                                    │ arm_base  │            │   │
│   │                                    └─────┬─────┘            │   │
│   │   🆕 POI BASE:       ┌───────────────────┴───────────────┐  │   │
│   │                      │           BASE                    │  │   │
│   │   • Angoli           │  ●────────────────────────────●   │  │   │
│   │   • Centro           │  │                            │   │  │   │
│   │   • Bordi            │  │            ●               │   │  │   │
│   │                      │  │         (center)           │   │  │   │
│   │                      │  ●────────────────────────────●   │  │   │
│   │                      └───────────────────────────────────┘  │   │
│   │                             (4 corners + center)            │   │
│   │                                                             │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│   NOTA: I POI della base usano una strategia diversa:               │
│   • Non contribuiscono alla velocità del TCP                        │
│   • Generano comandi per la base (stop, deviazione)                 │
│   • Possono usare la costmap 2D standard di move_base               │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 9.2. Propagazione della Repulsione

```
┌─────────────────────────────────────────────────────────────────────┐
│              PROPAGAZIONE REPULSIONE (ESTESA)                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Attualmente:                                                      │
│   v_repulsive_tcp → proiettato in joint space → joint velocities    │
│                                                                     │
│   Con base mobile:                                                  │
│                                                                     │
│   v_repulsive_tcp ──┬──▶ Arm Controller ──▶ joint velocities        │
│                     │                                               │
│                     └──▶ Base Controller ──▶ cmd_vel                │
│                          (se contributo significativo)              │
│                                                                     │
│   ─────────────────────────────────────────────────────────────     │
│                                                                     │
│   REPULSIONE DALLA BASE:                                            │
│                                                                     │
│   Se un ostacolo è rilevato vicino alla base:                       │
│   1. Comando diretto alla base: stop o backup                       │
│   2. Il braccio può continuare a lavorare (se non coinvolto)        │
│      OPPURE                                                         │
│   3. Il braccio compensa il movimento della base                    │
│      (per mantenere stabile il TCP)                                 │
│                                                                     │
│   Questa è una SCELTA DI DESIGN:                                    │
│   • Sicurezza first → tutto si ferma                                │
│   • Task completion → braccio compensa                              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 10. Piano di Estensione Incrementale

### 10.1. Approccio Consigliato

L'estensione dovrebbe essere fatta in modo **incrementale**, mantenendo la retrocompatibilità:

```
┌─────────────────────────────────────────────────────────────────────┐
│                 PIANO DI ESTENSIONE INCREMENTALE                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   FASE 0: PREPARAZIONE (attuale)                                    │
│   ═══════════════════════════════                                   │
│   ☐ Documentare architettura attuale (fatto con 00_Flow_Diagram)    │
│   ☐ Identificare punti di estensione                                │
│   ☐ Definire interfacce astratte                                    │
│                                                                     │
│   FASE 1: ASTRAZIONE DELLA CINEMATICA                               │
│   ═══════════════════════════════════                               │
│   ☐ Creare interfaccia `KinematicModel`                             │
│   ☐ Implementare `ArmOnlyKinematicModel` (attuale)                  │
│   ☐ I componenti esistenti usano l'interfaccia, non la concrete    │
│   ☐ Test: nessun cambiamento di comportamento                       │
│                                                                     │
│   FASE 2: MODELLO CINEMATICO ESTESO                                 │
│   ════════════════════════════════                                  │
│   ☐ Implementare `MobileManipulatorKinematicModel`                  │
│   ☐ Jacobiano composito (base + braccio)                            │
│   ☐ Gestione ridondanza                                             │
│   ☐ Test in simulazione con base fissa                              │
│                                                                     │
│   FASE 3: TASK ALLOCATION                                           │
│   ════════════════════════                                          │
│   ☐ Implementare `TaskAllocator`                                    │
│   ☐ Strategie: arm_first, coordinated, base_first                   │
│   ☐ Parametri configurabili                                         │
│   ☐ Test con base "simulata"                                        │
│                                                                     │
│   FASE 4: CONTROLLER BASE                                           │
│   ═══════════════════════                                           │
│   ☐ Interfaccia `BaseController`                                    │
│   ☐ Implementazione per base differenziale                          │
│   ☐ Gestione vincoli non-olonomi                                    │
│   ☐ Output: /cmd_vel                                                │
│                                                                     │
│   FASE 5: INTEGRAZIONE E TEST                                       │
│   ═══════════════════════════                                       │
│   ☐ Integrazione in simulazione (Gazebo)                            │
│   ☐ Test con diversi scenari                                        │
│   ☐ Tuning parametri                                                │
│   ☐ Documentazione                                                  │
│                                                                     │
│   FASE 6: ESTENSIONE OBSTACLE AVOIDANCE                             │
│   ═════════════════════════════════════                             │
│   ☐ POI per la base                                                 │
│   ☐ Dual-map (locale + globale)                                     │
│   ☐ Integrazione con costmap di move_base                           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 10.2. Punti di Estensione nel Codice Attuale

Identificazione dei punti dove il codice attuale necessita modifiche:

```
┌─────────────────────────────────────────────────────────────────────┐
│                PUNTI DI ESTENSIONE NEL CODICE                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   1. RobotStateManager                                              │
│      ───────────────────                                            │
│      • Attualmente: gestisce solo giunti del braccio                │
│      • Estensione: includere stato della base (odom)                │
│      • Nuovo: getBaseVelocity(), getBasePose()                      │
│                                                                     │
│   2. JacobianSolver                                                 │
│      ───────────────                                                │
│      • Attualmente: Jacobiano solo per il braccio                   │
│      • Estensione: Jacobiano composito                              │
│      • Nuovo: computeExtendedJacobian()                             │
│                                                                     │
│   3. LocalPlanner                                                   │
│      ─────────────                                                  │
│      • Attualmente: output è velocità cartesiana TCP                │
│      • Estensione: invariato, ma l'output va al TaskAllocator       │
│      • Nuovo componente a valle                                     │
│                                                                     │
│   4. Pipeline principale (executePipeline)                          │
│      ─────────────────────────────────────                          │
│      • Attualmente: termina con publishVelocityCommand (giunti)     │
│      • Estensione: due output (giunti + cmd_vel)                    │
│      • Nuovo: publishBaseCommand()                                  │
│                                                                     │
│   5. Map3DManager                                                   │
│      ────────────                                                   │
│      • Attualmente: mappa in base_link (fisso)                      │
│      • Estensione: tracciare movimento base per update TF           │
│      • Opzionale: dual-map per navigazione                          │
│                                                                     │
│   6. RepulsionDataManager                                           │
│      ─────────────────────                                          │
│      • Attualmente: POI solo sul braccio                            │
│      • Estensione: POI anche sulla base                             │
│      • Nuovo: getBasePOI() per collision avoidance base             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 10.3. Rischi e Mitigazioni

```
┌─────────────────────────────────────────────────────────────────────┐
│                    RISCHI E MITIGAZIONI                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   RISCHIO 1: Complessità eccessiva                                  │
│   ─────────────────────────────────                                 │
│   • Mitigazione: Implementare prima "arm_first" mode                │
│   • La base si muove solo quando necessario                         │
│   • Meno parametri da tuning                                        │
│                                                                     │
│   RISCHIO 2: Oscillazioni/instabilità                               │
│   ─────────────────────────────────                                 │
│   • Mitigazione: Filtraggio aggressivo sui comandi base             │
│   • Isteresi nella task allocation                                  │
│   • Rate limitato per comandi base                                  │
│                                                                     │
│   RISCHIO 3: Latenza/sincronizzazione                               │
│   ─────────────────────────────────                                 │
│   • Mitigazione: Feedforward della velocità base nel braccio        │
│   • Predizione stato base (se disponibile odometria accurata)       │
│   • Buffer per compensazione latenza TF                             │
│                                                                     │
│   RISCHIO 4: Test e validazione                                     │
│   ─────────────────────────────                                     │
│   • Mitigazione: Test incrementali in simulazione                   │
│   • Unit test per ogni nuovo componente                             │
│   • Scenari di test ben definiti                                    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Conclusioni

L'estensione del **Cartesian Velocity Controller** a un manipolatore mobile è un'evoluzione naturale dell'architettura attuale. Le modifiche principali sono:

1. **Cinematica Estesa**: Jacobiano composito che include base + braccio
2. **Task Allocation**: Nuovo modulo per decidere chi fa cosa
3. **Dual Control Output**: Comandi per giunti braccio + cmd_vel base
4. **Frame Dinamici**: Gestione di una catena TF più complessa
5. **Obstacle Avoidance Esteso**: POI anche sulla base

L'approccio incrementale proposto permette di:
- Mantenere il sistema funzionante durante lo sviluppo
- Testare ogni componente isolatamente
- Ridurre il rischio di introdurre bug

Il prossimo passo consigliato è definire le **interfacce astratte** (KinematicModel, BaseController, TaskAllocator) prima di procedere con l'implementazione.

---

*Documento generato per il pacchetto `cartesian_velocity_controller`.*
