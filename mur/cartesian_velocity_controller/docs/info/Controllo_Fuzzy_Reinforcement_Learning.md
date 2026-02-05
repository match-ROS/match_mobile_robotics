# Integrazione Controllo Fuzzy e Reinforcement Learning per Obstacle Avoidance Adattivo

**Data:** 26 Gennaio 2026  
**Versione:** 1.0  
**Stato:** Documento Concettuale  

---

## 📋 Indice

1. [Introduzione e Motivazione](#1-introduzione-e-motivazione)
2. [Architettura Proposta](#2-architettura-proposta)
3. [Sistema di Controllo Fuzzy](#3-sistema-di-controllo-fuzzy)
4. [Integrazione con Reinforcement Learning](#4-integrazione-con-reinforcement-learning)
5. [Classificazione degli Ostacoli](#5-classificazione-degli-ostacoli)
6. [Explainable AI per Contesto Industriale](#6-explainable-ai-per-contesto-industriale)
7. [Sicurezza e Certificabilità](#7-sicurezza-e-certificabilità)
8. [Roadmap di Implementazione](#8-roadmap-di-implementazione)

---

## 1. Introduzione e Motivazione

### 1.1. Contesto

Il **Cartesian Velocity Controller** attuale implementa un sistema di obstacle avoidance basato su campi potenziali repulsivi. Questo approccio, sebbene efficace, presenta limitazioni:

| Limitazione Attuale | Impatto |
|---------------------|---------|
| Comportamento uniforme | Stessa reazione per tutti gli ostacoli |
| Parametri statici | Non si adatta all'ambiente |
| Non distingue tipologie | Operatore umano = oggetto statico |
| Black-box per operatori | Difficile prevedere il comportamento |

### 1.2. Obiettivo

Sviluppare un sistema di controllo che:

1. **Adatti** il comportamento in base alla tipologia di ostacolo rilevato
2. **Apprenda** strategie ottimali tramite Reinforcement Learning
3. **Sia Explainable** per garantire sicurezza in contesto industriale
4. **Protegga** l'operatore umano con comportamenti certificabili

```
┌─────────────────────────────────────────────────────────────────────┐
│                    VISIONE DEL SISTEMA                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   PERCEZIONE              DECISIONE              AZIONE             │
│   ───────────             ─────────              ──────             │
│                                                                     │
│   ┌─────────────┐     ┌──────────────────┐      ┌────────────┐     │
│   │ Sensori 3D  │     │  FUZZY CONTROLLER │     │ Velocity   │     │
│   │ + Classif.  │────▶│  (trained by RL)  │────▶│ Commands   │     │
│   │ Ostacoli    │     │  + Safety Layer   │     │            │     │
│   └─────────────┘     └──────────────────┘      └────────────┘     │
│                              │                                      │
│                              ▼                                      │
│                       ┌──────────────┐                              │
│                       │ EXPLAINABLE  │                              │
│                       │ INTERFACE    │                              │
│                       └──────────────┘                              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.3. Perché Fuzzy + Reinforcement Learning?

| Approccio | Vantaggi | Limitazioni |
|-----------|----------|-------------|
| **Solo Fuzzy** | Interpretabile, deterministico | Richiede tuning manuale esperto |
| **Solo RL** | Ottimizzazione automatica | Black-box, difficile da certificare |
| **Fuzzy + RL** | ✅ Interpretabile + Ottimizzato | Complessità di integrazione |

L'approccio ibrido permette di:
- Usare la **struttura fuzzy** per garantire interpretabilità
- Usare il **RL** per ottimizzare i parametri automaticamente
- Mantenere **vincoli di sicurezza** espliciti e verificabili

---

## 2. Architettura Proposta

### 2.1. Schema Generale

```
┌─────────────────────────────────────────────────────────────────────┐
│              ARCHITETTURA FUZZY-RL OBSTACLE AVOIDANCE               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    PERCEPTION LAYER                          │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │   │
│  │  │ Mappa 3D     │  │ Obstacle     │  │ Human        │       │   │
│  │  │ (esistente)  │  │ Classifier   │  │ Detector     │       │   │
│  │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘       │   │
│  └─────────┼─────────────────┼─────────────────┼───────────────┘   │
│            │                 │                 │                    │
│            ▼                 ▼                 ▼                    │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    DECISION LAYER                            │   │
│  │                                                              │   │
│  │   ┌────────────────────────────────────────────────────┐    │   │
│  │   │           FUZZY INFERENCE SYSTEM                   │    │   │
│  │   │                                                    │    │   │
│  │   │  Inputs:                    Outputs:               │    │   │
│  │   │  • distance_to_obstacle     • velocity_scale       │    │   │
│  │   │  • obstacle_type            • approach_angle       │    │   │
│  │   │  • relative_velocity        • safety_margin        │    │   │
│  │   │  • workspace_position       • response_urgency     │    │   │
│  │   │                                                    │    │   │
│  │   │  ┌────────────────────────────────────────────┐   │    │   │
│  │   │  │         FUZZY RULE BASE                    │   │    │   │
│  │   │  │  (parametri ottimizzati da RL)             │   │    │   │
│  │   │  └────────────────────────────────────────────┘   │    │   │
│  │   └────────────────────────────────────────────────────┘    │   │
│  │                          │                                   │   │
│  │                          ▼                                   │   │
│  │   ╔════════════════════════════════════════════════════╗    │   │
│  │   ║              SAFETY SUPERVISOR                     ║    │   │
│  │   ║  (layer NON modificabile, vincoli hard-coded)      ║    │   │
│  │   ╚════════════════════════════════════════════════════╝    │   │
│  └──────────────────────────────┬──────────────────────────────┘   │
│                                 │                                   │
│                                 ▼                                   │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    EXECUTION LAYER                           │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │   │
│  │  │ Velocity     │  │ Jacobian IK  │  │ Joint        │       │   │
│  │  │ Modulator    │  │ (esistente)  │  │ Commands     │       │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘       │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2. Integrazione con il Controller Esistente

Il sistema fuzzy-RL si inserisce **tra** il LocalPlanner e il VelocityFilter esistenti:

```
Pipeline Attuale:
  GlobalPlanner → LocalPlanner → VelocityFilter → Jacobian → Joints

Pipeline Estesa:
  GlobalPlanner → LocalPlanner → [FUZZY-RL MODULE] → VelocityFilter → Jacobian → Joints
                                       │
                                       ├── Modula v_attractive
                                       ├── Modula v_repulsive  
                                       └── Adatta safety margins
```

---

## 3. Sistema di Controllo Fuzzy

### 3.1. Variabili di Input (Fuzzificazione)

```
┌─────────────────────────────────────────────────────────────────────┐
│                    VARIABILI INPUT FUZZY                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. DISTANCE TO OBSTACLE (d)                                        │
│     ───────────────────────                                         │
│     Universo: [0, 2.0] metri                                        │
│                                                                     │
│     μ(d)                                                            │
│      1│    VERY_CLOSE    CLOSE    MEDIUM    FAR                    │
│       │   ╱╲            ╱╲       ╱╲        ╱                        │
│       │  ╱  ╲          ╱  ╲     ╱  ╲      ╱                         │
│       │ ╱    ╲        ╱    ╲   ╱    ╲    ╱                          │
│      0│──────────────────────────────────────▶ d                    │
│        0    0.2  0.4  0.6  0.8  1.0  1.2  1.5  2.0                  │
│                                                                     │
│  2. OBSTACLE TYPE (type)                                            │
│     ──────────────────                                              │
│     Categorie discrete (crisp):                                     │
│     • HUMAN (operatore)                                             │
│     • DYNAMIC (oggetto in movimento)                                │
│     • STATIC (struttura fissa)                                      │
│     • UNKNOWN (non classificato)                                    │
│                                                                     │
│  3. RELATIVE VELOCITY (v_rel)                                       │
│     ─────────────────────────                                       │
│     Universo: [-1.0, 1.0] m/s (negativo = avvicinamento)           │
│                                                                     │
│     μ(v)                                                            │
│      1│  APPROACHING   STATIC    RECEDING                          │
│       │      ╱╲         ╱╲         ╱╲                               │
│       │     ╱  ╲       ╱  ╲       ╱  ╲                              │
│      0│────────────────────────────────▶ v                          │
│       -1.0   -0.3   0   0.3      1.0                                │
│                                                                     │
│  4. WORKSPACE POSITION (ws)                                         │
│     ──────────────────────                                          │
│     Posizione relativa nel workspace del braccio                    │
│     • CENTER (alta manipolabilità)                                  │
│     • EDGE (vicino ai limiti)                                       │
│     • LIMIT (ai limiti cinematici)                                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.2. Variabili di Output (Defuzzificazione)

```
┌─────────────────────────────────────────────────────────────────────┐
│                    VARIABILI OUTPUT FUZZY                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. VELOCITY_SCALE (α)                                              │
│     ──────────────────                                              │
│     Fattore di scala per velocità TCP [0, 1]                        │
│     • STOP (0.0-0.1)                                                │
│     • VERY_SLOW (0.1-0.3)                                           │
│     • SLOW (0.3-0.5)                                                │
│     • NORMAL (0.5-0.8)                                              │
│     • FAST (0.8-1.0)                                                │
│                                                                     │
│  2. SAFETY_MARGIN (m)                                               │
│     ──────────────────                                              │
│     Distanza minima da mantenere [0.1, 1.0] metri                   │
│     • MINIMAL (0.1-0.2m) - per ostacoli statici noti                │
│     • STANDARD (0.2-0.4m) - default                                 │
│     • EXTENDED (0.4-0.6m) - per ostacoli dinamici                   │
│     • MAXIMUM (0.6-1.0m) - per operatori umani                      │
│                                                                     │
│  3. RESPONSE_URGENCY (u)                                            │
│     ─────────────────────                                           │
│     Velocità di reazione del sistema [0, 1]                         │
│     • SMOOTH (lento, graduale)                                      │
│     • MODERATE                                                      │
│     • QUICK                                                         │
│     • IMMEDIATE (massima reattività)                                │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.3. Base di Regole Fuzzy

Le regole fuzzy definiscono il comportamento del sistema in modo **leggibile e verificabile**:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    ESEMPI DI REGOLE FUZZY                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  REGOLE PER OPERATORE UMANO (massima sicurezza)                     │
│  ───────────────────────────────────────────────                    │
│                                                                     │
│  R1: IF obstacle_type IS HUMAN                                      │
│      AND distance IS VERY_CLOSE                                     │
│      THEN velocity_scale IS STOP                                    │
│      AND safety_margin IS MAXIMUM                                   │
│      AND response_urgency IS IMMEDIATE                              │
│                                                                     │
│  R2: IF obstacle_type IS HUMAN                                      │
│      AND distance IS CLOSE                                          │
│      AND relative_velocity IS APPROACHING                           │
│      THEN velocity_scale IS VERY_SLOW                               │
│      AND safety_margin IS EXTENDED                                  │
│                                                                     │
│  R3: IF obstacle_type IS HUMAN                                      │
│      AND distance IS MEDIUM                                         │
│      THEN velocity_scale IS SLOW                                    │
│      AND safety_margin IS STANDARD                                  │
│                                                                     │
│  ─────────────────────────────────────────────────────────────      │
│                                                                     │
│  REGOLE PER OSTACOLI DINAMICI                                       │
│  ────────────────────────────                                       │
│                                                                     │
│  R4: IF obstacle_type IS DYNAMIC                                    │
│      AND distance IS CLOSE                                          │
│      AND relative_velocity IS APPROACHING                           │
│      THEN velocity_scale IS SLOW                                    │
│      AND response_urgency IS QUICK                                  │
│                                                                     │
│  R5: IF obstacle_type IS DYNAMIC                                    │
│      AND relative_velocity IS RECEDING                              │
│      THEN velocity_scale IS NORMAL                                  │
│      AND safety_margin IS STANDARD                                  │
│                                                                     │
│  ─────────────────────────────────────────────────────────────      │
│                                                                     │
│  REGOLE PER OSTACOLI STATICI                                        │
│  ───────────────────────────                                        │
│                                                                     │
│  R6: IF obstacle_type IS STATIC                                     │
│      AND distance IS CLOSE                                          │
│      THEN velocity_scale IS SLOW                                    │
│      AND safety_margin IS MINIMAL                                   │
│                                                                     │
│  R7: IF obstacle_type IS STATIC                                     │
│      AND distance IS FAR                                            │
│      THEN velocity_scale IS FAST                                    │
│      AND safety_margin IS MINIMAL                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 4. Integrazione con Reinforcement Learning

### 4.1. Cosa Ottimizza il RL?

Il Reinforcement Learning **NON** sostituisce il sistema fuzzy, ma **ottimizza i suoi parametri**:

```
┌─────────────────────────────────────────────────────────────────────┐
│               PARAMETRI OTTIMIZZABILI DAL RL                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. MEMBERSHIP FUNCTIONS                                            │
│     ────────────────────                                            │
│     I "confini" delle funzioni fuzzy possono essere adattati:       │
│                                                                     │
│     Prima del training:              Dopo il training:              │
│     μ(d)                             μ(d)                           │
│      1│  ╱╲    ╱╲                     1│   ╱╲  ╱╲                   │
│       │ ╱  ╲  ╱  ╲                     │  ╱  ╲╱  ╲                  │
│      0│─────────────▶                 0│─────────────▶              │
│        0  0.3 0.6                       0  0.4 0.7                  │
│                                                                     │
│  2. RULE WEIGHTS                                                    │
│     ────────────                                                    │
│     Ogni regola ha un peso che influenza l'output:                  │
│                                                                     │
│     R1: weight = 0.95  (alta priorità per sicurezza umana)          │
│     R6: weight = 0.60  (ottimizzato per efficienza)                 │
│                                                                     │
│  3. OUTPUT SCALING                                                  │
│     ──────────────                                                  │
│     Fattori di scala sugli output:                                  │
│                                                                     │
│     velocity_scale_gain = 0.85  (learned)                           │
│     safety_margin_offset = +0.05m (learned)                         │
│                                                                     │
│  ⚠️ PARAMETRI NON MODIFICABILI (safety-critical):                   │
│     • Distanza minima assoluta per HUMAN                            │
│     • Velocità massima assoluta                                     │
│     • Regole di stop di emergenza                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.2. Schema di Training RL

```
┌─────────────────────────────────────────────────────────────────────┐
│                    TRAINING LOOP RL                                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌───────────────────────────────────────────────────────────┐     │
│   │                    SIMULAZIONE                            │     │
│   │  (Gazebo + scenari con ostacoli diversi)                  │     │
│   └─────────────────────────┬─────────────────────────────────┘     │
│                             │                                        │
│                             ▼                                        │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  STATE (s)                                                  │   │
│   │  • Distanze POI-ostacoli                                    │   │
│   │  • Tipi di ostacoli                                         │   │
│   │  • Velocità relative                                        │   │
│   │  • Configurazione braccio                                   │   │
│   │  • Distanza dal target                                      │   │
│   └─────────────────────────┬───────────────────────────────────┘   │
│                             │                                        │
│                             ▼                                        │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  POLICY (π) = Fuzzy Controller con parametri θ              │   │
│   │                                                             │   │
│   │  θ = {membership_params, rule_weights, output_scales}       │   │
│   └─────────────────────────┬───────────────────────────────────┘   │
│                             │                                        │
│                             ▼                                        │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  ACTION (a)                                                 │   │
│   │  • velocity_scale                                           │   │
│   │  • safety_margin                                            │   │
│   │  • response_urgency                                         │   │
│   └─────────────────────────┬───────────────────────────────────┘   │
│                             │                                        │
│                             ▼                                        │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  REWARD (r)                                                 │   │
│   │                                                             │   │
│   │  r = r_task + r_safety + r_efficiency + r_smoothness        │   │
│   │                                                             │   │
│   │  r_task = +10 se raggiunge target                           │   │
│   │  r_safety = -100 se collisione con HUMAN                    │   │
│   │           = -50 se collisione con DYNAMIC                   │   │
│   │           = -20 se collisione con STATIC                    │   │
│   │  r_efficiency = -0.01 * tempo_impiegato                     │   │
│   │  r_smoothness = -0.1 * jerk (variazione accelerazione)      │   │
│   └─────────────────────────┬───────────────────────────────────┘   │
│                             │                                        │
│                             ▼                                        │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │  UPDATE θ (PPO, SAC, or similar)                            │   │
│   │                                                             │   │
│   │  θ_new = θ + α * ∇J(θ)                                      │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.3. Algoritmi RL Consigliati

| Algoritmo | Pro | Contro | Raccomandazione |
|-----------|-----|--------|-----------------|
| **PPO** | Stabile, sample-efficient | Tuning hyperparams | ✅ Consigliato |
| **SAC** | Ottimo per continuo | Più complesso | Alternativa |
| **TD3** | Robusto | Richiede molti samples | Per fine-tuning |

### 4.4. Ambiente di Training

```yaml
# training_config.yaml

environment:
  simulator: "gazebo"
  world: "industrial_scenario.world"
  
scenarios:
  - name: "static_obstacles"
    description: "Navigazione tra ostacoli fissi"
    weight: 0.2
    
  - name: "dynamic_obstacles"
    description: "Oggetti in movimento (carrelli, altri robot)"
    weight: 0.3
    
  - name: "human_presence"
    description: "Operatore umano nell'area di lavoro"
    weight: 0.4
    
  - name: "mixed_scenario"
    description: "Combinazione realistica"
    weight: 0.1

training:
  algorithm: "PPO"
  total_timesteps: 1_000_000
  learning_rate: 3e-4
  batch_size: 64
  n_epochs: 10
  
constraints:
  # Vincoli HARD - non violabili durante training
  min_human_distance: 0.5  # metri
  max_velocity: 0.5  # m/s vicino a umani
  emergency_stop_distance: 0.3  # metri
```

---

## 5. Classificazione degli Ostacoli

### 5.1. Pipeline di Classificazione

```
┌─────────────────────────────────────────────────────────────────────┐
│                 PIPELINE CLASSIFICAZIONE OSTACOLI                   │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌────────────┐     ┌────────────┐     ┌────────────┐              │
│  │ PointCloud │     │  Clustering │     │ Feature    │              │
│  │ 3D         │────▶│  (DBSCAN)  │────▶│ Extraction │              │
│  └────────────┘     └────────────┘     └─────┬──────┘              │
│                                              │                      │
│                                              ▼                      │
│                     ┌────────────────────────────────────────┐      │
│                     │         CLASSIFICATORE                 │      │
│                     │                                        │      │
│                     │  ┌─────────────────────────────────┐  │      │
│                     │  │ Features:                       │  │      │
│                     │  │ • Dimensioni (W × H × D)        │  │      │
│                     │  │ • Forma (aspect ratio)          │  │      │
│                     │  │ • Velocità (tracking)           │  │      │
│                     │  │ • Pattern movimento             │  │      │
│                     │  │ • Posizione verticale           │  │      │
│                     │  └─────────────────────────────────┘  │      │
│                     │                                        │      │
│                     │  ┌─────────────────────────────────┐  │      │
│                     │  │ Classificazione:                │  │      │
│                     │  │                                 │  │      │
│                     │  │  HUMAN:    altezza ~1.5-2m      │  │      │
│                     │  │            movimento organico    │  │      │
│                     │  │            pattern bipede        │  │      │
│                     │  │                                 │  │      │
│                     │  │  DYNAMIC:  movimento regolare   │  │      │
│                     │  │            velocità costante    │  │      │
│                     │  │                                 │  │      │
│                     │  │  STATIC:   velocità = 0         │  │      │
│                     │  │            forma stabile        │  │      │
│                     │  │                                 │  │      │
│                     │  │  UNKNOWN:  dati insufficienti   │  │      │
│                     │  └─────────────────────────────────┘  │      │
│                     └────────────────────────────────────────┘      │
│                                              │                      │
│                                              ▼                      │
│                     ┌────────────────────────────────────────┐      │
│                     │  OUTPUT:                               │      │
│                     │  {id, type, confidence, bbox, velocity}│      │
│                     └────────────────────────────────────────┘      │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.2. Comportamento per Tipologia

| Tipo Ostacolo | Safety Margin | Max Velocity | Comportamento |
|---------------|---------------|--------------|---------------|
| **HUMAN** | 0.8-1.0 m | 0.3 m/s | Massima cautela, stop se molto vicino |
| **DYNAMIC** | 0.4-0.6 m | 0.5 m/s | Traccia traiettoria, predice posizione |
| **STATIC** | 0.2-0.3 m | 0.8 m/s | Può avvicinarsi, evitamento efficiente |
| **UNKNOWN** | 0.6 m | 0.4 m/s | Trattato come potenzialmente pericoloso |

---

## 6. Explainable AI per Contesto Industriale

### 6.1. Requisiti di Explainability

In un contesto industriale, il sistema deve essere:

| Requisito | Descrizione | Implementazione |
|-----------|-------------|-----------------|
| **Interpretabile** | Operatori capiscono le decisioni | Regole fuzzy in linguaggio naturale |
| **Prevedibile** | Comportamento anticipabile | Visualizzazione zone di sicurezza |
| **Tracciabile** | Log delle decisioni | Registrazione regole attivate |
| **Verificabile** | Audit di sicurezza | Report automatici |

### 6.2. Interfaccia di Spiegazione

```
┌─────────────────────────────────────────────────────────────────────┐
│                 EXPLAINABILITY INTERFACE                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  LIVELLO 1: VISUALIZZAZIONE REAL-TIME                               │
│  ────────────────────────────────────                               │
│                                                                     │
│  ┌───────────────────────────────────────────────────────────┐     │
│  │  RViz / HMI Display                                       │     │
│  │                                                           │     │
│  │   ┌─────────────────────────────────────────────────┐    │     │
│  │   │    🔴 Zona Stop (umano rilevato)                │    │     │
│  │   │    🟡 Zona Rallentamento                        │    │     │
│  │   │    🟢 Zona Operazione Normale                   │    │     │
│  │   │                                                 │    │     │
│  │   │            [Robot]                              │    │     │
│  │   │               │                                 │    │     │
│  │   │    🟢🟢🟢🟢   │   🟡🟡 👤 🔴🔴                │    │     │
│  │   │    Ostacolo   └──────▶ Traiettoria              │    │     │
│  │   │    statico         (rallentando...)             │    │     │
│  │   └─────────────────────────────────────────────────┘    │     │
│  │                                                           │     │
│  │   Status: "Rallentamento per presenza operatore a 1.2m"  │     │
│  │   Velocità: 0.3 m/s (limite: 0.5 m/s)                    │     │
│  │   Regole attive: R2, R5                                   │     │
│  └───────────────────────────────────────────────────────────┘     │
│                                                                     │
│  LIVELLO 2: LOG DECISIONALE                                         │
│  ───────────────────────────                                        │
│                                                                     │
│  [12:34:56.789] Obstacle detected: HUMAN, distance=1.2m             │
│  [12:34:56.790] Active rules: R2 (weight=0.95), R5 (weight=0.7)     │
│  [12:34:56.791] Decision: velocity_scale=0.35, safety_margin=0.6m   │
│  [12:34:56.792] Explanation: "Reduced speed due to approaching      │
│                               human operator"                        │
│                                                                     │
│  LIVELLO 3: REPORT DI AUDIT                                         │
│  ───────────────────────────                                        │
│                                                                     │
│  - Totale interazioni con operatori: 47                             │
│  - Distanza minima raggiunta: 0.82m (sopra soglia 0.5m) ✅          │
│  - Stop di emergenza attivati: 0                                    │
│  - Regole più frequenti: R2 (68%), R3 (22%), R7 (10%)               │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 6.3. Generazione Automatica di Spiegazioni

```python
# Pseudo-codice per generazione spiegazioni

def generate_explanation(active_rules, inputs, outputs):
    """
    Genera una spiegazione in linguaggio naturale
    della decisione corrente del sistema fuzzy.
    """
    
    explanations = []
    
    for rule in active_rules:
        if rule.id == "R1" and rule.activation > 0.5:
            explanations.append(
                f"STOP richiesto: operatore umano rilevato a {inputs['distance']:.2f}m"
            )
        elif rule.id == "R2" and rule.activation > 0.5:
            explanations.append(
                f"Velocità ridotta: operatore in avvicinamento "
                f"(velocità relativa: {inputs['rel_velocity']:.2f} m/s)"
            )
        elif rule.id == "R4" and rule.activation > 0.5:
            explanations.append(
                f"Attenzione: ostacolo dinamico in movimento"
            )
        # ... altre regole
    
    # Sintesi finale
    summary = f"Decisione: velocità={outputs['velocity_scale']*100:.0f}%, " \
              f"margine sicurezza={outputs['safety_margin']:.2f}m"
    
    return {
        "summary": summary,
        "details": explanations,
        "confidence": min([r.activation for r in active_rules])
    }
```

---

## 7. Sicurezza e Certificabilità

### 7.1. Architettura di Sicurezza a Layer

```
┌─────────────────────────────────────────────────────────────────────┐
│                    ARCHITETTURA DI SICUREZZA                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  LAYER 4: HARDWARE SAFETY (esterno al software)                     │
│  ══════════════════════════════════════════════                     │
│  • Safety PLC                                                       │
│  • Light curtains                                                   │
│  • Emergency stop hardware                                          │
│                                                                     │
│  ─────────────────────────────────────────────────────────────      │
│                                                                     │
│  LAYER 3: SOFTWARE SAFETY SUPERVISOR                                │
│  ════════════════════════════════════                               │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  VINCOLI HARD-CODED (NON modificabili da RL)                │   │
│  │                                                             │   │
│  │  • IF distance_to_human < 0.3m THEN IMMEDIATE_STOP          │   │
│  │  • IF velocity > MAX_SAFE_VELOCITY THEN CLAMP               │   │
│  │  • IF sensor_failure THEN SAFE_STATE                        │   │
│  │                                                             │   │
│  │  Questi vincoli SOVRASCRIVONO qualsiasi output fuzzy/RL     │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ─────────────────────────────────────────────────────────────      │
│                                                                     │
│  LAYER 2: FUZZY-RL CONTROLLER (adattivo)                            │
│  ═══════════════════════════════════════                            │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  • Ottimizzato da RL                                        │   │
│  │  • Può essere aggiornato                                    │   │
│  │  • Vincolato dai limiti del Layer 3                         │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ─────────────────────────────────────────────────────────────      │
│                                                                     │
│  LAYER 1: VELOCITY CONTROLLER (esistente)                           │
│  ═════════════════════════════════════════                          │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  • Jacobian IK                                              │   │
│  │  • Joint limits                                             │   │
│  │  • Collision detection                                      │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 7.2. Vincoli di Sicurezza Hard-Coded

```cpp
/**
 * @class SafetySupervisor
 * @brief Layer di sicurezza non modificabile
 * 
 * Questi vincoli hanno priorità ASSOLUTA su qualsiasi
 * output del sistema fuzzy-RL.
 */
class SafetySupervisor
{
public:
    // Costanti di sicurezza (NON configurabili a runtime)
    static constexpr double HUMAN_MIN_DISTANCE = 0.3;      // [m]
    static constexpr double HUMAN_STOP_VELOCITY = 0.0;     // [m/s]
    static constexpr double MAX_VELOCITY_NEAR_HUMAN = 0.5; // [m/s]
    static constexpr double MAX_ACCELERATION = 2.0;        // [m/s²]
    
    /**
     * @brief Applica vincoli di sicurezza all'output
     * @param fuzzy_output Output dal controller fuzzy-RL
     * @param obstacles Lista ostacoli rilevati
     * @return Output safe (potenzialmente più restrittivo)
     */
    SafeOutput applySafetyConstraints(
        const FuzzyOutput& fuzzy_output,
        const std::vector<Obstacle>& obstacles)
    {
        SafeOutput safe = fuzzy_output;
        
        // Vincolo 1: Stop immediato se umano troppo vicino
        for (const auto& obs : obstacles)
        {
            if (obs.type == ObstacleType::HUMAN && 
                obs.distance < HUMAN_MIN_DISTANCE)
            {
                safe.velocity_scale = 0.0;
                safe.reason = "EMERGENCY: Human too close";
                return safe;  // Priorità massima
            }
        }
        
        // Vincolo 2: Limita velocità vicino a umani
        double min_human_dist = getMinHumanDistance(obstacles);
        if (min_human_dist < 1.0)  // Entro 1 metro
        {
            double max_allowed = MAX_VELOCITY_NEAR_HUMAN * 
                                 (min_human_dist / 1.0);
            safe.velocity_scale = std::min(
                safe.velocity_scale, 
                max_allowed
            );
        }
        
        // Vincolo 3: Limita accelerazione
        safe.velocity_scale = applyAccelerationLimit(
            safe.velocity_scale,
            last_velocity_scale_,
            MAX_ACCELERATION
        );
        
        return safe;
    }
};
```

### 7.3. Conformità agli Standard

| Standard | Descrizione | Applicabilità |
|----------|-------------|---------------|
| **ISO 10218** | Safety requirements for industrial robots | Robot industriali |
| **ISO/TS 15066** | Collaborative robots | Interazione umano-robot |
| **ISO 13849** | Safety-related control systems | PL (Performance Level) |
| **IEC 62443** | Cybersecurity for industrial systems | Sicurezza IT |

### 7.4. Processo di Validazione

```
┌─────────────────────────────────────────────────────────────────────┐
│                    PROCESSO DI VALIDAZIONE                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  FASE 1: TRAINING IN SIMULAZIONE                                    │
│  ─────────────────────────────────                                  │
│  • Ambiente Gazebo con scenari vari                                 │
│  • Validazione su 10.000+ episodi                                   │
│  • Verifica: zero collisioni con HUMAN                              │
│                                                                     │
│  FASE 2: TESTING FORMALE                                            │
│  ───────────────────────                                            │
│  • Model checking delle regole fuzzy                                │
│  • Verifica formale dei vincoli di sicurezza                        │
│  • Coverage analysis delle regole                                   │
│                                                                     │
│  FASE 3: HARDWARE-IN-THE-LOOP                                       │
│  ────────────────────────────                                       │
│  • Robot reale con manichino/dummy                                  │
│  • Ostacoli dinamici controllati                                    │
│  • Misurazione distanze e tempi di reazione                         │
│                                                                     │
│  FASE 4: VALIDAZIONE SUL CAMPO                                      │
│  ─────────────────────────────                                      │
│  • Deployment graduale                                              │
│  • Monitoraggio continuo                                            │
│  • Feedback operatori                                               │
│                                                                     │
│  FASE 5: CERTIFICAZIONE                                             │
│  ────────────────────────                                           │
│  • Documentazione completa                                          │
│  • Risk assessment                                                  │
│  • Audit esterno                                                    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 8. Roadmap di Implementazione

### 8.1. Fasi di Sviluppo

```
┌─────────────────────────────────────────────────────────────────────┐
│                    ROADMAP IMPLEMENTAZIONE                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  FASE 1: CLASSIFICATORE OSTACOLI (4-6 settimane)                    │
│  ═══════════════════════════════════════════════                    │
│  □ Sviluppo modulo di clustering DBSCAN                             │
│  □ Feature extraction da point cloud                                │
│  □ Classificatore (HUMAN/DYNAMIC/STATIC/UNKNOWN)                    │
│  □ Tracking temporale ostacoli                                      │
│  □ Integrazione con mappa 3D esistente                              │
│                                                                     │
│  FASE 2: SISTEMA FUZZY BASE (3-4 settimane)                         │
│  ═══════════════════════════════════════════                        │
│  □ Implementazione FIS (Fuzzy Inference System)                     │
│  □ Definizione membership functions                                 │
│  □ Implementazione rule base iniziale                               │
│  □ Integrazione con pipeline velocity controller                    │
│  □ Testing con regole manuali                                       │
│                                                                     │
│  FASE 3: SAFETY SUPERVISOR (2-3 settimane)                          │
│  ═════════════════════════════════════════                          │
│  □ Implementazione layer di sicurezza hard-coded                    │
│  □ Testing vincoli di sicurezza                                     │
│  □ Logging e audit trail                                            │
│  □ Validazione formale vincoli                                      │
│                                                                     │
│  FASE 4: AMBIENTE RL E TRAINING (6-8 settimane)                     │
│  ═══════════════════════════════════════════════                    │
│  □ Setup ambiente Gym/Gymnasium                                     │
│  □ Wrapper per Gazebo simulation                                    │
│  □ Definizione reward function                                      │
│  □ Training con PPO/SAC                                             │
│  □ Analisi convergenza e tuning hyperparameters                     │
│                                                                     │
│  FASE 5: EXPLAINABILITY (2-3 settimane)                             │
│  ══════════════════════════════════════                             │
│  □ Interfaccia RViz per visualizzazione                             │
│  □ Sistema di logging decisionale                                   │
│  □ Generatore spiegazioni in linguaggio naturale                    │
│  □ Dashboard di monitoring                                          │
│                                                                     │
│  FASE 6: VALIDAZIONE E DEPLOY (4-6 settimane)                       │
│  ═══════════════════════════════════════════                        │
│  □ Test estensivi in simulazione                                    │
│  □ Hardware-in-the-loop testing                                     │
│  □ Validazione con operatori                                        │
│  □ Documentazione per certificazione                                │
│  □ Deploy graduale                                                  │
│                                                                     │
│  TIMELINE TOTALE: ~6-8 mesi                                         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 8.2. Dipendenze e Librerie

| Componente | Libreria Consigliata | Note |
|------------|---------------------|------|
| Fuzzy Logic | `scikit-fuzzy` / `fuzzylite` | Python / C++ |
| Reinforcement Learning | `stable-baselines3` | PPO, SAC, TD3 |
| Obstacle Clustering | `pcl` / `open3d` | Point cloud processing |
| Human Detection | `OpenPose` / `MoveNet` | Opzionale, per conferma |
| Simulation | `Gazebo` + `ros_control` | Già in uso |
| Visualization | `RViz` + custom markers | Già in uso |

### 8.3. Rischi e Mitigazioni

| Rischio | Probabilità | Impatto | Mitigazione |
|---------|-------------|---------|-------------|
| Performance insufficiente | Media | Alto | Ottimizzazione, C++ per componenti critici |
| RL non converge | Media | Medio | Reward shaping, pretrain con expert |
| Classificazione errata umani | Bassa | Critico | Fallback conservativo (UNKNOWN = HUMAN) |
| Comportamento imprevedibile | Bassa | Alto | Safety supervisor + testing estensivo |

---

## Conclusioni

L'integrazione di **controllo fuzzy** con **reinforcement learning** rappresenta un approccio promettente per creare sistemi robotici **adattivi**, **sicuri** ed **explainable** in contesti industriali.

**Punti chiave:**

1. **Fuzzy come struttura interpretabile**: Le regole fuzzy forniscono una base comprensibile per operatori e auditor
2. **RL come ottimizzatore**: Il reinforcement learning ottimizza i parametri senza compromettere l'interpretabilità
3. **Safety-first**: I vincoli di sicurezza hard-coded garantiscono comportamenti sicuri indipendentemente dal training
4. **Explainability integrata**: Ogni decisione può essere spiegata e tracciata

Questo approccio ibrido bilancia **efficienza operativa** e **sicurezza**, rendendo il sistema adatto per deployment in ambienti industriali dove collaborazione umano-robot è essenziale.

---

*Documento generato per il progetto Cartesian Velocity Controller*  
*Ultimo aggiornamento: 26 Gennaio 2026*
