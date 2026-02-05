# Sistema di Predizione della Posizione Futura degli Ostacoli

Questo documento descrive concettualmente come implementare un sistema di predizione della posizione futura degli ostacoli e come utilizzare queste informazioni per aggiornare proattivamente la mappa 3D nel contesto del `cartesian_velocity_controller`.

---

## 📑 Indice

1. [Motivazioni e Obiettivi](#motivazioni-e-obiettivi)
2. [Panoramica dell'Architettura Predittiva](#panoramica-dellarchitettura-predittiva)
3. [Acquisizione dei Dati Storici](#acquisizione-dei-dati-storici)
4. [Modelli di Predizione](#modelli-di-predizione)
5. [Integrazione con la Mappa 3D](#integrazione-con-la-mappa-3d)
6. [Gestione dell'Incertezza](#gestione-dellincertezza)
7. [Considerazioni sull'Orizzonte Temporale](#considerazioni-sullorizzonte-temporale)
8. [Impatto sul Controllo](#impatto-sul-controllo)
9. [Sfide e Compromessi](#sfide-e-compromessi)

---

## Motivazioni e Obiettivi

### Il Problema della Reattività Pura

L'architettura attuale della mappa 3D è **puramente reattiva**: legge la posizione corrente degli ostacoli dalla PlanningScene, calcola il Distance Field, e fornisce informazioni di distanza/gradiente al controller. Questo approccio presenta alcune limitazioni intrinseche:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      APPROCCIO REATTIVO (attuale)                           │
│                                                                             │
│   t = 0          t = Δt                      t = 2Δt                        │
│    ───────────────►───────────────────────────►                             │
│                                                                             │
│   Ostacolo      Sensing →      Map Update    Sensing →   Map Update         │
│   si muove      (latenza)     (calcolo EDT)   ...         ...               │
│                                                                             │
│   ⚠️ Il robot reagisce sempre con ritardo rispetto al movimento reale      │
│      dell'ostacolo                                                          │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Latenze cumulative nel sistema attuale:**
- Ritardo del sensore (se presente)
- Latenza nella pipeline di aggiornamento della PlanningScene
- Tempo di calcolo della mappa (voxelizzazione + EDT)
- Ritardo del loop di controllo

### Obiettivo della Predizione

L'introduzione di un sistema predittivo mira a **anticipare** la posizione futura degli ostacoli, permettendo al controller di:

1. **Reagire in anticipo** a traiettorie di collisione
2. **Generare movimenti più fluidi** evitando correzioni brusche dell'ultimo istante
3. **Aumentare il margine di sicurezza** in scenari dinamici
4. **Compensare le latenze** del sistema di percezione

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      APPROCCIO PREDITTIVO (proposto)                        │
│                                                                             │
│   t = 0          t = Δt                      t = 2Δt                        │
│    ───────────────►───────────────────────────►                             │
│                                                                             │
│   Ostacolo      Sensing + Prediction         Map con posizioni              │
│   si muove      della pos. a t+T_horizon     predette a t+T                 │
│                                                                             │
│   ✓ Il robot può iniziare manovre evasive PRIMA che l'ostacolo arrivi      │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Panoramica dell'Architettura Predittiva

### Schema Concettuale

L'architettura predittiva si articola in tre macro-componenti che si integrano con il sistema esistente:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    ARCHITETTURA PREDITTIVA - VISTA D'INSIEME                │
│                                                                             │
│   ┌──────────────────────┐                                                  │
│   │   PlanningScene      │                                                  │
│   │   (stato corrente)   │                                                  │
│   └──────────┬───────────┘                                                  │
│              │                                                              │
│              ▼                                                              │
│   ┌──────────────────────┐      ┌──────────────────────┐                    │
│   │   HISTORY TRACKER    │──────│   MOTION PREDICTOR   │                    │
│   │   ━━━━━━━━━━━━━━━━   │      │   ━━━━━━━━━━━━━━━━   │                    │
│   │   • Buffer circolare │      │   • Modelli lineari  │                    │
│   │   • Velocità stimate │      │   • Modelli Kalman   │                    │
│   │   • Timestamp        │      │   • Intent. learning │                    │
│   └──────────┬───────────┘      └──────────┬───────────┘                    │
│              │                             │                                │
│              │                             ▼                                │
│              │              ┌──────────────────────────┐                    │
│              │              │   PREDICTION OUTPUT      │                    │
│              │              │   ━━━━━━━━━━━━━━━━━━━━   │                    │
│              │              │   • Posizione predetta   │                    │
│              │              │   • Incertezza (cov.)    │                    │
│              │              │   • Confidence level     │                    │
│              │              └──────────┬───────────────┘                    │
│              │                         │                                    │
│              ▼                         ▼                                    │
│   ┌──────────────────────────────────────────────────────┐                  │
│   │              MAP FUSION STRATEGY                      │                  │
│   │   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━   │                  │
│   │   • Unione stato corrente + stato predetto           │                  │
│   │   • Inflazione basata su incertezza                  │                  │
│   │   • Gestione multi-orizzonte temporale               │                  │
│   └──────────────────────────────────────────────────────┘                  │
│              │                                                              │
│              ▼                                                              │
│   ┌──────────────────────┐                                                  │
│   │   VOXEL GRID 3D      │                                                  │
│   │   (EDT aggiornato)   │                                                  │
│   └──────────────────────┘                                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Flusso Logico del Sistema

1. **Acquisizione**: Ad ogni ciclo di aggiornamento, si leggono le posizioni degli ostacoli (sfere)
2. **Tracciamento**: Ogni ostacolo viene associato a un ID persistente per tracciarne l'evoluzione nel tempo
3. **Stima velocità**: Dalle posizioni storiche si stima la velocità (e eventualmente accelerazione)
4. **Predizione**: Si calcola la posizione attesa a un tempo futuro t + T_horizon
5. **Fusione**: La mappa viene costruita considerando sia lo stato attuale che quello predetto
6. **Controllo**: Il controller riceve distanze/gradienti che già tengono conto del movimento futuro

---

## Acquisizione dei Dati Storici

### Concetto di History Tracker

Per poter predire la posizione futura, è necessario **memorizzare la storia** delle osservazioni di ciascun ostacolo. Questo richiede:

#### Associazione Persistente degli Ostacoli

Gli ostacoli nella PlanningScene hanno un ID univoco. Questo ID può essere utilizzato per tracciare lo stesso ostacolo attraverso multiple osservazioni:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    OBSTACLE TRACKING (concettuale)                          │
│                                                                             │
│   Tempo t₀:  "obstacle_1" @ (1.0, 2.0, 1.5)                                 │
│   Tempo t₁:  "obstacle_1" @ (1.1, 2.0, 1.5)   ← stesso ID, nuova posizione  │
│   Tempo t₂:  "obstacle_1" @ (1.2, 2.0, 1.5)                                 │
│   ...                                                                       │
│                                                                             │
│   Per ogni ID, si mantiene un buffer circolare di N osservazioni:           │
│                                                                             │
│   "obstacle_1" → [(t₀, p₀), (t₁, p₁), (t₂, p₂), ..., (t_N, p_N)]           │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Struttura del Buffer Storico

Ogni ostacolo tracciato dovrebbe avere:

- **ID univoco**: stringa identificativa dalla PlanningScene
- **Buffer circolare**: capacità limitata (es. ultimi 20-50 campioni)
- **Timestamp**: per ogni osservazione, il tempo esatto
- **Posizione**: centro della sfera in un frame di riferimento stabile
- **Raggio**: (può variare nel tempo in alcuni scenari)

#### Considerazioni sul Frame di Riferimento

È cruciale scegliere un **frame di riferimento appropriato** per memorizzare la storia:

| Frame | Pro | Contro |
|-------|-----|--------|
| `world` / `odom` | Stabile, movimento assoluto | Accumula drift |
| `base_link` | Coerente con la mappa | L'ostacolo "si muove" apparentemente se il robot si muove |
| Frame dell'ostacolo | Movimento relativo vero | Richiede info sulla cinematica dell'ostacolo |

**Raccomandazione**: Memorizzare la storia nel frame `world` o `odom` per catturare il movimento reale dell'ostacolo. La trasformazione nel frame della mappa (`base_link`) avviene dopo la predizione.

---

## Modelli di Predizione

### Approcci Concettuali

Esistono diversi livelli di complessità per predire il movimento degli ostacoli:

### 1. Modello a Velocità Costante (Linear)

L'approccio più semplice assume che l'ostacolo mantenga la velocità corrente:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    MODELLO LINEARE (velocità costante)                      │
│                                                                             │
│   Stima velocità:                                                           │
│                                                                             │
│        p(t₁) - p(t₀)                                                        │
│   v ≈ ───────────────      (differenza finita)                              │
│         t₁ - t₀                                                             │
│                                                                             │
│   oppure: regressione lineare sugli ultimi N punti (più robusto)            │
│                                                                             │
│   Predizione:                                                               │
│                                                                             │
│   p(t + T) = p(t) + v · T                                                   │
│                                                                             │
│   ✓ Semplice da implementare                                                │
│   ✓ Basso costo computazionale                                              │
│   ✗ Non modella cambi di direzione                                          │
│   ✗ Accumula errore rapidamente                                             │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2. Modello con Accelerazione (Quadratico)

Estende il modello precedente includendo l'accelerazione:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    MODELLO QUADRATICO (con accelerazione)                   │
│                                                                             │
│   Si stimano velocità e accelerazione:                                      │
│                                                                             │
│   v(t) ≈ derivata prima (fitting lineare su pos.)                           │
│   a(t) ≈ derivata seconda (fitting quadratico su pos.)                      │
│                                                                             │
│   Predizione:                                                               │
│                                                                             │
│   p(t + T) = p(t) + v·T + ½·a·T²                                            │
│                                                                             │
│   ✓ Cattura accelerazioni/decelerazioni                                     │
│   ✓ Migliore per oggetti in frenata o accelerazione                         │
│   ✗ Ancora lineare nelle derivate                                           │
│   ✗ Richiede più dati storici stabili                                       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3. Filtro di Kalman

Approccio più sofisticato che modella esplicitamente lo stato e l'incertezza:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    FILTRO DI KALMAN (stato + incertezza)                    │
│                                                                             │
│   Stato: x = [px, py, pz, vx, vy, vz]ᵀ                                      │
│   (estensibile a: [..., ax, ay, az]ᵀ)                                       │
│                                                                             │
│   Modello dinamico:                                                         │
│   x(k+1) = F·x(k) + w     (con w ~ N(0, Q) rumore di processo)              │
│                                                                             │
│   │ p(k+1) │   │ I  Δt·I │ │ p(k) │   │ q_p │                               │
│   │        │ = │         │·│      │ + │     │                               │
│   │ v(k+1) │   │ 0    I  │ │ v(k) │   │ q_v │                               │
│                                                                             │
│   Predizione:                                                               │
│   x̂(t+T) = F^n · x̂(t)     (propagazione a T = n·Δt)                         │
│                                                                             │
│   P(t+T) = Fⁿ·P(t)·(Fⁿ)ᵀ + Q_total  (covarianza propagata)                  │
│                                                                             │
│   ✓ Stima ottimale in senso statistico (per sistemi lineari)                │
│   ✓ Quantifica l'incertezza della predizione                                │
│   ✓ Fusion naturale di osservazioni rumorose                                │
│   ✗ Richiede tuning dei parametri Q, R                                      │
│   ✗ Assume modello lineare (EKF/UKF per non linearità)                      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4. Approcci Basati su Apprendimento (ML/RL)

Per scenari più complessi o comportamenti non lineari:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    APPROCCI ML-BASED (concettuale)                          │
│                                                                             │
│   • Reti neurali ricorrenti (LSTM, GRU) per sequenze temporali              │
│   • Modelli di comportamento intent-based (es. predizione di pedoni)        │
│   • Gaussian Processes per predizione con incertezza                        │
│                                                                             │
│   Input: storia di posizioni + contesto (tipo oggetto, ambiente, ...)       │
│   Output: distribuzione su posizioni future                                 │
│                                                                             │
│   ✓ Può catturare pattern complessi e non lineari                           │
│   ✓ Adattamento a scenari specifici                                         │
│   ✗ Richiede dati di training                                               │
│   ✗ Maggior costo computazionale                                            │
│   ✗ Difficile da integrare in sistemi hard real-time                        │
│                                                                             │
│   ⚠️ Applicabilità limitata nel caso di ostacoli generici                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Raccomandazione

Per un'implementazione iniziale nel contesto del `cartesian_velocity_controller`, il **Filtro di Kalman** rappresenta un buon compromesso:

- Fornisce stima della velocità filtrata (meno rumore)
- Propaga automaticamente l'incertezza
- Costo computazionale accettabile
- Ben compreso e facile da debuggare

---

## Integrazione con la Mappa 3D

### Strategia di Fusione

Una volta ottenute le predizioni, bisogna decidere **come integrare** lo stato predetto nella mappa 3D. Esistono diverse strategie:

### 1. Approccio con Ostacoli Duplicati

Si aggiungono alla scena sia gli ostacoli nella posizione corrente che "fantasmi" nelle posizioni predette:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    STRATEGIA: DUAL REPRESENTATION                           │
│                                                                             │
│   Per ogni ostacolo con ID "obs_i":                                         │
│                                                                             │
│   ┌───────────────┐            ┌───────────────┐                            │
│   │  obs_i        │            │  obs_i_pred   │                            │
│   │  (corrente)   │            │  (predetto)   │                            │
│   │  pos: p(t)    │────────────│  pos: p(t+T)  │                            │
│   │  radius: r    │   prediz.  │  radius: r'   │                            │
│   └───────────────┘            └───────────────┘                            │
│                                                                             │
│   Voxelizzazione: entrambi gli ostacoli vengono voxelizzati                 │
│   EDT: calcolato sull'unione                                                │
│                                                                             │
│   ✓ Semplice da implementare                                                │
│   ✓ Non richiede modifiche all'EDT                                          │
│   ✗ Può creare "tunnel" tra posizione attuale e predetta                    │
│   ✗ Non modella il corridoio di movimento                                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2. Approccio con Swept Volume

Si considera l'intero volume spazzato dall'ostacolo tra t e t+T:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    STRATEGIA: SWEPT VOLUME                                  │
│                                                                             │
│                    ╭─────────────────────────────────────╮                  │
│                   ╱       Swept Volume (capsula)          ╲                 │
│    ●────────────────────────────────────────────●                           │
│  p(t)                                        p(t+T)                         │
│                                                                             │
│   Il volume occupato è l'unione di tutte le sfere lungo la traiettoria      │
│   → per v costante, equivale a una "capsula" (cilindro + 2 semisfere)       │
│                                                                             │
│   Voxelizzazione: richiede un voxelizer per capsule                         │
│                                                                             │
│   ✓ Conservativo: copre tutto il movimento                                  │
│   ✓ Rappresentazione realistica del pericolo                                │
│   ✗ Più complesso da voxelizzare                                            │
│   ✗ Può essere troppo conservativo (blocca aree sicure)                     │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3. Approccio Multi-Frame Temporale

Si mantengono più "snapshot" a diversi istanti futuri:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    STRATEGIA: MULTI-HORIZON                                 │
│                                                                             │
│   Grid @ t+0.1s:   ●                                                        │
│                      ↘                                                       │
│   Grid @ t+0.3s:       ●                                                    │
│                          ↘                                                   │
│   Grid @ t+0.5s:           ●                                                │
│                                                                             │
│   Il controller può consultare la mappa a diversi orizzonti:                │
│   - Breve termine (t+0.1s): pianificazione immediata                        │
│   - Medio termine (t+0.5s): strategia di evitamento                         │
│                                                                             │
│   ✓ Flessibilità nel controller                                             │
│   ✓ Permette strategie adattive                                             │
│   ✗ Moltiplica il costo computazionale (N griglie × EDT)                    │
│   ✗ Maggior uso di memoria                                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4. Approccio con Inflazione Dinamica (consigliato)

Si utilizza una singola mappa, ma il raggio degli ostacoli viene inflato in base alla velocità e all'incertezza:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    STRATEGIA: DYNAMIC INFLATION (consigliata)               │
│                                                                             │
│   Per ogni ostacolo in movimento:                                           │
│                                                                             │
│   r_effective = r_base + r_margin + r_velocity + r_uncertainty              │
│                                                                             │
│   dove:                                                                     │
│   • r_base:        raggio originale della sfera                             │
│   • r_margin:      margine di sicurezza statico (cfg.obstacle_margin)       │
│   • r_velocity:    ||v|| · T_horizon (distanza percorsa nell'orizzonte)     │
│   • r_uncertainty: k · σ (incertezza dalla covarianza del Kalman)           │
│                                                                             │
│                                                                             │
│              ┌─────────────────────┐                                        │
│              │   ╭───────────────╮ │                                        │
│              │  ╱   ╭───────╮    ╲│                                        │
│              │ │   ╱  ●     ╲    ││                                        │
│              │ │  │ base     │   ││                                        │
│              │ │   ╲        ╱    ││                                        │
│              │  ╲   ╰───────╯   ╱ │                                        │
│              │   ╰─────────────╯  │  ← r_velocity                          │
│              └─────────────────────┘  ← r_uncertainty                      │
│                                                                             │
│   ✓ Singola mappa (no overhead computazionale)                              │
│   ✓ Integrazione naturale con sistema esistente                             │
│   ✓ L'incertezza si traduce direttamente in sicurezza                       │
│   ✗ Approssimazione (non distingue direzione del movimento)                 │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Gestione dell'Incertezza

### Perché l'Incertezza è Importante

La predizione del futuro è intrinsecamente incerta. Questa incertezza deve essere **quantificata e propagata** per permettere decisioni di controllo appropriate.

### Fonti di Incertezza

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    FONTI DI INCERTEZZA                                      │
│                                                                             │
│   1. RUMORE DI OSSERVAZIONE                                                 │
│      • Precisione del sensore                                               │
│      • Latenza variabile nella pipeline                                     │
│      • Errori di segmentazione/riconoscimento                               │
│                                                                             │
│   2. ERRORE DEL MODELLO                                                     │
│      • L'ostacolo cambia comportamento (es. inverte direzione)              │
│      • Il modello è troppo semplice (lineare vs. realtà non lineare)        │
│      • Interazioni con altri oggetti non modellate                          │
│                                                                             │
│   3. PROPAGAZIONE TEMPORALE                                                 │
│      • L'incertezza cresce con l'orizzonte temporale                        │
│      • Effetto "cono di incertezza"                                         │
│                                                                             │
│                         Incertezza                                          │
│                            σ                                                │
│                            ▲                                                │
│                            │       ╱                                        │
│                            │     ╱                                          │
│                            │   ╱                                            │
│                            │ ╱                                              │
│                            │────────────► T_horizon                         │
│                            0                                                │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Strategie di Utilizzo dell'Incertezza

| Strategia | Descrizione |
|-----------|-------------|
| **Inflazione proporzionale** | Raggio ostacolo += k · σ (es. k=2 → 95% coverage) |
| **Soglia di confidenza** | Usa predizione solo se σ < σ_max |
| **Peso nel controllo** | Velocità repulsiva proporzionale a 1/σ |
| **Decay temporale** | Peso della predizione decresce con σ |

---

## Considerazioni sull'Orizzonte Temporale

### Scelta dell'Orizzonte di Predizione

L'**orizzonte temporale** T_horizon è un parametro critico:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    TRADE-OFF DELL'ORIZZONTE TEMPORALE                       │
│                                                                             │
│   T_horizon PICCOLO (es. 0.1s):                                             │
│   ✓ Predizione più affidabile                                               │
│   ✓ Minore incertezza                                                       │
│   ✗ Poco tempo per reagire                                                  │
│   ✗ Simile all'approccio reattivo                                           │
│                                                                             │
│   T_horizon GRANDE (es. 1.0s):                                              │
│   ✓ Più tempo per pianificare manovre                                       │
│   ✓ Movimenti più fluidi                                                    │
│   ✗ Maggiore incertezza                                                     │
│   ✗ Rischio di falsi positivi (evitare ostacoli che cambieranno direzione) │
│                                                                             │
│   ─────────────────────────────────────────────────────────────────────     │
│                                                                             │
│   SUGGERIMENTO:                                                             │
│   • T_horizon_min: max(latenza_sistema, 2 × ciclo_controllo)                │
│   • T_horizon_max: distanza_sicurezza / v_tipica_ostacolo                   │
│   • Valore tipico: 0.2s - 0.5s per scenari indoor con ostacoli lenti        │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Orizzonte Adattivo

Un approccio avanzato prevede un **orizzonte adattivo** basato su:

- **Velocità dell'ostacolo**: ostacoli più veloci → orizzonte più lungo
- **Distanza dall'ostacolo**: ostacoli vicini → orizzonte più corto (reazione immediata)
- **Confidenza della predizione**: alta incertezza → orizzonte più corto

---

## Impatto sul Controllo

### Modifica della Forza Repulsiva

Il controller attuale calcola una velocità repulsiva basata sulla distanza. Con la predizione, questa può essere estesa:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    IMPATTO SULLA VELOCITÀ REPULSIVA                         │
│                                                                             │
│   APPROCCIO ATTUALE (reattivo):                                             │
│                                                                             │
│   v_rep = f(d_current) · gradient                                           │
│                                                                             │
│   dove d_current è la distanza dall'EDT                                     │
│                                                                             │
│   ─────────────────────────────────────────────────────────────────────     │
│                                                                             │
│   APPROCCIO PREDITTIVO (proposto):                                          │
│                                                                             │
│   v_rep = f(d_effective) · gradient_effective                               │
│                                                                             │
│   dove:                                                                     │
│   • d_effective = distanza dalla mappa che include ostacoli predetti        │
│   • gradient_effective = direzione di fuga dalla configurazione futura     │
│                                                                             │
│   OPPURE (combinazione):                                                    │
│                                                                             │
│   v_rep = α · v_rep_current + (1-α) · v_rep_predicted                       │
│                                                                             │
│   con α ∈ [0,1] parametro di blending                                       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Vantaggi per la Fluidità del Movimento

La predizione permette di:

1. **Iniziare manovre evasive prima**: il robot si sposta gradualmente invece che di colpo
2. **Evitare oscillazioni**: conoscendo la traiettoria dell'ostacolo, si evita di "inseguirlo"
3. **Scegliere direzioni migliori**: evitare di spostarsi verso dove l'ostacolo sta andando

---

## Sfide e Compromessi

### Sfide Principali

| Sfida | Descrizione | Mitigazione |
|-------|-------------|-------------|
| **Data association** | Collegare osservazioni allo stesso ostacolo nel tempo | Usare ID dalla PlanningScene; se non disponibile, nearest-neighbor con soglia |
| **Ostacoli nuovi** | Nessuna storia disponibile | Fall-back a modello statico per primi N cicli |
| **Ostacoli persi** | Scompaiono temporaneamente | Mantenere "memoria" per M cicli, poi rimuovere |
| **Costo computazionale** | Kalman + predizione per ogni ostacolo | Filtrare ostacoli lontani; usare modelli semplici |
| **Tuning parametri** | Q, R del Kalman, T_horizon | Test empirici; dynamic reconfigure per tuning runtime |

### Compromessi Architetturali

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    COMPROMESSI DA CONSIDERARE                               │
│                                                                             │
│   1. COMPLESSITÀ vs. BENEFICIO                                              │
│      • Per ostacoli lenti/statici, la predizione aggiunge poco              │
│      • Valutare se lo scenario richiede realmente predizione                │
│                                                                             │
│   2. CONSERVATIVITÀ vs. AGILITÀ                                             │
│      • Mappa troppo inflata → robot bloccato più spesso                     │
│      • Mappa poco conservativa → rischio collisione                         │
│                                                                             │
│   3. LATENZA vs. ACCURATEZZA                                                │
│      • Modelli complessi → risultati migliori ma più lenti                  │
│      • Trade-off con real-time requirements del controller                  │
│                                                                             │
│   4. ACCOPPIAMENTO MODULI                                                   │
│      • Integrazione stretta col sistema esistente?                          │
│      • O modulo separato che "preprocessa" gli ostacoli?                    │
│      → Consigliato: modulo separato per flessibilità                        │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Conclusioni

L'introduzione di un sistema di predizione della posizione degli ostacoli rappresenta un'evoluzione significativa dell'architettura attuale, passando da un paradigma **puramente reattivo** a uno **proattivo**. 

### Punti Chiave

1. **Il tracking storico** degli ostacoli è la base: senza storia, non c'è predizione
2. **Il Filtro di Kalman** offre il miglior rapporto costo/beneficio per un'implementazione iniziale
3. **L'inflazione dinamica** è la strategia di fusione più semplice da integrare
4. **L'orizzonte temporale** deve essere scelto considerando latenze e velocità tipiche
5. **L'incertezza** deve essere quantificata e usata per modulare la conservatività

### Passi Successivi (Concettuali)

1. Implementare il tracking storico degli ostacoli (buffer circolare per ID)
2. Aggiungere stima della velocità (differenze finite → Kalman)
3. Modificare il Voxelizer per inflare in base alla velocità stimata
4. Validare con scenari simulati a velocità crescenti
5. Tuning dei parametri con dynamic reconfigure

---

*Documento generato il: 2026-01-26*

*Riferimento all'architettura esistente: vedere `01_Map3D_Architecture.md`*
