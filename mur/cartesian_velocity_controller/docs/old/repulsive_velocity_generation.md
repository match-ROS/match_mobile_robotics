# Generazione delle Velocità Repulsive

Questo documento descrive in dettaglio come vengono generate le velocità repulsive nel sistema di controllo, le formule matematiche utilizzate e la gestione degli ostacoli.

---

## Indice

1. [Panoramica del Sistema](#panoramica-del-sistema)
2. [Tipi di Ostacoli e Punti di Interesse (POI)](#tipi-di-ostacoli-e-punti-di-interesse-poi)
3. [Calcolo della Distanza Effettiva](#calcolo-della-distanza-effettiva)
4. [Formule per le Velocità Repulsive](#formule-per-le-velocità-repulsive)
5. [Velocità Repulsive per i Link POI](#velocità-repulsive-per-i-link-poi)
6. [Combinazione delle Velocità](#combinazione-delle-velocità)
7. [Parametri Configurabili](#parametri-configurabili)

---

## Panoramica del Sistema

Il sistema implementa un approccio basato su **Potential Field** (campo potenziale) per la navigazione reattiva del robot, combinando:

- **Velocità attrattiva**: dirige il TCP verso il waypoint desiderato
- **Velocità repulsiva ostacoli (TCP)**: allontana il TCP dagli ostacoli vicini
- **Velocità repulsiva link (POI)**: genera velocità articolari per allontanare i punti sensibili sui link del robot dagli ostacoli

### Flusso dei Dati

```
                    ┌─────────────────────────┐
                    │   scene_builder         │
                    │   (RobotPointsInfo)     │
                    └───────────┬─────────────┘
                                │
                                ▼
                    ┌─────────────────────────┐
                    │ RepulsionDataManager    │
                    │ - Filtra POI abilitati  │
                    │ - Calcola d_effective   │
                    │ - Genera ObstacleInfo   │
                    │ - Genera LinkPOI        │
                    └───────────┬─────────────┘
                                │
                    ┌───────────┴───────────┐
                    │                       │
                    ▼                       ▼
            ┌───────────────┐       ┌───────────────┐
            │ ObstacleInfo  │       │   LinkPOI     │
            │ (TCP repuls.) │       │ (Link repuls.)│
            └───────┬───────┘       └───────┬───────┘
                    │                       │
                    ▼                       ▼
                    ┌─────────────────────────┐
                    │     LocalPlanner        │
                    │ - Calcola v_repulsive   │
                    │ - Applica guadagni      │
                    │ - Combina velocità      │
                    └─────────────────────────┘
```

---

## Tipi di Ostacoli e Punti di Interesse (POI)

### Configurazione dei POI

Ogni POI è configurato tramite `RepulsivePointConfig`:

| Parametro | Descrizione | Range |
|-----------|-------------|-------|
| `name` | Identificatore del POI (es. "tcp", "elbow") | - |
| `weight` | Peso per la velocità repulsiva | [0.0, 2.0] |
| `radius` | Raggio del POI (per calcolo superficie-superficie) | [0.0, 0.5] m |
| `enabled` | Se il POI è attivo | true/false |
| `is_tcp` | Se è il POI del TCP (genera ObstacleInfo) | true/false |

### POI di Default

```yaml
default_pois:
  - tcp         # Tool Center Point
  - elbow       # Gomito del robot
  - wrist       # Polso
  - forearm_mid # Centro avambraccio
```

### Due Categorie di Output

1. **ObstacleInfo** (quando `is_tcp = true`):
   - Usato per calcolare la velocità repulsiva **Cartesiana** diretta al TCP
   - La velocità risultante viene sommata direttamente alla velocità del punto virtuale

2. **LinkPOI** (quando `is_tcp = false`):
   - Usato per calcolare velocità repulsive **articolari** tramite Jacobiano parziale
   - Permette di proteggere i link intermedi del robot

---

## Calcolo della Distanza Effettiva

Il sistema utilizza una **distanza superficie-superficie** anziché centro-centro:

$$
d_{effective} = \max(0.001, \; d_{raw} - r_{poi} - r_{object})
$$

Dove:
- $d_{raw}$ = distanza centro-centro (dal messaggio RobotPointsInfo)
- $r_{poi}$ = raggio del punto di interesse configurato
- $r_{object}$ = raggio caratteristico dell'oggetto/ostacolo

### Direzione Repulsiva

La direzione repulsiva è **opposta** al vettore distanza (allontanamento dall'ostacolo):

$$
\hat{d}_{repulsive} = -\frac{\vec{d}_{vector}}{||\vec{d}_{vector}||}
$$

Dove $\vec{d}_{vector}$ è il vettore dal POI al centro dell'ostacolo.

---

## Formule per le Velocità Repulsive

Il sistema supporta **due modalità** di calcolo della magnitudine repulsiva:

### Modalità QUADRATIC (Consigliata)

Fornisce una risposta più aggressiva vicino agli ostacoli con un decadimento naturale.

**Caso 1**: $d \leq d_{min\_safe}$
$$
||\vec{v}_{rep}|| = v_{max}
$$

**Caso 2**: $d_{min\_safe} < d < d_{influence}$
$$
||\vec{v}_{rep}|| = \min\left(v_{max}, \; v_{max} \cdot \left(\frac{d_{min\_safe}}{d}\right)^2\right)
$$

**Taper Smoothing** (per transizione graduale a zero):

Se $d > 0.8 \cdot d_{influence}$:
$$
||\vec{v}_{rep}|| = ||\vec{v}_{rep}|| \cdot \frac{d_{influence} - d}{d_{influence} - 0.8 \cdot d_{influence}}
$$

### Modalità LINEAR (Legacy)

Interpolazione lineare tra distanza minima sicura e distanza di influenza.

**Caso 1**: $d \leq d_{min\_safe}$
$$
||\vec{v}_{rep}|| = v_{max}
$$

**Caso 2**: $d_{min\_safe} < d < d_{influence}$
$$
||\vec{v}_{rep}|| = v_{max} \cdot \frac{d_{influence} - d}{d_{influence} - d_{min\_safe}}
$$

### Grafico Comparativo

```
Magnitude
    ▲
    │
vmax├─────┐
    │     │\
    │     │ \          LINEAR
    │     │  \________
    │     │   \      ──────
    │     │    \              QUADRATIC
    │     │     ──────────
    └─────┴─────┬─────┬───────────► d
          d_min  0.8·d_inf  d_inf
```

### Velocità Repulsiva Finale per Ostacolo Singolo

$$
\vec{v}_{rep,i} = \hat{d}_{repulsive,i} \cdot ||\vec{v}_{rep,i}||
$$

### Velocità Repulsiva Totale (TCP)

Somma di tutti gli ostacoli con guadagno applicato:

$$
\vec{v}_{rep,TCP} = k_{rep} \cdot \sum_{i=1}^{N} \vec{v}_{rep,i}
$$

Dove $k_{rep}$ è il guadagno repulsivo (`k_repulsive_obstacle_`).

---

## Velocità Repulsive per i Link POI

Per i POI sui link intermedi, il calcolo è più complesso e coinvolge il Jacobiano.

### Step 1: Calcolo Velocità Cartesiana del POI

Per ogni LinkPOI abilitato con distanza nell'intervallo di influenza:

$$
\vec{v}_{cart,poi} = \hat{d}_{repulsive} \cdot ||\vec{v}_{rep}|| \cdot w_{poi}
$$

Dove $w_{poi}$ è il peso configurato per quel POI.

### Step 2: Jacobiano Parziale

Si calcola il Jacobiano dal frame base al link specifico, considerando l'offset del POI:

$$
J_{poi} = \text{getJacobian}(link, \vec{p}_{link})
$$

Dove $\vec{p}_{link}$ è la posizione del POI nel frame del link.

### Step 3: Conversione in Velocità Articolare

Usando lo **pseudo-inverso** del Jacobiano (solo parte lineare, prime 3 righe):

$$
J_{linear} = J_{poi}[0:3, :]
$$

$$
J^+ = J_{linear}^T \cdot (J_{linear} \cdot J_{linear}^T)^{-1}
$$

$$
\dot{q}_{rep,poi} = J^+ \cdot \vec{v}_{cart,poi}
$$

### Step 4: Somma delle Velocità Articolari

$$
\dot{q}_{rep,links} = k_{rep,link} \cdot \sum_{poi} \dot{q}_{rep,poi}
$$

Dove $k_{rep,link}$ è il guadagno repulsivo per i link (`k_repulsive_link_`).

### Step 5: Conversione a Cartesiano (per il Virtual Point)

Le velocità articolari repulsive dei link vengono convertite in velocità Cartesiana al TCP tramite il Jacobiano completo:

$$
\vec{v}_{rep,links,cart} = J_{TCP} \cdot \dot{q}_{rep,links}
$$

Si prende solo la parte lineare (prime 3 componenti).

---

## Combinazione delle Velocità

### Velocità Combinata

$$
\vec{v}_{combined,lin} = \vec{v}_{attractive,lin} + \vec{v}_{rep,TCP} + \vec{v}_{rep,links,cart}
$$

$$
\vec{v}_{combined,ang} = \vec{v}_{attractive,ang}
$$

> **Nota**: Le velocità repulsive non influenzano direttamente l'orientamento.

### Limitazione della Velocità

Le velocità vengono limitate ai valori massimi configurati:

$$
\vec{v}_{limited} = \begin{cases}
\vec{v} & \text{se } ||\vec{v}|| \leq v_{max} \\
\frac{\vec{v}}{||\vec{v}||} \cdot v_{max} & \text{altrimenti}
\end{cases}
$$

### Integrazione del Target Virtuale

Con soglie di freeze per evitare jitter:

$$
\vec{p}_{target}(t+dt) = \vec{p}_{target}(t) + \vec{v}_{combined,lin} \cdot dt
$$

L'orientamento viene integrato usando la **mappa esponenziale**:

$$
R_{target}(t+dt) = \Delta R \cdot R_{target}(t)
$$

Dove $\Delta R = \exp(\vec{\omega} \cdot dt)$.

---

## Parametri Configurabili

### Guadagni

| Parametro | Default | Descrizione |
|-----------|---------|-------------|
| `k_attractive_` | - | Guadagno attrattivo verso waypoint |
| `k_repulsive_obstacle_` | - | Guadagno repulsivo ostacoli TCP |
| `k_repulsive_link_` | - | Guadagno repulsivo link POI |

### Distanze

| Parametro | Default | Descrizione |
|-----------|---------|-------------|
| `influence_distance_` | - | Distanza massima di influenza |
| `min_safe_distance_` | - | Distanza minima sicura (velocità max) |

### Limiti Velocità

| Parametro | Default | Descrizione |
|-----------|---------|-------------|
| `max_linear_velocity_` | - | Velocità lineare massima [m/s] |
| `max_angular_velocity_` | - | Velocità angolare massima [rad/s] |
| `freeze_linear_threshold_` | - | Soglia freeze lineare |
| `freeze_angular_threshold_` | - | Soglia freeze angolare |

### Data Manager

| Parametro | Default | Descrizione |
|-----------|---------|-------------|
| `robot_points_topic` | `/robot_points_info` | Topic dati distanze |
| `stale_timeout` | 0.3 s | Timeout dati obsoleti |

---

## Diagramma di Flusso Completo

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        LocalPlanner::compute()                          │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
         ┌──────────────────────────┼──────────────────────────┐
         │                          │                          │
         ▼                          ▼                          ▼
┌─────────────────┐      ┌─────────────────┐      ┌─────────────────────┐
│ Attractive      │      │ Repulsive TCP   │      │ Repulsive Link POI  │
│ Velocity        │      │ (ObstacleInfo)  │      │ (LinkPOI)           │
├─────────────────┤      ├─────────────────┤      ├─────────────────────┤
│ v = k_att *     │      │ For each obs:   │      │ For each POI:       │
│ (waypoint -     │      │ v += k_rep *    │      │ 1. v_cart = dir*mag │
│  current)       │      │ dir * mag(d)    │      │ 2. J_poi = Jacobian │
│                 │      │                 │      │ 3. q_dot = J+ * v   │
└────────┬────────┘      └────────┬────────┘      │ 4. Sum all q_dot    │
         │                        │               │ 5. v = J_tcp * q_dot│
         │                        │               └──────────┬──────────┘
         │                        │                          │
         │                        │                          │
         └────────────────────────┼──────────────────────────┘
                                  │
                                  ▼
                        ┌─────────────────┐
                        │ v_combined =    │
                        │ v_att + v_rep_  │
                        │ tcp + v_rep_    │
                        │ links           │
                        └────────┬────────┘
                                 │
                                 ▼
                        ┌─────────────────┐
                        │ Limit Velocity  │
                        │ & Integrate     │
                        │ Virtual Target  │
                        └─────────────────┘
```

---

## Note Implementative

1. **Thread Safety**: Tutti gli accessi ai parametri e allo stato sono protetti da mutex (`params_mutex_`, `state_mutex_`, `config_mutex_`, `msg_mutex_`).

2. **Validazione Dati**: I dati sono considerati validi solo se ricevuti entro `stale_timeout` secondi.

3. **Ostacolo più Vicino per POI**: Per ogni POI viene considerato solo l'ostacolo più vicino (ottimizzazione).

4. **Pseudo-Inverso senza Damping**: Lo pseudo-inverso del Jacobiano viene calcolato senza damping per massimizzare la reattività:
   $$J^+ = J^T (J J^T)^{-1}$$

5. **Output Diagnostici**: La struttura `LocalPlannerOutput` contiene informazioni dettagliate per debug e visualizzazione, incluse le velocità repulsive calcolate per ogni POI.

