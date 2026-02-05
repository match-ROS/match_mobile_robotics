

---

## 1. Architettura Target

La nuova pipeline è strutturata su 4 livelli gerarchici:

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                            NUOVA ARCHITETTURA PIPELINE                               │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│  ┌─────────────────────────────────────────────────────────────────────────────┐    │
│  │  LIVELLO A: GLOBAL PLANNER                                                   │    │
│  │  ────────────────────────────────────────────────────────────────────────── │    │
│  │  • Lista di waypoint: [Waypoint_0, Waypoint_1, ..., Waypoint_N]             │    │
│  │  • Mantiene indice del "Waypoint Attivo Corrente"                           │    │
│  │  • Switch al waypoint successivo quando distanza < soglia                   │    │
│  │  • Se singolo waypoint → rimane come target                                 │    │
│  │                                                                              │    │
│  │  OUTPUT: Waypoint_attivo (Pose 6D)                                          │    │
│  └───────────────────────────────────┬─────────────────────────────────────────┘    │
│                                      │                                               │
│                                      ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────────────┐    │
│  │  LIVELLO B: LOCAL PLANNER (Punto Virtuale Intelligente)                     │    │
│  │  ────────────────────────────────────────────────────────────────────────── │    │
│  │                                                                              │    │
│  │  1. VELOCITÀ ATTRATTIVA (verso Waypoint)                                    │    │
│  │     V_goal = Waypoint_attivo - Pos_corrente                                 │    │
│  │                                                                              │    │
│  │  2. VELOCITÀ REPULSIVA OSTACOLI (dal TCP/Payload)                           │    │
│  │     V_obs = Σ (Pos_corrente - Ostacolo_i) / distanza_i²                     │    │
│  │                                                                              │    │
│  │  3. VELOCITÀ REPULSIVA LINK POI (Skeleton Repulsion)                        │    │
│  │     Per ogni POI (gomito, polso, braccio):                                  │    │
│  │       V_link = Σ (POI_pos - Ostacolo_i) / distanza_i²                       │    │
│  │     [Convertita in joint space con pseudo-inversa non smorzata parziale]
        E poi riconvertita in velocità cartesiana applicata al tcp tramite il jacobiano 
        Totale e poi viene passata alla fase di combinazione successiva
│  │                                                                              │    │
│  │  4. COMBINAZIONE                                                             │    │
│  │     V_desired = k_att · V_goal + k_rep · V_obs + k_rep_link · V_link        │    │
│  │                                                                              │    │
│  │  5. INTEGRAZIONE (Posizione Target Raw)                                     │    │
│  │     P_target_raw = P_precedente + V_desired · dt                            │    │
│  │                                                                              │    │
│  │  OUTPUT: P_target_raw, V_desired          │    │
        Quindi il P_target_raw è influenzato viene mosso dai campi artificiali
        é il punto in cui teoricamente vorrei, con quella velocità
│  └───────────────────────────────────┬─────────────────────────────────────────┘    │
│                                      │                                               │
│                                      ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────────────┐    │
│  │  LIVELLO C: MOTION GENERATOR (Filtro Smorzatore)                            │    │
│  │  ────────────────────────────────────────────────────────────────────────── │    │
│  │                                                                              │    │
│  │  Filtro del Secondo Ordine con Costante di Tempo τ                          │    │
│  │                                                                              │    │
│  │  a_desired = (V_desired - V_corrente) / τ                                   │    │
│  │  jerk = (a_desired - a_corrente) / dt                                       │    │
│  │                                                                              │    │
│  │  Limiti applicati:                                                           │    │
│  │    • |jerk| ≤ max_jerk                                                      │    │
│  │    • |accelerazione| ≤ max_acceleration                                     │    │
│  │    • |velocità| ≤ max_velocity                                              │    │
│  │                                                                              │    │
│  │  OUTPUT: V_filtrata, P_target_filtrato
        Siccome so che non ce la farò, mi accontento di essere nel punto filtrato con
        la velocità filtrata
│  └───────────────────────────────────┬─────────────────────────────────────────┘    │
│                                      │                                               │
│                                      ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────────────┐    │
│  │  LIVELLO D: PID + IK (con Feed Forward)                                     │    │
│  │  ────────────────────────────────────────────────────────────────────────── │    │
│  │                
        A questo punto mi ritrovo con un comando di feed forward che dovrebbe essere
        Quasi il totale del mio comando. Nel mio pid devo comunque saturare il comando
        massimo.
        1. FEED FORWARD                                                             │    │
│  │     V_ff = V_filtrata (dal Motion Generator) 

        La parte rimanente di comando se non si è arrivati a saturazione
        Può essere data prima al proporzionale e successivamente all'integrativa e 
        al derivativo. Nel codice ci dovrebbe essere implementata la logica degli estremi dinamici per quanto riguarda la saturazione. Anche per quanto riguarda il reset
        dell'integrale
│  │  2. CONTROLLO PID                                                            │    │
│  │     errore = P_target_filtrato - P_corrente                                 │    │
│  │     V_pid = Kp · errore + Ki · ∫errore + Kd · d(errore)/dt                  │    │
│  │                                                                              │    │
│  │                                 │    │
│  │                                                                              │    │
│  │  
        Ottenuto il mio comando vado a calcolare il valore delle velocità dei giunti come sotto
│  │                                                                              │    │
│  │  3. CONVERSIONE IK (Pseudo-inversa smorzata)                                │    │
│  │     q̇_cmd = J⁺_damped · V_cartesiana                                        │    │
│  │                                                                              │                                       │    │
│  │                                                                              │    │
│  │  OUTPUT: q̇_cmd (pre-limiting)                                             │    │
│  └───────────────────────────────────┬─────────────────────────────────────────┘    │
│                                      │                                               │
│                                      ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────────────┐    │
│  │  SAFETY LIMITER (Joint Space Scaling)                                       │    │
│  │  ────────────────────────────────────────────────────────────────────────── │    │
│  │                                                                              │    │
│  │  Scala UNIFORMEMENTE tutto il vettore se un giunto viola i limiti:          │    │
│  │                                                                              │    │
│  │  scaling_factor = 1.0                                                        │    │
│  │                                                                              │    │
│  │  // Check velocità                                                           │    │
│  │  for each joint i:                                                           │    │
│  │    if |q̇_cmd[i]| > max_joint_vel[i]:                                        │    │
│  │      factor = max_joint_vel[i] / |q̇_cmd[i]|                                 │    │
│  │      scaling_factor = min(scaling_factor, factor)                           │    │
│  │                                                                              │    │
│  │  // Check accelerazione                                                      │    │
│  │  for each joint i:                                                           │    │
│  │    acc = (q̇_cmd[i] - q̇_prev[i]) / dt                                        │    │
│  │    if |acc| > max_joint_acc[i]:                                              │    │
│  │      v_max_reachable = |q̇_prev[i]| + max_joint_acc[i] * dt                  │    │
│  │      factor = v_max_reachable / |q̇_cmd[i]|                                  │    │
│  │      scaling_factor = min(scaling_factor, factor)                           │    │
│  │                                                                              │    │
│  │  q̇_final = q̇_cmd × scaling_factor                                          │    │
│  │                                                                              │    │
│  │  OUTPUT: q̇_final → joint_group_vel_controller                               │    │
│  └─────────────────────────────────────────────────────────────────────────────┘    │
│                                                                                      │
└─────────────────────────────────────────────────────────────────────────────────────┘
```


**Formula Velocità Repulsiva Quadratica:**
```
V_rep = Σ [ k_rep * (P_current - P_obstacle) / ||P_current - P_obstacle||² ]

Solo se distanza < influence_distance
```





Quello che vorrei che facessi è dare una ripulita profonda a tutti questi file perché ci sono funzioni che non servono per seguire questa logica, e molti file possono essere semplificati. Per il momento non modificare il controllo fuzzy, ci lavorerò prossimamente, eventualmente fai modifiche che ti permettano di compilare il codice. In questo momento lascerei il controllo fuzzy disabilitato se dovessi usare questo pacchetto.
fai pulizia anche al debug e a tutto quello che non è necessario. Voglio tenere solo lo stretto indispensabile per poter utilizzare la pipiline sopra riportata. Eventualmente ristruttura il codice se lo ritieni opportuno


