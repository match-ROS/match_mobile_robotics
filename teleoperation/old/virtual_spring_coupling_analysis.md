# Analisi dell'architettura bilaterale e proposta di accoppiamento a molla virtuale

## 1. Stato attuale dell'architettura

### Master (`teleop_master_haptic_controller`)
Ammettenza massa–smorzatore **senza termine elastico** ($K=0$):

$$
M \dot{v} + D v = F_{\text{hand}} - F_{\text{fb}}
$$

Dove il feedback di forza è la semplice riflessione del wrench FT dello slave:

$$
F_{\text{fb}} = -k_f F_s + F_c
$$

> [!WARNING]
> Non c'è **nessun legame posizionale** tra master e slave. Il feedback è puramente basato sulla forza misurata dallo slave, il che significa che il master non "sente" quanto la posa dello slave si sia discostata rispetto al suo obiettivo.

### Slave (`teleop_slave_twist_outer_loop`)

Loop esterno: PID su errore di posa + feedforward + compliance:

$$
v_{\text{cmd}} = \alpha \cdot k_{\text{ff}} \cdot v_{\text{ff}} + K_p e_p + k_{\text{adm}} \cdot (-F_s)
$$

---

## 2. Problemi identificati nella riflessione di forza

| Problema | Causa | Effetto |
|---|---|---|
| **Deriva posizionale** | Nessun termine $K$ (molla): il master non ha alcuna nozione della divergenza di posizione | Master e slave possono divergere gradualmente senza che l'operatore lo percepisca |
| **Ritorno lento a riposo** | Solo lo smorzatore $D$ frena il master quando viene rilasciato $F_{\text{hand}}=0$ | Il master non tende a tornare verso la posa dello slave — si ferma dove la velocità si annulla |
| **Feedback indiretto** | Il feedback è la forza *di contatto* dello slave, non l'errore di tracking | L'operatore sente la forza di contatto *dopo* il fatto, non una forza proporzionale a "quanto è lontano lo slave" |
| **Trasparenza bassa** | Senza accoppiamento posizionale, in aria il sistema è completamente disaccoppiato | L'operatore non percepisce alcuna resistenza quando master e slave non sono allineati |

---

## 3. Proposta: architettura con molla virtuale bilaterale

L'idea è aggiungere un **termine elastico (molla virtuale)** basato sull'errore di posizione tra master e slave, su **entrambi i lati**.

### 3.1 Nuove equazioni — Master

$$
M \dot{v} + D v = F_{\text{hand}} \underbrace{- K_s (p_m - p_s)}_{\text{molla virtuale}} \underbrace{- k_f F_s}_{\text{riflessione FT}} + F_c
$$

Dove:
- $p_m$ = posizione corrente TCP master (base_link)
- $p_s$ = posizione corrente TCP slave (già disponibile via TF + [slaveTargetTick](file:///home/pantanetti/catkin_ws/src/match_mobile_robotics/teleoperation/src/teleop_master_haptic_controller_node.cpp#366-418))
- $K_s$ = rigidità della molla virtuale (N/m), parametro **nuovo**

**Effetto fisico**: quando il master si muove e lo slave resta indietro (per inerzia, limiti di velocità, o contatto con ostacolo), il termine $-K_s (p_m - p_s)$ genera una forza che rallenta/tira indietro il master, dando all'operatore un feedback haptico proporzionale alla divergenza.

### 3.2 Nuove equazioni — Slave

$$
v_{\text{cmd}} = \alpha \cdot k_{\text{ff}} \cdot v_{\text{ff}} + K_p e_p + k_{\text{adm}} (-F_s) \underbrace{+ K_s^{\text{slave}} (p_m - p_s)}_{\text{molla virtuale slave}}
$$

Oppure, dato che il PID sullo slave già insegue $p_m$ come target, il PID **è già la molla lato slave** ($K_p$ gioca questo ruolo). Quindi potrebbe non servire un termine aggiuntivo sullo slave — a meno che non si voglia una gain separata per il contributo elastico.

> [!IMPORTANT]
> **Lato slave**, il PID già funge da "molla" (è proporzionale all'errore di posa $e_p = p_{\text{target}} - p_s$ dove $p_{\text{target}} = p_m$). Il termine nuovo è quindi **principalmente lato master**, dove attualmente manca del tutto.

### 3.3 Damping della molla (opzionale)

Per evitare oscillazioni della molla virtuale, si può aggiungere un termine di smorzamento proporzionale alla differenza di velocità:

$$
F_{\text{spring}} = -K_s (p_m - p_s) - B_s (\dot{p}_m - \dot{p}_s)
$$

Questo crea un sistema **massa-molla-smorzatore** completo tra le due "estremità" (master e slave).

### 3.4 Schema riassuntivo

```
┌─────────────┐        Molla virtuale          ┌─────────────┐
│   MASTER    │◄────── K_s·(pm - ps) ──────────►│   SLAVE     │
│  M·dv + D·v │      + B_s·(vm - vs)            │  PID + FF   │
│  = F_hand   │                                  │  + compliance│
│  - F_spring │     Riflessione FT classica       │             │
│  - kf·Fs    │◄────── kf · Fs ─────────────────│  FT sensor  │
└─────────────┘                                  └─────────────┘
     ▲ v_cmd                                          ▲ v_cmd
     │                                                │
  Master HW                                       Slave HW
```

---

## 4. Requisiti di implementazione

### Cosa serve per il master:
1. **Sottoscrivere la posa TCP dello slave** (già disponibile come topic TF, basta un `lookupTransform`)
2. **Conoscere la propria posizione TCP** (di nuovo TF: `slave_base_frame_` → `slave_tcp_frame_`, e stessa cosa per il master)
3. **Nuovi parametri**: `spring_stiffness_linear`, `spring_damping_linear`, (e opzionalmente `spring_stiffness_angular`, `spring_damping_angular`)
4. **Computare** $\Delta p = p_m - p_s$ e $\Delta v = v_m - v_s$ nel loop [tick()](file:///home/pantanetti/catkin_ws/src/match_mobile_robotics/teleoperation/src/teleop_slave_twist_outer_loop_node.cpp#327-553)

### Cosa serve per lo slave:
- **Niente di obbligatorio**, perché il PID già funge da molla posizionale. Eventualmente si potrebbe voler separare i guadagni PID dai guadagni della molla virtuale, ma è una scelta di design, non una necessità.

### Attenzioni:
- La molla e il ritardo del canale di comunicazione possono generare **oscillazioni**. È fondamentale avere $B_s > 0$ (smorzamento) e possibilmente una saturazione sulla forza della molla (`max_spring_force`).
- Il parametro $K_s$ deve essere **basso** (ordine $1\text{–}50\ \text{N/m}$) per evitare instabilità passiva.
- La posizione dello slave arriva via TF e quindi ha un **ritardo intrinseco**. Con $K_s$ troppo alto questo genera oscillazioni.

---

## 5. Vantaggi attesi

| Aspetto | Stato attuale | Con molla virtuale |
|---|---|---|
| **Feedback in aria** | Nessuno: master e slave disaccoppiati | L'operatore sente tensione proporzionale alla divergenza |
| **Deriva posizionale** | Si accumula senza avviso | La molla tende a riallineare master e slave |
| **Trasparenza al contatto** | Solo forza FT | Forza FT + molla: l'operatore sente sia il contatto sia la divergenza |
| **Ritorno a riposo** | Lento (solo damping) | La molla riporta il master verso lo slave quando l'operatore rilascia |

---

## 6. Raccomandazione

Procedere con l'implementazione del **termine molla virtuale sul master** come primo passo, con i seguenti parametri configurabili:

```yaml
# Virtual spring coupling (master → slave position feedback)
spring_stiffness_linear: 10.0    # N/m — iniziare basso
spring_damping_linear: 2.0       # N·s/m — smorzamento
spring_stiffness_angular: 0.0    # N·m/rad — disabilitato inizialmente
spring_damping_angular: 0.0
max_spring_force: 20.0           # N — saturazione sicurezza
max_spring_torque: 5.0           # N·m
# Master TCP frames (per calcolare p_master)
master_base_frame: base_link
master_tcp_frame: tool0
```

Sul lato slave, il PID esistente è sufficiente ma si potrebbe voler aumentare leggermente $K_p$ (attualmente 1.0) per rendere lo slave più reattivo alla posa target.

**Vuoi che proceda con l'implementazione del termine molla virtuale sul nodo master?** Posso anche creare un file di implementazione plan dettagliato con i cambiamenti codice specifici.
