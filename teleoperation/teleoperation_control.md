# Architettura di Controllo Teleoperazione Bimanuale (Velocity-Velocity)

Questo documento descrive la formulazione matematica per un sistema di teleoperazione bilaterale dove **sia il Master (es. UR10e) che lo Slave** sono controllati tramite comandi di velocità (Controllo di Ammettenza).

## 1. Definizioni e Variabili

### Indici
* **$m$**: Master (lato operatore/UR10e)
* **$s$**: Slave (lato remoto)
* **$L, R$**: Braccio Sinistro (Left) e Destro (Right)

### Variabili di Stato
* $x, \dot{x}, \ddot{x} \in \mathbb{R}^6$: Posizione, Velocità, Accelerazione Cartesiana (Spazio Operativo).
* $q, \dot{q} \in \mathbb{R}^n$: Posizione e Velocità dei Giunti.
* $J(q)$: Jacobiano del robot.

### Forze
* $F_{hand}$: Forza applicata dalla mano dell'operatore sul Master (misurata dal sensore F/T del UR10e).
* $F_{ext}$: Forza di interazione ambientale misurata dal sensore dello Slave.
* $F_{net}$: Forza netta risultante per il calcolo della dinamica virtuale.

---

## 2. Controllo del Master (Admittance Control)
Poiché il Master è controllato in velocità, simuliamo una dinamica massa-smorzatore virtuale per generare il feedback aptico. L'operatore "spinge" la massa virtuale, e il feedback dallo slave la "frena".

### Dinamica Virtuale
L'equazione che governa il comportamento del Master è:

$$M_m \ddot{x}_m + D_m \dot{x}_m = F_{hand} - K_{scale} F_{ext}$$

Dove:
* $M_m$: Matrice di Massa virtuale (inerzia percepita dall'utente).
* $D_m$: Matrice di Smorzamento virtuale (stabilizza il movimento).
* $K_{scale}$: Fattore di scala della forza (se voglio sentire il 50% della forza reale, $K=0.5$).

### Calcolo del Comando di Velocità (Loop Discreto)
Ad ogni ciclo di controllo $\Delta t$, calcoliamo la velocità cartesiana target per il Master:

1.  **Calcolo Accelerazione target:**
    $$\ddot{x}_{cmd} = M_m^{-1} \left( F_{hand} - K_{scale} F_{ext} - D_m \dot{x}_{m,prev} \right)$$

2.  **Integrazione (Eulero):**
    $$\dot{x}_{m,cmd} = \dot{x}_{m,prev} + \ddot{x}_{cmd} \Delta t$$

3.  **Mapping ai Giunti:**
    $$\dot{q}_{m,cmd} = J_m^{-1}(q_m) \cdot \dot{x}_{m,cmd}$$

> **Nota:** $F_{hand}$ deve essere depurata dalla gravità dell'eventuale maniglia/tool montato sul sensore del Master (Gravity Compensation del tool).

---

## 3. Controllo dello Slave (Position-Force Tracking)
Lo Slave deve seguire la posizione del Master, ma deve mostrare "cedevolezza" (compliance) se incontra un ostacolo, per evitare danni.

### Legge di Controllo
La velocità comandata allo Slave è una combinazione di inseguimento di traiettoria e reazione alle forze esterne:

$$\dot{x}_{s,cmd} = \underbrace{K_p (x_m - x_s)}_{\text{Errore Posizione}} + \underbrace{\dot{x}_m}_{\text{Feedforward}} - \underbrace{K_{adm} F_{ext}}_{\text{Cedevolezza}}$$

Dove:
* $K_p$: Guadagno proporzionale di posizione (rigidità del tracking in aria libera).
* $K_{adm}$: Guadagno di ammettenza (quanto il robot rallenta/indietreggia quando sente una forza).

### Saturazione e Conversione
$$\dot{q}_{s,cmd} = J_s^{-1}(q_s) \cdot \text{Saturate}(\dot{x}_{s,cmd}, V_{max})$$

---

## 4. Coordinamento Bimanuale (Coupling)
Per gestire due braccia che manipolano un oggetto comune, introduciamo un termine di forza correttivo basato sulla distanza relativa.

Definiamo l'errore di distanza relativa (compressione/trazione indebita):
$$\epsilon_{rel} = \| x_{s,L} - x_{s,R} \| - \| x_{m,L} - x_{m,R} \|$$

### Forza di Accoppiamento Virtuale
Se $|\epsilon_{rel}|$ supera una soglia (i robot slave sono più vicini o più lontani dei master, indicando stress sull'oggetto), generiamo forze repulsive/attrattive aggiuntive sui Master.

Forza aggiuntiva da sommare a $F_{ext}$ nell'equazione del Master:

$$F_{couple, L} = K_{virt} \cdot \epsilon_{rel} \cdot \vec{u}_{L \to R}$$
$$F_{couple, R} = K_{virt} \cdot \epsilon_{rel} \cdot \vec{u}_{R \to L}$$

Dove $\vec{u}$ è il vettore unitario che congiunge le due mani.

---

## 5. Schema Riassuntivo del Flusso Dati

Per ogni lato (Sinistro/Destro):

1.  **INPUT MASTER:** Leggi $q_m, \dot{q}_m$ e $F_{hand}$ (Sensore Master).
2.  **INPUT SLAVE:** Leggi $q_s, \dot{q}_s$ e $F_{ext}$ (Sensore Slave).
3.  **KINEMATICS:** Calcola $x_m, x_s$ (Cinematica Diretta) e $J_m, J_s$.
4.  **LOGICA SLAVE:**
    $$v_{slave} = K_p(x_m - x_s) + \dot{x}_m - K_{adm} F_{ext}$$
    $$\dot{q}_{s,cmd} = J_s^{-1} v_{slave}$$
5.  **LOGICA MASTER:**
    $$F_{feedback} = K_{scale} F_{ext} + F_{couple}$$
    $$a_{master} = M_m^{-1} (F_{hand} - F_{feedback} - D_m \dot{x}_m)$$
    $$v_{master} = \dot{x}_{m,prev} + a_{master} \Delta t$$
    $$\dot{q}_{m,cmd} = J_m^{-1} v_{master}$$
6.  **OUTPUT:** Invia $\dot{q}_{s,cmd}$ allo Slave e $\dot{q}_{m,cmd}$ al Master.