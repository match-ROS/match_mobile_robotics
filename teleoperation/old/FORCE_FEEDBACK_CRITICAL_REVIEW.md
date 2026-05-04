# Analisi Critica delle Proposte di FORCE_FEEDBACK_ANALYSIS.md

> **Contesto architetturale del sistema reale**  
> Il sistema è composto da due UR10e su un MUR620b, comunicanti al rate di **250 Hz**.  
> Il master usa un controller Admittanza (`teleop_master_haptic_controller`), lo slave usa un outer-loop
> Posizione + compliance (`teleop_slave_twist_outer_loop`).  
> Entrambi i lati pubblicano i wrench direttamente senza compensazione della dinamica del tool.

---

## 1. Deadman Switch — *"Fortemente Consigliato"*

### Cosa propone l'analisi
Disabilitare `kf_force_` / `kf_torque_` quando la norma di `F_hand` scende sotto una soglia: se l'operatore lascia la presa, il force-reflection viene annullato e la frizione virtuale alzata.

### Punti di forza
- **Concettualmente corretto**: il problema dell'instabilità da backdrivability senza operatore è reale e la logica condizionale è la soluzione più semplice.
- **Zero costo computazionale e zero parametri aggiuntivi** da tarare (basta usare la deadband master già presente).

### Criticità e limiti

#### 1.1 — L'heuristica `|F_hand|` è ambigua
La forza letta dal sensore del master è la *somma* di:
- Forza applicata dall'operatore
- Gravità/inerzia del tool (non compensata) durante il moto
- Force-reflection dello slave già filtrato

Usare `|F_hand|` come discriminante "operatore presente/assente" è **fragile**:  
un moto rapido della base mobile genera accelerazioni che fanno salire `|F_hand|` anche senza nessuno che tenga l'handle, **riattivando** il force-reflection indesiderato.

**Alternativa più robusta**: un segnale digitale esplicito (pulsante hardware / topic `/operator_present`) oppure una stima di contatto basata su cambio-segno e varianza della forza nel tempo.

#### 1.2 — Transizione discontinua
Azzerare di colpo `kf_force_` (da `0.3` a `0.0`) introduce un **gradino nella legge di controllo** che può generare un impulso di velocità sul master.  
Serve una **rampa temporizzata** (e.g. spline su 100–300 ms) oppure un filtro passa-basso sullo scalare `kf`.

#### 1.3 — Il "damping alto" in assenza di operatore regge?
Con `damping_linear = 80.0` (proposto, vs `10.0` attuale) e `mass_linear = 3.0`:  
$\tau = M/D = 3.0/80.0 \approx 37 \text{ms}$
Il sistema è **criticamente sovrasmorzato** ma la costante di tempo è molto breve: qualsiasi disturbo esterno (cavo del robot, vibrazione base) con frequenza > ~4 Hz supera comunque la frizione. Non è un problema insuperabile, ma il valore `80.0` va validato sul campo — potrebbe risultare troppo "duro" da pilotare durante la riattivazione.

---

## 2. Damping Variabile / Frenata Attiva

### Cosa propone l'analisi
Modulare `damping_linear` linearmente tra `80.0` (assenza di contatto dell'operatore) e `10.0` (operatore in presa attiva), usando la forza misurata come segnale di giudizio.

### Punti di forza
- Risolve il problema della transizione brusca del punto 1.2.
- Dà un feeling progressivo all'operatore.

### Criticità e limiti

#### 2.1 — `damping_linear` non è un parametro runtime nel codice attuale
Nel file `master_real_mur620b_ur10_l.yaml`, `damping_linear` è letto **una volta sola** in `onInit()` come valore fisso. Implementare un damping variabile richiede di:
1. Esporre un parametro `damping_linear_max` e `damping_linear_min` in YAML.
2. Ricalcolare l'integrazione dell'admittanza ad ogni tick con il valore corrente.

Non è banale: il damping entra nell'**equazione alle differenze discreta** dell'admittanza (dipende da `dt`), quindi cambiarlo online può violare la stabilità numerica del Metodo di Eulero implicito se non si aggiorna correttamente.

#### 2.2 — Circolarità logica
Il damping è modulato dalla forza del master → ma la forza del master include già il force-reflection dello slave → quindi il damping reagisce parzialmente al carico dello slave, non solo alla "presa" dell'operatore. Questo crea un loop implícito che può instabilizzarsi se `kf_force_ = 0.3` è troppo alto rispetto alla banda di modulazione.

#### 2.3 — Mancanza di isteresi
Senza isteresi sul passaggio `80→10 N·s/m`, il damping oscilla rapidamente attorno alla soglia di attivazione, producendo un **chatter** percepibile dall'operatore come vibrazione nell'handle. Il file analisi non menziona l'isteresi.

---

## 3. Deadband Asimmetrica Separata per Slave e Master

### Cosa propone l'analisi
Creare una deadband dedicata per il wrench dello slave, suggerendo `slave_force_deadband ≈ 10.0 N` (vs `2.0 N` attuale), per filtrare i piccoli disturbi da motion.

### Punti di forza
- **È la proposta più concreta e immediatamente implementabile**: basta aggiungere un parametro YAML distinto e applicarlo prima che `F_slave` entri nell'equazione del master.
- Con `slave_force_deadband = 10.0 N` si elimina il crosstalk dovuto alla compliance residua del load cell durante i moti liberi dello slave.
- **Non tocca la legge di controllo**, solo il filtraggio dell'ingresso.

### Criticità e limiti

#### 3.1 — Il valore `10.0 N` è arbitrario e potrebbe mascherare urti reali leggeri
Per un UR10e a bassa velocità, molti contatti significativi (prelievo di un oggetto di 500 g) generano forze nell'ordine di **5–15 N**. Con una deadband a `10.0 N` si rischia di **perdere il feedback di contatti reali** di media entità.  
Serve una campagna di misurazione su dati reali (bag files del wrench slave durante moti liberi) per scegliere il valore ottimale.

#### 3.2 — La deadband "soft" attuale ha isteresi; quella nuova deve averla
Il codice esistente usa `softDeadzoneNormWithHysteresis` con `enter/exit`. Il documento non specifica se la nuova `slave_force_deadband` dovrà avere la stessa isteresi: se viene reimplementata come deadband "hard" (sgn funzione), introduce **discontinuità nel segnale** che possono eccitare le oscillazioni che si vuole sopprimere.

#### 3.3 — Contraddizione con la cutoff del filtro LP
Entrambi i YAML usano `wrench_filter_cutoff_hz: 30.0`. Con 30 Hz di cutoff e 250 Hz di rate, il segnale slave è già filtrato con $\tau \approx 5$ ms. Alzare ulteriormente la deadband è corretto, ma **conviene prima abbassare la cutoff** (e.g. a `10–15 Hz`) e verificare se la deadband può restate più bassa, preservando meglio la sensibilità.

---

## 4. Gravity & Inertia Compensation (Dinamica Inversa sullo Slave)

### Cosa propone l'analisi
Implementare una pre-compensazione della massa e del baricentro del tool sull'end-effector dello slave per eliminare gli offset da accelerazione.

### Punti di forza
- **È il vero fix strutturale**: senza compensazione del tool, ogni accellerazione del braccio proietta la forza gravitazionale e inerziale del payload sul sensore, creando un segnale spuro che l'analisi correttamente identifica.
- I driver UR supportano la payload compensation via `set_payload` URScript già dall'UR CB-series: non richiede codice ROS aggiuntivo se configurato sul teach pendant.

### Criticità e limiti

#### 4.1 — La proposta è incompleta: non basta la massa
La compensazione corretta richiede **tre parametri per ogni tool**:
1. Massa `m` [kg]
2. Baricentro `{cx, cy, cz}` rispetto al TCP [m]
3. Tensore di inerzia `I` rispetto al TCP [kg·m²] (trascurabile per tool piccoli ad alte velocità moderate)

L'analisi menziona solo "massa, baricentro" — il tensore è spesso trascurabile a velocità < 1 rad/s ma non a 250 Hz di rate con giunti veloci.

#### 4.2 — La zeratura all'avvio (`ur_zero_ftsensor.py`) non sostituisce la compensazione dinamica
Il launch file esegue `ur_zero_ftsensor.py` all'avvio con `delay_s: 0.5` — questo azzera il bias statico in una configurazione fissa. Ma **durante il moto**, l'offset da inerzia varia con la postura e l'accelerazione del braccio. La zeratura statica non risolve il problema dinamico che l'analisi descrive correttamente.

#### 4.3 — L'implementazione ROS è non banale
In ROS non esiste un nodo standard pronto per la FT dynamic compensation su UR10. Le opzioni sono:
- Usare il payload setting URScript (lato robot, più affidabile)
- Scrivere un filtro ROS che computa $F_{grav} = R(\theta) \cdot [0, 0, m \cdot g]^T$ e lo sottrae online (richiede la cinematica diretta in tempo reale via TF al rate del sensore)

La seconda opzione ha una **latenza non trascurabile** se il TF lookup non è real-time-safe.

---

## Considerazioni Trasversali Mancanti nell'Analisi Originale

### A — Latenza di comunicazione master → slave
Il target pose viaggia su `/teleop/mur620b/target_pose` a **250 Hz**. Su setup ROS standard (non PREEMPT_RT) la latenza bidirezionale tipica sulla loopback è 2–8 ms, ma su rete Ethernet tra due macchine può arrivare a **15–30 ms**. Con questa latenza, il loop master→slave→master introduce una fase aggiuntiva che **abbassa la frequenza di stabilità del sistema accoppiato** — nessuna delle proposte dell'analisi la quantifica.

### B — `slave_base_inertia_frame` e la TF mancante
Dal launch file:
```xml
<param name="slave_base_frame" value="$(arg master_base_inertia_frame)"/>
<param name="slave_frame_id_override" value="$(arg slave_base_inertia_frame)"/>
```
Il master calcola la posa nel **proprio** frame (`master_base_inertia_frame`) ma l'etichetta come `slave_base_inertia_frame`. Funziona **solo se i bracci sono cinematicamente identici e montati in modo speculare a distanza nota**. Qualsiasi disallineamento geometrico tra i due sistemi di riferimento base introduce un errore di tracking sistematico che il PID dello slave (`kp: 0.5, ki: 0.8`) non può compensare completamente, e che amplifica l'oscillazione.

### C — PID con integrale (`ki: 0.8`) sul slave senza anti-windup esplicito
Il file `slave_real_mur620b_ur10_r.yaml` ha:
```yaml
pid:
  position:
    ki: 0.8
    output_limit: 0.05
```
L'`output_limit` satura l'**uscita totale** ma non l'integrale separatamente (anti-windup). Se lo slave è bloccato meccanicamente (contatto duro), l'integrale aumenta fino a `output_limit / ki` prima di saturare, poi al rilascio causa una **sovra-elongazione** e un colpo sul master via force-reflection. Questo è un possibile generatore delle "forti oscillazioni" citate.

---

## Riepilogo Critico per Priorità di Implementazione

| Proposta | Efficacia Attesa | Difficoltà Impl. | Rischi Residui | Priorità |
|---|---|---|---|---|
| **Deadband slave separata** (≥ 8–12 N) | Alta | Bassa | Perdita feedback contatti leggeri | ⭐⭐⭐ Immediata |
| **LP filter cutoff slave** ridotta a 10 Hz | Media | Minima | Ritardo risposta | ⭐⭐⭐ Immediata |
| **Deadman Switch hardware/software** | Alta | Media | Transizione brusca → serve rampa | ⭐⭐ Breve termine |
| **URI payload compensation (URScript)** | Molto alta | Bassa (lato robot) | Serve misurazione massa+CoM | ⭐⭐ Breve termine |
| **Damping variabile** | Media | Alta (modifica codice) | Chatter, circolarità, instabilità | ⭐ Lungo termine |
| **Anti-windup sull'integrale PID slave** | Media | Media | Nessuno rilevante | ⭐⭐ Breve termine |
| **Compensazione latenza rete** | Alta (sistemica) | Alta | Richiede RT kernel / DDS | ⭐ Lungo termine |
