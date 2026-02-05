# Analisi problema: Virtual Target Leash che blocca anche le componenti tangenziali

## Contesto (riassunto)
Nel `cartesian_velocity_controller` il **LocalPlanner** genera una velocità cartesiana desiderata \(`combined_linear`, `combined_angular`\) che viene:

1. filtrata dal **VelocityFilter** (Level C),
2. usata come **feed-forward** nel **PID** (Level D),
3. e, in parallelo, integrata per aggiornare la posa del **target virtuale** (`target_raw_`).

Per evitare l’“overshoot” del target virtuale (il punto che “scappa” avanti al robot quando il robot è in ritardo), è stata introdotta la logica **Virtual Target Leash**.

---

## Sintomo riportato
Quando il punto virtuale raggiunge la **distanza massima consentita** dal robot:

- il punto virtuale viene rallentato,
- viene ridotto il **comando di feed-forward**,
- e se si applica una velocità (repulsiva) **perpendicolare** alla congiungente robot↔punto, il robot **non si muove** neanche in quella direzione (effetto “bloccato”).

Questo è particolarmente evidente in scenari “sliding”: vorresti che il robot si muovesse **tangenzialmente** lungo la superficie di distanza massima, senza aumentare ulteriormente la distanza robot↔punto.

---

## Dove succede nel codice (root cause “meccanica”)

### 1) Il leash scala *tutta* la velocità del punto virtuale (anche la tangenziale)
In `LocalPlanner::compute()` viene calcolato `scaling_factor` in base alla distanza tra robot e target virtuale:

- per `dist > leash_start_distance` → scala da 1 a 0
- per `dist ≈ leash_stop_distance` → `scaling_factor → 0`
- per `dist > leash_reset_threshold` → reset “hard”

Poi, se lo scaling è attivo, si fa:

```cpp
// Soft leash scaling
v_combined_linear *= scaling_factor;
v_combined_angular *= scaling_factor;
```

Questo implica che **quando il target è “al limite”** (`scaling_factor = 0`) vengono annullate *tutte* le componenti di velocità del target virtuale, incluse quelle che non aumenterebbero la distanza (es. componenti tangenziali).

File: `cartesian_velocity_controller/src/components/local_planner.cpp` (step “Virtual Target Leash”).

### 2) La stessa velocità scalata diventa anche il feed-forward del PID
Nel loop principale:

- `LocalPlannerOutput::combined_*` → `desired_twist`
- `desired_twist` → `VelocityFilter::filter()`
- `filtered_twist` → **feedforward** del `PIDController`

Quindi, quando il leash scala a zero la velocità, **si azzera anche il feed-forward**, e il robot perde la “spinta” nella direzione repulsiva/tangenziale che ti aspetti.

File: `cartesian_velocity_controller/src/cartesian_velocity_controller.cpp` (feed-forward dal filtro).

---

## Perché “blocca” proprio il moto perpendicolare (spiegazione geometrica)
Indichiamo con:

- \(p_r\): posizione robot (TCP)
- \(p_v\): posizione target virtuale (`target_raw`)
- \(r = p_v - p_r\)
- \(n = r / \|r\|\) (direzione radiale robot→target)
- \(v\): velocità desiderata del target virtuale (`v_combined_linear`)

La componente che **aumenta** la distanza è la parte “radiale uscente”:
\[
v_\parallel = (v \cdot n)\,n
\]
La componente “tangenziale” è:
\[
v_\perp = v - v_\parallel
\]

Se il leash scala **tutto** \(v\) con un fattore \(s \in [0,1]\), allora:
\[
v' = s\,v = s\,v_\parallel + s\,v_\perp
\]
Quando \(s \to 0\) (al limite), anche \(v_\perp \to 0\). Quindi **non esiste più** alcun comando che cambi “angolo” di \(r\): il target resta fermo e il robot non riceve (via feed-forward) il contributo tangenziale.

---

## Soluzioni possibili (con pro/contro)

### Soluzione A — Leash anisotropo: limita solo la componente “radiale uscente”
Idea: lasciare inalterata la componente tangenziale \(v_\perp\) e limitare (o annullare) solo la parte che farebbe aumentare \(\|r\|\).

Schema:
1. calcola \(n\)
2. decomponi \(v\) in \(v_\parallel\) e \(v_\perp\)
3. se \((v \cdot n) > 0\) (uscente), scala solo quella componente con \(s(d)\)
4. ricompone:
\[
v' = v_\perp + s(d)\,v_\parallel \quad \text{(solo se uscente)}
\]
e lascia la componente entrante \((v \cdot n) < 0\) non scalata (o scalata meno).

**Pro**
- Risolve esattamente il caso che descrivi: al “limite” il target può ancora “scivolare” tangenzialmente.
- Mantiene la protezione dall’overshoot (non cresce ulteriormente la distanza).
- È un cambiamento localizzato (solo in `LocalPlanner`).

**Contro / criticità**
- Se il target “orbita” attorno al robot senza avanzare verso il waypoint, potresti avere traiettorie meno intuitive (serve valutare).
- Senza stimare la velocità del robot, il vincolo è applicato sul target “assoluto”, non sul target “relativo” (vedi Soluzione D).

**Varianti**
- Applicare la stessa idea anche a `v_combined_angular` (tipicamente separando leash traslazionale e rotazionale).
- Aggiungere una piccola correzione entrante quando \(\|r\|\) è alta, per riportare gradualmente il target sotto `leash_stop_distance`.

---

### Soluzione B — Vincolo sullo stato: integra normalmente, poi “clampa” la posizione del target
Idea: non scalare la velocità, ma dopo l’integrazione imporre:
\[
\|p_v - p_r\| \le R
\]

Operativamente:
- si integra `target_raw_` con la velocità originale \(v\),
- se il target finisce fuori dalla sfera di raggio \(R = leash_stop_distance\), si proietta:
  \[
  p_v \leftarrow p_r + R \, \frac{(p_v - p_r)}{\|p_v - p_r\|}
  \]

**Pro**
- La componente tangenziale non viene “uccisa”: il target può muoversi lungo il bordo.
- È concettualmente “anti-windup sullo stato” (più vicino alla natura del problema: `target_raw_` è un integratore).

**Contro / criticità**
- La proiezione può introdurre discontinuità nello stato (salti piccoli ma non nulli), che possono:
  - creare jerk (soprattutto se la proiezione agisce spesso),
  - generare incoerenza tra velocità usata per integrare e posizione finale “clampata”.
- Se `filtered_twist` (feed-forward) resta grande ma lo stato viene clampato, potresti “spingere” il robot con feed-forward senza che il setpoint si muova coerentemente (dipende da come scegli di combinare i segnali; vedi Soluzione C).

---

### Soluzione C — Decoupling: il leash agisce sul target, non sul feed-forward (o viceversa)
Il problema che segnali include esplicitamente: “il leash diminuisce il feed-forward e rallenta il robot”.

Qui l’idea è separare due concetti:
- **(i) controllo dell’overshoot**: riguarda l’aggiornamento/inseguimento del target virtuale (setpoint),
- **(ii) feed-forward**: riguarda la “traiettoria di velocità” desiderata.

Varianti:

**C1 — Leash solo sull’integrazione del target (`target_raw_`), non sull’output `combined_*`**
- Si mantiene `local_output.combined_*` *non scalato* (quindi il feed-forward continua).
- Si applica la logica leash solo alla velocità usata da `integrateTarget()`.

**Pro**
- Attacca direttamente la tua criticità: il feed-forward non collassa a zero solo perché il target è lontano.

**Contro**
- Potenziale incoerenza: feed-forward “spinge” in una direzione mentre il target (setpoint) è “tenuto” dal leash.
- Da valutare stabilità/tuning (dipende da `kff`, `kp`, `output_limit`).

**C2 — Leash solo sull’output (feed-forward), ma non sul target**
- È quasi l’opposto di quello che vuoi, quindi la cito solo come completezza: può essere utile se si vuole limitare aggressività, ma non risolve il blocco tangenziale.

**Nota**: se scegli C1, spesso conviene combinare con A o B (altrimenti il target può comunque scappare come dinamica integrativa).

---

### Soluzione D — Leash “relativo”: vincolo sulla distanza in termini di velocità relativa target↔robot
Oggi il leash valuta solo \( \|p_v - p_r\| \). Ma la distanza evolve come:
\[
\dot{r} = \dot{p}_v - \dot{p}_r
\]
Quindi un vincolo più corretto è imporre qualcosa come:
\[
n \cdot (\dot{p}_v - \dot{p}_r) \le \text{limite}(d)
\]

Per farlo servirebbe una stima di \(\dot{p}_r\) (velocità del TCP), per esempio:
- differenza finita tra `current_tcp_pose.translation()` e la posa precedente (tenendo in memoria `prev_pose`), oppure
- ricavandola dalle velocità giunti (se disponibili) e Jacobiano.

**Pro**
- Il leash diventa “fisicamente corretto”: se il robot sta recuperando distanza, il target può continuare ad avanzare senza essere strangolato.

**Contro / criticità**
- Introduce un nuovo stato (stima velocità) e sensibilità al rumore/filtri.
- Va progettata bene la filtratura per non introdurre instabilità.

---

### Soluzione E — Formulazione come problema vincolato (proiezione/QP)
Si può scegliere \(v'\) come la velocità “più vicina possibile” a \(v\) ma che rispetta vincoli:

Esempio (lineare, facile):
- vincolo “non aumentare distanza” quando sei al limite:
  \[
  n \cdot v' \le 0 \quad \text{se } \|r\| \ge R
  \]
- e limite di norma: \(\|v'\| \le v_{max}\)

Questo può essere implementato:
- come proiezione semplice (equivalente alla Soluzione A in molti casi),
- oppure come mini-QP (più generale: puoi aggiungere anche vincoli su assi, su componenti, ecc.).

**Pro**
- Robustezza: facile incorporare più vincoli (es. mantenere distanza entro banda, priorità, ecc.).

**Contro**
- Più complessità; va evitato di introdurre dipendenze pesanti/solver se non serve.

---

### Soluzione F — “Workaround” di configurazione per validare la diagnosi (non è una fix)
Per confermare sperimentalmente che il colpevole è il leash:
- disabilitare temporaneamente `local_planner/leash_enabled` (o aumentare `leash_stop_distance`) e verificare che la componente tangenziale torni a funzionare.

Questo non risolve l’overshoot, ma aiuta a isolare il problema prima di implementare.

---

## Criticità e problemi aperti (da discutere insieme)

### 1) Qual è il comportamento desiderato al “limite”?
Domande pratiche:
- Vuoi **vietare solo l’uscita** (radiale positiva) e consentire sempre la tangenziale? (tipico “sliding along boundary”)
- Vuoi anche consentire un po’ di uscita ma molto attenuata?
- Vuoi che la tangenziale venga attenuata comunque (per evitare orbite troppo veloci) ma non annullata?

### 2) Interazione con l’inseguimento del waypoint
Se al limite il target “scivola”, potresti:
- perdere momentaneamente avanzamento verso waypoint (se il waypoint richiede una componente radiale),
- generare traiettorie “a spirale”/orbite.

Serve definire una priorità: sicurezza/repulsione vs avanzamento verso goal.

### 3) Angolare: va davvero scalata insieme al lineare?
Attualmente lo scaling applica anche:
- `v_combined_angular *= scaling_factor`

Questo può avere effetti collaterali:
- se sei al limite di distanza traslazionale, potresti comunque voler ruotare per orientare tool o evitare ostacoli.

Possibile decisione: leash solo su traslazione, o leash angolare separato.

### 4) Reset hard: cosa succede a filtro e PID quando `perform_reset == true`?
In reset hard il `LocalPlanner`:
- riporta `target_raw_` sulla posa corrente,
- mette le velocità a zero in quel ciclo.

Punti aperti:
- conviene anche resettare `VelocityFilter` e/o integrali del PID per evitare transitori?
- conviene introdurre isteresi sul reset (o limitarne la frequenza)?

### 5) Altri limitatori “globali” che possono mascherare il comportamento
Anche se non sembrano la causa primaria del tuo sintomo, possono amplificarlo:
- `VelocityFilter` limita la norma della velocità filtrata (\(|v| \le max\_velocity\)).
- `JointSafetyLimiter` applica **scaling uniforme** alle velocità giunti (se un giunto satura, scala tutto).

Quindi, quando testiamo la fix, va osservato anche:
- `joint_safety_limiter` scaling factor (per capire se la limitazione è in joint space e non nel leash).

---

## Esperimenti/diagnostica consigliata per scegliere la soluzione

### Esperimento 1 — Isolare il leash
- set `local_planner/leash_enabled: false`
- ripeti il test “repulsiva tangenziale al limite”
- se torna a muoversi → conferma forte della root cause.

### Esperimento 2 — Guardare i segnali già disponibili nel debug
Nel `debug_data` del controller sono già presenti segnali utili:
- `virtual_target_scaling_factor`
- `distance_target_raw_to_current`
- `pid_feedforward_linear` / `pid_output_linear`

Ci interessa verificare: quando sei “bloccato”, `virtual_target_scaling_factor` sta effettivamente a ~0?

### Esperimento 3 — Test “tangenziale pura”
Costruire un caso:
- `dist(target, robot) ≈ leash_stop_distance`
- impostare una velocità desiderata \(v\) tale che \(v \cdot n = 0\) (tangenziale)
- aspettativa desiderata: il target si muove e il robot lo segue tangenzialmente.

Questo test è perfetto per validare Soluzione A/B/E.

---

## Raccomandazione (opzione “più semplice e mirata”)
Se l’obiettivo è: “al limite posso muovermi tangenzialmente ma non aumentare distanza”, la soluzione più diretta è:

**Soluzione A (leash anisotropo: scala solo la componente radiale uscente)**  
eventualmente completata con:
- leash solo traslazionale (non angolare), oppure leash angolare separato,
- una leggera isteresi per evitare chattering.

---

## Domande aperte per decidere insieme (checklist)
1. Il “limite” deve essere una **sfera** attorno al TCP o attorno a un altro punto (base, flange, ecc.)?
2. La tangenziale deve essere:
   - sempre permessa,
   - permessa ma limitata,
   - permessa solo se contribuisce ad allontanarsi da un ostacolo?
3. Il feed-forward deve rimanere attivo anche quando il target è leashed (decoupling C1) oppure vuoi che anche il FF rispetti il vincolo (A/B/E)?
4. Come gestiamo l’orientamento quando la traslazione è al limite?

