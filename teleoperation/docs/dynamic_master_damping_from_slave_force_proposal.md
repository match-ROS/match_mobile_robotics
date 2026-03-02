# Proposta: smorzamento master dinamico in funzione della forza sullo slave

## Obiettivo

Introdurre un meccanismo che **aumenti dinamicamente lo smorzamento (damping) del master** al crescere della **forza misurata sulla cella di carico/FT dello slave**, con l’obiettivo di:

- aumentare stabilità e “sensazione di contatto” (meno oscillazioni/energia iniettata) quando lo slave va in contatto;
- mantenere bassa la dissipazione (quindi buona trasparenza) in free-space.

L’analisi e la proposta si basano **solo** su:

- `launch/master_slave_mur620b_real.launch`
- sorgenti C++ in `src/`
- YAML di configurazione in `config/`

## Stato attuale (catena segnali e dove si può intervenire)

### 1) Il master riceve già la forza dello slave

Nel `launch/master_slave_mur620b_real.launch` il nodo master `teleop_master_haptic_controller` viene cablato con:

- `slave_wrench_topic` impostato su `$(arg slave_wrench_topic_for_master)` che, di default, coincide con `$(arg slave_wrench_topic)` (topic wrench dello slave).

Quindi **il master ha già accesso alla misura di forza dello slave** (come `geometry_msgs/WrenchStamped`).

### 2) Nel master lo smorzamento è un parametro statico

In `src/teleop_master_haptic_controller_node.cpp` i parametri `damping_linear` / `damping_angular` (e opzionalmente `damping_*_xyz`) vengono letti una volta e poi usati nella dinamica:

- dinamica per asse: $ M \dot v + D v = (F_\text{hand} - F_\text{feedback}) $
- `D_lin`/`D_ang` sono vettori (3D) derivati da scalari o da `*_xyz`.

Non c’è un meccanismo attuale per rendere `D` dipendente dalla forza esterna, ma il punto d’iniezione è chiaro: **basta rendere `D_lin`/`D_ang` funzione della forza dello slave** prima di calcolare `rhs_lin`/`rhs_ang`.

## Proposta A (consigliata): dynamic damping dentro `teleop_master_haptic_controller`

### Idea

Riutilizzare direttamente il wrench dello slave già sottoscritto dal nodo master e calcolare un fattore $s(F)\in[0,1]$ crescente con la forza:

- $F$ = metrica di contatto basata su `|F_slave|` (norma della componente lineare);
- $s(F)$ = rampa liscia (smoothstep) tra due soglie;
- `D = D_base + s(F) * D_extra_max` (con clamp).

**Per robustezza**, la metrica consigliata è la **norma** della forza: è invariata a rotazioni e al segno, quindi meno sensibile a dettagli di frame.

### Definizione della metrica di forza

Usare nel master:

- `F_env = slave_filt_.f.norm()`

Note:

- `slave_filt_` nel master è già filtrato + deadband + clamp tramite `filterClampDeadbandWrenchNorm(...)`.
- Se si preferisce non “spegnere” il damping sotto deadband, si può calcolare `F_env` da un segnale filtrato *prima* della deadband (richiede piccola modifica per conservare anche “filtered-no-deadband” o usare `slave_raw` con filtro dedicato).

### Legge di scheduling (esempio)

Parametri:

- `F_start` (N): soglia da cui iniziare ad aumentare damping
- `F_stop` (N): soglia a cui il damping extra arriva al massimo
- `D_extra_lin_max` (N·s/m) e `D_extra_ang_max` (N·m·s/rad) (o versioni `*_xyz`)

Scheduling:

- $t = \mathrm{clamp}\Big(\frac{F_\text{env}-F_\text{start}}{F_\text{stop}-F_\text{start}}, 0, 1\Big)$
- $s = \mathrm{smoothstep}(t) = t^2(3-2t)$
- $D_\text{lin} = D_{\text{lin},base} + s \cdot D_{\text{lin},extra,max}$
- $D_\text{ang} = D_{\text{ang},base} + s \cdot D_{\text{ang},extra,max}$

Nota su “damping base” vs “extra”:

- il **damping di base** esiste già (`damping_linear`, `damping_angular`, o `damping_*_xyz`);
- qui basta definire solo l’**extra massimo** (`extra_damping_*`) che viene moltiplicato per \(s\) e sommato al base;
- non serve un ulteriore parametro “`damping_*_max`”: dato che \(s \in [0,1]\), il totale è automaticamente limitato a \(D_\text{base} + D_\text{extra,max}\) (oltre al fatto che il codice già sanitizza la non-negatività del damping).

### Rate limiting / filtro sul damping (fortemente raccomandato)

Per evitare “salti” percettivi e instabilità da variazioni rapide di $F$, introdurre un **low-pass direttamente su `D_extra`** (es. 5–15 Hz).

Questo è particolarmente importante perché:

- il wrench può avere rumore e picchi;
- il master usa già jerk-limiter su accelerazione, ma `D` che cambia istantaneamente può comunque creare discontinuità nella dinamica.

### Parametri da aggiungere (proposta YAML)

Nel file `config/master_real_mur620b_ur10_l.yaml` (namespace privato del nodo master) aggiungere, ad esempio:

```yaml
# --- Dynamic damping from slave force ---
dynamic_damping:
  enabled: true
  use_slave_wrench: true          # se false -> fallback a D_base
  force_metric: "norm"            # "norm" (consigliato), futuro: "parallel_to_v", "axis"
  force_start: 5.0                # N
  force_stop: 25.0                # N

  extra_damping_linear: 30.0      # N*s/m (aggiunta massima isotropa)
  extra_damping_angular: 1.5      # N*m*s/rad (aggiunta massima isotropa)

  d_extra_lowpass_cutoff_hz: 10.0   # low-pass su D_extra (0 o <0 per disabilitare)
```

Opzionale (più controllo): versioni per asse (Utente: non mi serve questa cosa opzionale)

```yaml
dynamic_damping:
  extra_damping_linear_xyz: [30.0, 30.0, 40.0]
  extra_damping_angular_xyz: [1.5, 1.5, 1.5]
  d_extra_lowpass_cutoff_hz: 10.0
```

### Modifiche al C++ (punti concreti)

In `teleop_master_haptic_controller_node.cpp`:

1) **Parse parametri** `dynamic_damping/*` nel costruttore.
2) Nel `tick(...)`, dopo aver aggiornato `slave_filt_`:
   - calcolare `F_env` (se `has_slave` e non stale; altrimenti `F_env=0`);
   - calcolare `s(F_env)` con smoothstep;
   - calcolare `D_extra = s * D_extra_max` e applicare **low-pass su `D_extra`** con `d_extra_lowpass_cutoff_hz`;
   - costruire `D_lin`/`D_ang` finali:
     - `D = D_base + D_extra_filtered`
3) Usare `D_lin`/`D_ang` finali al posto di quelli statici nel calcolo:
   - `rhs_lin = (F_hand - F_feedback) - D_lin.cwiseProduct(v_lin_cmd_)`
   - `rhs_ang = (Tau_hand - Tau_feedback) - D_ang.cwiseProduct(v_ang_cmd_)`
4) (Debug) pubblicare su un topic debug o estendere `debug/admittance_stats` con:
   - `F_env`, `s`, `D_lin_eff`, `D_ang_eff`.

### Impatto sul launch

Nessuna modifica strettamente necessaria al `launch` perché:

- il master già riceve `slave_wrench_topic`;
- i parametri possono vivere nel YAML master.

Opzionale: aggiungere argomenti per attivare/disattivare rapidamente in launch, sovrascrivendo i parametri YAML con `<param ...>`.



## Punti aperti (da decidere prima di implementare)

- **Quale forza usare come metrica**:
  - `|F_slave|` (norma) è semplice e robusta;
  - proiezione lungo direzione di moto/asse specifico può dare un comportamento più “intelligente”, ma richiede definizioni di frame coerenti e può essere fragile.
  **Risposta:** Farlo sulla norma e basta, non mi serve lungo direzioni specifiche.
- **Uso torque**:
  - per ora la richiesta parla di forza; decidere se includere anche `|Tau_slave|` per smorzamento angolare.
  **Risposta**: Fare la stessa cosa anche per la coppia
- **Soglie `F_start/F_stop`**:
  - devono stare sopra la deadband e sotto eventuali clamp.
  - attenzione: nel master la deadband/clamp della forza di feedback possono mascherare piccoli contatti.
  **Risposta:** Inserirò io i valori, non ti preoccupare
- **Cosa fare quando il wrench dello slave è stale o assente**:
  - oggi, se `slave_wrench_topic` è attivo e diventa stale, il master entra nella logica “stale wrench” e pubblica zero twist (fail-safe).
  - se vuoi che la teleop continui anche senza wrench slave, va introdotto un parametro tipo `require_slave_wrench=false` o separare il canale “damping scheduler” dalla logica di stale.
  **Risposta:** Se il topic diventa stale è come se avrò una forza nulla
- **Velocità di aggiornamento e filtraggio**:
  - scegliere il cutoff del low-pass su `D_extra` per evitare oscillazioni introdotte dallo scheduling stesso.
  **Risposta:** Lo sceglierò io
- **Interazione con altri feedback**:
  - virtual spring coupling (`spring_*`) e force reflection (`force_reflection_scale`) già iniettano un feedback; aumentare `D` cambia il bilancio dissipativo e la “trasparenza”.
  **Risposta:** Farò dei test

## Criticità / rischi principali (tecniche e di integrazione)

- **Trasparenza vs stabilità**: troppo damping rende il master “pesante” e riduce la fedeltà della teleoperazione.
- **Passività / energia**: lo scheduling deve essere monotono e smussato; variazioni rapide del damping possono introdurre sensazioni non lineari.
- **Saturazioni**:
  - il master ha saturazioni su velocità e jerk limiter sull’accelerazione; con damping alto, può essere più difficile seguire l’operatore, e la dinamica può “incollarsi” vicino ai limiti.
- **Deadband/clamp sul wrench**:
  - se `F_env` è calcolata da un segnale post-deadband, sotto soglia il damping non cambia affatto; può essere desiderato o no.
- **Affidabilità stream slave**:
  - con `slave_wrench_topic` attivo, il master considera il wrench slave “necessario” (stale => stop). Questo è un comportamento già presente che diventa ancora più centrale se lo smorzamento dipende dal wrench.
- **Tare/zero e deriva sensore**:
  - il `launch` include `ur_zero_ftsensor.py` per zeroing a startup; deriva o bias residui possono far crescere damping anche senza vero contatto (mitigabile con deadband e soglie).

## Piano di validazione (test)

- **Test offline/rosbag**:
  - loggare `slave_wrench`, `F_env`, `s`, `D_eff`, `v_cmd` e verificare monotonia e assenza di chattering.
- **Test in free-space**:
  - verificare che `D_eff ≈ D_base` e che la manovrabilità non peggiori.
- **Test contatto graduale**:
  - aumentare progressivamente contatto e verificare che il master smorzi senza oscillazioni o “scatti”.
- **Test fault**:
  - interrompere temporaneamente `slave_wrench_topic` e verificare comportamento desiderato (stop vs fallback).

## Checklist modifiche (riassunto)

- **C++**: `src/teleop_master_haptic_controller_node.cpp`
  - aggiunta parametri `dynamic_damping/*`
  - calcolo `F_env` da wrench slave filtrato
  - scheduling + smoothing/limiting
  - applicazione a `D_lin`/`D_ang`
  - (opzionale) debug topic
- **YAML**: `config/master_real_mur620b_ur10_l.yaml`
  - aggiunta sezione `dynamic_damping`
- **Launch**: opzionale
  - arg per enable/disable e override soglie

