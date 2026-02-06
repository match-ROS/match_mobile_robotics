# Analisi problema: `ur_l_rpy` (rotazione base braccio) → tracking TCP “invertito/anomalo”

## 1) Sintomi osservati

- Se nel modello URDF/Xacro del MUR620 imposti una rotazione della base del braccio sinistro diversa da 0 (parametro `ur_l_rpy`), il controllore cartesiano fatica a portare il TCP sul target.
- All’aumentare dell’angolo (fino a ~180°) il problema peggiora; intorno a 180° il robot **va nella direzione opposta** e il moto diventa “anomalo”.

Questo pattern (errore che cresce monotonicamente con una rotazione fino a diventare quasi l’opposto) è *molto tipico* di un **mismatch di frame**: una grandezza vettoriale (errore/velocità) viene calcolata in un frame ma poi “interpretata”/iniettata in un Jacobiano espresso in un altro frame ruotato.

---

## 2) Da dove arriva `ur_l_rpy` nel modello

Nel file:

- `mur_description/urdf/mur_620.gazebo.xacro`

la rotazione `ur_l_rpy` viene applicata nel giunto fisso:

- `plattform_ur_calibration_joint_l` (parent `UR10_l/base_ideal`, child `UR10_l/base_link`)

cioè impone una **rotazione fissa** tra il frame “di montaggio ideale” e il frame base reale del braccio.

Con `ur_l_rpy = 0` quei due frame coincidono. Con `ur_l_rpy ≠ 0` introduci una rotazione $R_z(\theta)$ (se modifichi lo yaw) tra i frame.

---

## 3) Come funziona oggi il `cartesian_velocity_controller` (punti rilevanti)

### 3.1 Target pose (frame)

Nel controller C++ (`cartesian_velocity_controller/src/cartesian_velocity_controller.cpp`):

- Il parametro `global_frame` viene letto da YAML/param server.
- `setTargetPose(const geometry_msgs::PoseStamped&)`:
  - se `frame_id != global_frame`, tenta una trasformazione TF2 verso `global_frame`
  - poi **salva internamente** la posa trasformata (senza più il metadato `frame_id`, perché diventa `Eigen::Isometry3d`)

Quindi: *la pipeline pianificatore→PID usa target in `global_frame`*.

### 3.2 Posa corrente del TCP + Jacobiano (frame)

La posa corrente del TCP e il Jacobiano vengono calcolati tramite MoveIt:

- `RobotStateManager::computeTcpPose(...)` usa `moveit::core::RobotState::getGlobalLinkTransform(...)`
- `RobotStateManager::getJacobian(...)` usa `moveit::core::RobotState::getJacobian(...)`

Entrambe queste API lavorano nel **frame “global/model” di MoveIt**, che in pratica è il **root link del robot model** (URDF root link / “model root frame”).

Nel codice del controller esiste esplicitamente un servizio di debug:

- `~get_frame_info` (`cartesian_velocity_controller/srv/GetFrameInfo.srv`)

che serve proprio a confrontare:

- `model_root_frame` (root del RobotModel di MoveIt)
- `config_global_frame` (il `global_frame` configurato)
- TF tra i due (se disponibile)

### 3.3 Assunzione implicita critica

Nel loop di controllo:

- l’errore $e$ (posizione/orientazione) viene calcolato come differenza tra **target** (in `global_frame`) e **tcp corrente** (in *frame modello MoveIt*)
- la twist comandata `command_twist` viene poi convertita in joint velocities via `qdot = J^+ * command_twist`, dove `J` è il Jacobiano MoveIt (frame modello MoveIt)

Questo *funziona solo se*:

> `global_frame` **coincide** (o è equivalente) al frame in cui MoveIt esprime FK e Jacobiano.

Se invece tra questi due frame c’è una rotazione $R$, allora stai facendo (di fatto):

$$
qdot = J_{model}^{+}\, v_{global}
$$

ma il Jacobiano “si aspetta” un twist nel frame `model`, cioè:

$$
v_{model} = \begin{bmatrix}R^\top & 0\\ 0 & R^\top\end{bmatrix} v_{global}
$$

Se non applichi quella rotazione, la direzione del moto viene ruotata di $\theta$. Quando $\theta \to \pi$ (180°), $R \approx -I$ e quindi il comando **diventa quasi l’opposto**: esattamente il comportamento che descrivi.

---

## 4) Perché il problema “scala” con `ur_l_rpy` (collegamento con la tua osservazione)

La rotazione `ur_l_rpy` introduce una rotazione fissa nella catena TF tra frame legati al montaggio del braccio.

Se il controller sta mescolando:

- target in un frame legato alla base mobile (`mur620/base_link`) **oppure** a `UR10_l/base_ideal`
- Jacobiano/FK in un frame legato alla base del braccio (`UR10_l/base_link`) **oppure** al model root MoveIt

allora la rotazione introdotta dal giunto fisso diventa proprio la $R$ che “ruota” (o inverte) l’errore.

La cosa importante è che **non serve** che i nomi dei frame siano “sbagliati”: basta che *numericamente* target e Jacobiano siano espressi in basi diverse e tu li tratti come la stessa base.

---

## 5) Come verificare in modo deterministico (senza ipotesi)

### 5.1 Verifica frame MoveIt vs frame controller (C++ service)

Chiama il servizio `get_frame_info` dell’istanza del controller che stai usando.

Esempi (dual-arm):

- `/mur620/cartesian_velocity_controller_l/get_frame_info`
- `/mur620/cartesian_velocity_controller_r/get_frame_info`

Se:

- `frames_match = false`
- e `tf_model_to_global.rotation` mostra una rotazione “coerente” con `ur_l_rpy` (tipicamente yaw ≈ `ur_l_rpy[2]`)

allora hai trovato la causa primaria: **mismatch tra frame modello (MoveIt) e global_frame del controller**.

### 5.2 Test “round-trip” (target = posa corrente)

Test robusto per scovare mismatch frame/TF:

1. Leggi la posa corrente del TCP dal TF (in un frame che sai essere corretto).
2. Pubblica esattamente quella posa come `target_pose` con lo **stesso `frame_id`**.

Comportamento atteso:

- il robot deve muoversi ~0 (solo micro-correzioni).

Se invece si muove “forte” o nella direzione opposta, significa che:

- o il `frame_id` del messaggio non corrisponde al frame in cui sono scritti i numeri
- o dentro al controller FK/Jacobiano e target non sono nello stesso frame (mismatch interno).

### 5.3 Test “position-only” (separare l’errore orientamento)

Attorno a 180° l’errore orientamento in axis-angle può diventare *numericamente* instabile (asse ambiguo quando l’angolo è $\pi$).

Per capire se il problema è *solo* orientamento o *anche* Jacobiano/frame:

- imposta temporaneamente i guadagni orientamento a zero (o molto piccoli) e riprova.

Se il tracking posizione torna ragionevole → la parte di orientamento stava dominando e “inquinando” il moto.
Se resta invertito/opposto → il problema è quasi certamente frame/Jacobiano.

---

## 6) Cause più probabili (in ordine di probabilità)

### Causa A (più probabile): mismatch tra frame di target (global_frame) e frame di FK/Jacobiano (MoveIt model)

**Segnale tipico**: errore che ruota con `ur_l_rpy`, fino a diventare quasi opposto a 180°.

**Perché può capitare qui**:

- Il controller assume implicitamente che `global_frame` e `model_root_frame` coincidano.
- Ma sul MUR620 spesso si lavora con frame “prefissati” (`mur620/...`) e frame “non prefissati” (`base_link`, `UR10_l/base_link`, …), e/o con root MoveIt che non è `mur620/base_link`.

### Causa B: target numericamente espresso in `UR10_l/base_ideal` ma pubblicato con `frame_id` diverso (es. `.../base_link`)

Con `ur_l_rpy=0` non lo vedi (perché `base_ideal`≈`base_link`), con `ur_l_rpy→π` diventa un “comando al contrario”.

Questo può succedere in tool esterni o in script/pose salvate vecchie.

### Causa C: errore orientamento in axis-angle vicino a $\pi$ + controllo 6D rigido

Anche se il calcolo usa quaternioni e “shortest path”, a $\theta=\pi$ l’asse non è univoco e piccole variazioni possono flipparlo.
Se l’orientamento richiesto dal target non è compatibile (o è “lontanissimo” perché non compensi il mounting), l’azione sull’orientamento può saturare e portare il robot a muoversi in modo non intuitivo anche sulla posizione.

---

## 7) Fix / soluzioni (pratiche)

### Soluzione 1 (config, spesso sufficiente): allinea `global_frame` al frame del modello MoveIt

Obiettivo: fare in modo che:

- il target venga interpretato nel **medesimo frame** in cui MoveIt restituisce FK/Jacobiano.

Procedura:

1. Usa `get_frame_info` per scoprire `model_root_frame`.
2. Imposta `global_frame` = `model_root_frame` nel YAML del controller (o via launch param).
3. Assicurati che gli strumenti che pubblicano target usino quel frame.

Pro:
- zero patch C++, “leva” il problema alla radice.

Contro:
- se preferisci mantenere `global_frame` “umano” (`mur620/base_link`), ti serve una politica di alias/TF coerente o trasformazioni affidabili lato input.

### Soluzione 2 (robusta, consigliata lato software): rendere il controller *frame-consistent* internamente

Obiettivo: permettere `global_frame != model_root_frame` senza bug.

Idea:

- calcola $T^{global}_{model}$ via TF2 (`lookupTransform(global_frame, model_root_frame, ...)`)
- usa $T^{global}_{model}$ per trasformare:
  - la posa FK del TCP dal frame modello → `global_frame`
  - il Jacobiano dal frame modello → `global_frame` (rotazione su parte lineare e angolare)
  - (e simmetricamente) i target per IK/reachability check dal `global_frame` → frame modello

Nota tecnica:

- per convertire un twist/vettore 6D tra frame con stessa origine del twist al TCP, basta la rotazione:
  $$
  v_{global} =
  \begin{bmatrix}
  R & 0 \\
  0 & R
  \end{bmatrix}
  v_{model}
  $$
  e quindi $J_{global} = \mathrm{Ad}(R)\, J_{model}$.

Impatto positivo collaterale:

- marker/debug/feedback (`header.frame_id = global_frame`) diventano coerenti anche quando i frame non coincidono.

### Soluzione 3 (mitigazione): “position-first” o disabilitare temporaneamente l’orientamento

Se il tuo caso d’uso è principalmente position tracking:

- riduci `pid_controller/orientation/*` (kp, ki, kff) oppure mettili a 0 per test.

Questo non risolve un mismatch frame, ma può evitare comportamenti catastrofici quando l’orientamento richiesto è lontano o ambiguo.

---

## 8) Checklist rapida per isolare la causa in 10 minuti

- [ ] Chiama `.../get_frame_info` e verifica `frames_match`.
- [ ] Se `frames_match=false`, controlla che la rotazione TF model→global sia ~ `ur_l_rpy` (specialmente yaw).
- [ ] Esegui “round-trip”: target = posa corrente con frame coerente.
- [ ] Ripeti con guadagni orientamento a 0 per capire se l’anomalia è dominata dall’orientamento.
- [ ] Verifica che i target che invii abbiano `header.frame_id` corretto rispetto ai numeri.

---

## 9) Conclusione (root-cause più probabile)

Dato il fatto che l’errore cresce con l’angolo fino a invertirsi a ~180°, la spiegazione più consistente è:

> Il controllore sta combinando **errore/target in un frame** con un **Jacobiano/FK in un frame ruotato**, senza trasformare le grandezze tra i frame.  
> La rotazione `ur_l_rpy` è esattamente la rotazione che “entra” nel mismatch, quindi quando $\theta\to\pi$ il comando diventa (quasi) l’opposto.

Il passo successivo più efficace è usare `get_frame_info` per confermare quale coppia di frame non è allineata, e poi scegliere tra:

- soluzione “config” (allineare i frame)
- soluzione “robusta” (patch per trasformare Jacobiano/FK/IK tra model e global)

