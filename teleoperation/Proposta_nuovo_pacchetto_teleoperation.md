# Proposta concreta per il nuovo pacchetto di teleoperazione

## Obiettivo

Creare un nuovo pacchetto ROS dedicato alla teleoperazione semplificata, separato dal pacchetto `teleoperation` attuale, con questi obiettivi:

- ridurre al minimo la complessita del codice
- mantenere solo la logica realmente usata
- eliminare le funzionalita non piu desiderate
- avere un'architettura chiara e facile da testare

La proposta qui sotto recepisce le decisioni gia prese durante la discussione.

## Decisioni gia fissate

### Master

- mantiene una dinamica di tipo massa-smorzatore
- il feedback dal lato slave non arriva piu dalla F/T dello slave
- il feedback aptico verso il master deriva dalla differenza di posa tra master e slave tramite molla virtuale
- deve poter abilitare o disabilitare separatamente traslazione e rotazione

### Slave

- non usa piu il feedforward della velocita del master
- si muove solo tramite errore di posa
- deve seguire una legge coerente con le equazioni delle specifiche
- non servono force limiter
- non serve hard guard
- deve poter abilitare o disabilitare separatamente traslazione e rotazione

### Infrastruttura da mantenere

- override del `frame_id` quando viene pubblicata la posa target
- override del `frame_id` quando lo slave pubblica la propria posa al master
- wrench ruotato tra frame usando solo la rotazione
- filtro esponenziale con parametro espresso in `Hz`
- deadband come implementazione attuale
- limiti su velocita, accelerazione e jerk
- sanitizzazione di `dt` con substep

### Cose da rimuovere dal nuovo pacchetto

- riflessione diretta della forza dello slave nel master
- massa dinamica e smorzamento dinamico
- coupling
- feedforward twist master -> slave
- PID legacy nello slave
- launch di taratura e script non necessari al funzionamento base
- naming ereditato dal pacchetto vecchio

## Nome proposto del nuovo pacchetto

Propongo come nome iniziale:

`teleoperation_simplified`

E semplice, descrittivo e coerente con il fatto che il pacchetto nuovo nasce come versione ridotta del sistema attuale.

Se vuoi un nome ancora piu corto, una buona alternativa e:

`teleop_simple`

## Architettura proposta

Il nuovo pacchetto contiene solo due nodi di controllo e poche utility condivise.

### Nodo master

Nome proposto:

`teleop_master_admittance`

Responsabilita:

- legge la F/T del master
- legge la posa attuale dello slave pubblicata dal nodo slave
- calcola il comando di velocita del master
- pubblica la posa target per lo slave
- pubblica opzionalmente la F/T filtrata del master per debug

### Nodo slave

Nome proposto:

`teleop_slave_pose_admittance`

Responsabilita:

- legge la posa target pubblicata dal master
- legge la propria F/T
- calcola la velocita del robot slave usando solo errore di posa e forza esterna
- pubblica la propria posa attuale per il nodo master
- pubblica opzionalmente la F/T filtrata dello slave per debug

## Flusso dati per una singola coppia master/slave

Per ogni braccio:

1. il nodo `teleop_master_admittance` legge la F/T del master
2. il nodo `teleop_master_admittance` legge la posa attuale dello slave
3. il nodo `teleop_master_admittance` genera il comando twist del master
4. il nodo `teleop_master_admittance` pubblica la posa target per lo slave
5. il nodo `teleop_slave_pose_admittance` legge la posa target
6. il nodo `teleop_slave_pose_admittance` legge la F/T dello slave
7. il nodo `teleop_slave_pose_admittance` genera il comando twist dello slave
8. il nodo `teleop_slave_pose_admittance` pubblica la propria posa attuale verso il master

Nel nuovo schema non esiste piu il topic di feedforward twist verso lo slave.

## Leggi di controllo proposte

## Master

Il master resta concettualmente vicino alla versione attuale, ma senza F/T dello slave nel controllo.

### Parte traslazionale

Si propone:

$$
F_{fb} = K_m \cdot (x_s - x_m)
$$

$$
\ddot{x}_m = \frac{1}{M_m}\left(F_{umano} + F_{fb} - D_m \dot{x}_m\right)
$$

integrazione numerica:

$$
\dot{x}_m(t+\Delta t) = \dot{x}_m(t) + \ddot{x}_m \Delta t
$$

Il comando inviato al controller di velocita del master e quindi la velocita integrata saturata e limitata.

### Parte rotazionale

Stessa idea usando:

- errore orientazionale in asse-angolo
- momento misurato sul master
- inerzia e smorzamento virtuali angolari
- molla virtuale angolare sulla differenza di orientazione

### Nota implementativa

Il master continua a pubblicare la posa target per lo slave usando il meccanismo di `frame_id_override`, visto che master e slave sono numericamente identici.

## Slave

Lo slave deve seguire le equazioni delle specifiche, cioe una legge basata solo su errore di posa e forza esterna.

### Parte traslazionale

Proposta diretta:

$$
F_{virtuale} = K_s \cdot (x_{target} - x_s)
$$

$$
v_s = \frac{1}{D_s} \left(F_{virtuale} - F_{esterna}\right)
$$

ovvero:

$$
v_s = \frac{K_s}{D_s}(x_{target} - x_s) - \frac{1}{D_s}F_{esterna}
$$

Questo implica che nello slave:

- non c'e feedforward del twist del master
- non c'e integrale sulla posizione
- non c'e PID legacy
- la compliance nasce direttamente dal termine con la F/T esterna

### Parte rotazionale

Per la parte angolare si propone lo stesso schema:

$$
\tau_{virtuale} = K_{s,\omega} \cdot e_o
$$

$$
\omega_s = \frac{1}{D_{s,\omega}} \left(\tau_{virtuale} - \tau_{esterna}\right)
$$

dove `e_o` e l'errore orientazionale in asse-angolo.

### Abilitazione separata traslazione / rotazione

Il nodo slave avra due flag espliciti:

- `enable_translation`
- `enable_rotation`

Comportamento previsto:

- se `enable_translation=false`, la parte lineare del comando e zero
- se `enable_rotation=false`, la parte angolare del comando e zero

Gli stessi flag esisteranno anche sul master.

## Scelte implementative consigliate

### 1. Posizione target dello slave

La posa target pubblicata dal master dovrebbe essere, nella prima versione, la posa attuale misurata del TCP master espressa nel frame base del master ma pubblicata con `frame_id` dello slave.

Motivo:

- e la soluzione piu semplice
- riusa la logica gia testata di `frame override`
- evita di introdurre uno stato virtuale aggiuntivo per il target dello slave

### 2. Wrench e frame

Si mantiene il comportamento gia validato:

- la forza e il momento vengono letti nel frame del tool
- vengono ruotati nel frame base
- i calcoli di controllo vengono eseguiti in modo coerente nel frame base

Questo vale sia per il master sia per lo slave.

### 3. Filtraggio e deadband

Si mantiene la stessa filosofia del pacchetto attuale:

- filtro esponenziale parametrizzato in `Hz`
- deadband sul modulo con isteresi
- clamp del modulo di forza e momento

### 4. Robustezza runtime

Anche se il nuovo pacchetto non deve avere "sicurezze" aggiuntive di contatto, conviene mantenere alcune protezioni infrastrutturali che non cambiano la legge di controllo:

- se TF fallisce, il nodo salta il campione
- se gli input diventano stale, il comando pubblicato va a zero
- `dt` viene clampato e suddiviso in substep

Queste non sono logiche di controllo aggiuntive, ma solo robustezza numerica e di esecuzione.

## Struttura proposta del pacchetto

```text
teleoperation_simplified/
  CMakeLists.txt
  package.xml
  include/teleoperation_simplified/core/
    math_utils.hpp
    tf_utils.hpp
    wrench_utils.hpp
    jerk_limiter.hpp
    types.hpp
    wrench_debug_publisher.hpp
  src/
    teleop_master_admittance_node.cpp
    teleop_slave_pose_admittance_node.cpp
  launch/
    one_arm_pair_real.launch
    dual_arm_real.launch
  config/
    master.yaml
    slave.yaml
```

## Launch file proposti

### `one_arm_pair_real.launch`

Launch base per una sola coppia master/slave.

Argomenti:

- `master_robot_ns`
- `slave_robot_ns`
- `arm_ns`
- `master_wrench_topic`
- `slave_wrench_topic`
- `master_command_topic`
- `slave_command_topic`
- `master_tool_frame`
- `master_base_frame`
- `slave_tool_frame`
- `slave_base_frame`

Topic interni proposti:

- `/teleop/<pair_name>/target_pose`
- `/teleop/<pair_name>/slave_actual_pose`
- `/teleop_debug/<pair_name>/master_wrench_filtered`
- `/teleop_debug/<pair_name>/slave_wrench_filtered`

### `dual_arm_real.launch`

Instanzia due volte `one_arm_pair_real.launch`:

- coppia `left`
- coppia `right`

Non include:

- zero loadcell
- MoveIt home
- calibrazioni
- utility accessorie

## Parametri minimi proposti

## Master

### Parametri di controllo

- `enable_translation`
- `enable_rotation`
- `mass_linear`
- `damping_linear`
- `mass_angular`
- `damping_angular`
- `spring_stiffness_linear`
- `spring_stiffness_angular`

### Parametri di I/O e runtime

- `control_rate`
- `tf_timeout_s`
- `wrench_timeout_s`
- `wrench_filter_cutoff_hz`
- `force_deadband_enter`
- `force_deadband_exit`
- `torque_deadband_enter`
- `torque_deadband_exit`
- `cross_deadband_scale`
- `max_force`
- `max_torque`
- `max_linear_speed`
- `max_angular_speed`
- `max_linear_accel`
- `max_angular_accel`
- `max_linear_jerk`
- `max_angular_jerk`
- `dt_min_factor`
- `dt_max_factor`
- `dt_use_substepping`
- `dt_max_substeps`
- `publish_filtered_wrench_debug`

## Slave

### Parametri di controllo

- `enable_translation`
- `enable_rotation`
- `stiffness_linear`
- `damping_linear`
- `stiffness_angular`
- `damping_angular`

### Parametri di I/O e runtime

- `control_rate`
- `tf_timeout_s`
- `target_pose_timeout`
- `wrench_timeout`
- `wrench_filter_cutoff_hz`
- `force_deadband_enter`
- `force_deadband_exit`
- `torque_deadband_enter`
- `torque_deadband_exit`
- `cross_deadband_scale`
- `max_force`
- `max_torque`
- `max_linear_speed`
- `max_angular_speed`
- `max_linear_accel`
- `max_angular_accel`
- `max_linear_jerk`
- `max_angular_jerk`
- `dt_min_factor`
- `dt_max_factor`
- `dt_use_substepping`
- `dt_max_substeps`
- `publish_filtered_wrench_debug`

## Parametri eliminati esplicitamente

Nel nuovo pacchetto non ci saranno:

- `slave_wrench_topic` nel nodo master come ingresso di controllo
- `force_reflection_scale`
- `torque_reflection_scale`
- `dynamic_damping/*`
- `dynamic_mass/*`
- `coupling_wrench_topic`
- `publish_slave_targets` come opzione generale
- `slave_feedforward_twist_topic`
- `k_ff`
- `spring_ki_linear`
- `spring_ki_angular`
- `integral_max_*`
- `integral_force_freeze`
- `integral_torque_freeze`
- `alpha_mode`
- `force_limiter/*`
- `torque_limiter/*`
- `hard_force_threshold`
- `hard_force_duration`
- `hard_guard_action`
- `retreat_speed`
- `pid/*`
- tutti i parametri per asse tipo `*_xyz`

## Dipendenze ROS minime attese

Il nuovo pacchetto dovrebbe dipendere solo da cio che serve davvero:

- `roscpp`
- `geometry_msgs`
- `std_msgs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `Eigen3`

Non dovrebbero servire dipendenze MoveIt per i due nodi di controllo.

## Strategia di implementazione

### Fase 1

Creare il nuovo pacchetto e copiare solo le utility comuni davvero utili:

- `types.hpp`
- `math_utils.hpp`
- `tf_utils.hpp`
- `wrench_utils.hpp`
- `jerk_limiter.hpp`
- `wrench_debug_publisher.hpp`

adattandole al nuovo namespace del pacchetto.

### Fase 2

Implementare `teleop_master_admittance_node.cpp`:

- partendo dal nodo master attuale
- rimuovendo tutta la logica relativa alla F/T dello slave
- rimuovendo tutta la logica di massa e smorzamento dinamici
- rimuovendo coupling
- lasciando solo il feedback di posa dallo slave
- pubblicando solo `target_pose` e non piu `feedforward_twist`

### Fase 3

Implementare `teleop_slave_pose_admittance_node.cpp`:

- partendo dal nodo slave attuale
- eliminando completamente il ramo PID legacy
- eliminando il ramo feedforward
- eliminando integralita e logiche di molla + integrale
- implementando la legge:
  - errore di posa -> forza virtuale
  - forza virtuale meno forza esterna
  - divisione per damping virtuale
- eliminando limiter e hard guard

### Fase 4

Creare `one_arm_pair_real.launch` con naming pulito e riuso semplice.

### Fase 5

Creare `dual_arm_real.launch` come semplice duplicazione della coppia sinistra/destra.

### Fase 6

Tuning e validazione:

- test master da solo
- test slave da solo con target statico
- test coppia singola in free-space
- test coppia singola in contatto
- test dual arm

## Piano di validazione suggerito

### Test 1: coerenza frame

Verificare che:

- il target pubblicato dal master abbia il `frame_id` override corretto
- lo slave consumi la posa senza richiedere una trasformazione geometrica esplicita tra robot identici

### Test 2: slave in free-space

Verificare che:

- a errore di posa nullo lo slave pubblichi velocita circa nulla
- a errore di posa costante lo slave converga verso il target

### Test 3: slave in contatto

Verificare che:

- la velocita si riduca coerentemente con il termine `-F_esterna / D_s`
- il comportamento risulti compliant e non rigido

### Test 4: master aptico

Verificare che:

- il master dia feedback tramite differenza di posa
- l'assenza della F/T dello slave nel controllo non rompa la sensazione generale desiderata

### Test 5: abilitazione selettiva

Verificare separatamente:

- solo traslazione attiva
- solo rotazione attiva
- entrambe attive

## Punti aperti aggiuntivi

Questi sono, al momento, i pochi punti che conviene chiarire prima di implementare.

### 1. Sorgente della posa target dello slave

Proposta attuale:

- usare la posa reale misurata del TCP del master

Punto aperto:

- vuoi confermare questa scelta
- oppure vuoi che il target dello slave sia una posa "virtuale" interna integrata dal nodo master invece della posa reale misurata

La proposta consigliata, per semplicita, e usare la posa reale misurata del master.

### 2. Parametri lineari e angolari: uno per tutto o separati

Proposta attuale:

- mantenere solo parametri scalari distinti per parte lineare e parte angolare

Quindi:

- un `stiffness_linear`
- un `damping_linear`
- un `stiffness_angular`
- un `damping_angular`

Punto aperto:

- confermare che non vuoi neppure rigidezze/smorzamenti diversi sui singoli assi dello slave

La proposta consigliata e no, cioe niente parametri per asse.

### 3. Frequenza di pubblicazione del target pose

Proposta attuale:

- pubblicare la posa target alla stessa frequenza del loop di controllo del master

Punto aperto:

- vuoi un parametro separato di publish rate
- oppure teniamo un unico `control_rate` per semplificare

La proposta consigliata e un solo `control_rate`.

### 4. Topic di debug della F/T filtrata dello slave

Proposta attuale:

- la F/T filtrata dello slave viene pubblicata solo dal nodo slave
- il master non la usa nel controllo

Punto aperto:

- confermare che questo e sufficiente per il monitoraggio che vuoi

### 5. Nome finale del pacchetto

Proposta attuale:

- `teleoperation_simplified`

Punto aperto:

- scegliere se usare questo nome o una variante piu corta

## Raccomandazione finale

La scelta piu pulita e costruire il nuovo pacchetto come una piccola riscrittura controllata, non come semplice copia del pacchetto attuale.

La logica da riusare e soprattutto infrastrutturale:

- trasformazioni TF
- filtraggio wrench
- deadband
- limiter cinematici
- gestione del tempo

La logica di controllo, soprattutto lato slave, conviene invece riscriverla in modo esplicito attorno alle equazioni obiettivo, cosi il nuovo pacchetto nasce davvero semplice e non eredita compromessi del codice vecchio.
