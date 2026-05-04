# Piano per scalatura movimento e doppio master

Questo documento propone come implementare le due funzionalita richieste in
`teleoperation/Spec.md` mantenendo il comportamento attuale come default.

## Stato attuale rilevante

Il percorso runtime usato dai launch reali e questo:

- `teleop_master_haptic_controller` comanda il master con ammettenza e feedback
  aptico.
- Lo stesso nodo, se `publish_slave_targets=true`, pubblica verso lo slave:
  `slave_target_pose_topic` (`geometry_msgs/PoseStamped`) e
  `slave_feedforward_twist_topic` (`geometry_msgs/TwistStamped`).
- `teleop_slave_twist_outer_loop` consuma target pose + twist feedforward e
  genera il twist del controller cartesiano dello slave.
- Lo slave pubblica la propria posa TCP su `master_feedback_pose_topic`; il
  master la usa per il virtual spring coupling (`spring_stiffness_*`).

Il launch `master_slave_mur620b_real.launch` usa una scorciatoia intenzionale:
il master pubblica la propria posa numerica in un frame master, ma la etichetta
come frame slave tramite `slave_frame_id_override`. Nel launch dual-real e gia
presente una rotazione statica `slave_pose_twist_rotation_rpy` per allineare le
basi tra robot.

Questa architettura funziona, ma oggi il mapping master->slave e rigido:

```text
p_slave_target = R_ms * p_master
q_slave_target = R_ms * q_master
v_slave_ff     = R_ms * v_master
w_slave_ff     = R_ms * w_master
```

Non esiste ancora il concetto di anchor iniziale o di scala indipendente tra
traslazione e rotazione.

## Conflitti da gestire

Il conflitto principale non e nel controller dello slave, ma nel feedback aptico
del master.

Se si scala solo il target verso lo slave:

```text
p_slave_target = p_slave0 + (p_master - p_master0) / scale
```

allora lo slave reale non si trovera piu numericamente nella stessa posa del
master. Il master pero oggi calcola la molla aptica confrontando:

```text
p_master - p_slave_actual
```

Con scala diversa da 1 questo errore diventerebbe artificiale e il master
spingerebbe l'operatore verso una posa sbagliata. Quindi la scalatura deve
essere applicata anche al canale inverso usato dal feedback:

```text
p_slave_actual_equiv =
  p_master0 + scale_translation * (p_slave_actual - p_slave0)
```

Lo stesso vale per l'orientamento, usando log/exp quaternion invece di scalare
Euler o componenti quaternion.

Altro conflitto: con due master non possono esistere due publisher sullo stesso
`target_pose_topic`/`feedforward_twist_topic` finale dello slave. Ogni master
deve pubblicare su topic candidato separato, e un solo nodo deve produrre il
target finale.

## Architettura consigliata

Consiglio di introdurre un nodo dedicato di mapping/aggregazione, senza
modificare subito la logica interna dello slave:

```text
master haptic node(s)
  -> candidate pose/twist in common master frame
  -> teleop_target_mapper
  -> final slave target pose/twist
  -> teleop_slave_twist_outer_loop

slave actual pose
  -> teleop_target_mapper inverse feedback
  -> mapped slave feedback pose
  -> master haptic node(s)
```

Nome suggerito:

```text
teleop_target_mapper
```

Responsabilita del nodo:

- ricevere una o piu pose/twist candidate dai master;
- trasformarle in un frame comune master definito da parametro;
- opzionalmente calcolare la media tra master;
- applicare scaling traslazionale e rotazionale attorno ad anchor iniziali;
- pubblicare target pose/twist finali nel frame dello slave;
- pubblicare la posa slave reale rimappata nello spazio master-equivalente per
  il virtual spring coupling del master.

Questo mantiene invariati:

- `teleop_slave_twist_outer_loop`;
- la semantica dei suoi ingressi;
- il comportamento attuale quando il nuovo nodo non viene lanciato o quando
  `scale_translation=1`, `scale_rotation=1`, `input_count=1`.

## Fase 1: scalatura single-master

Implementerei prima questa fase.

### Dati in ingresso

Il master deve pubblicare target grezzi su topic intermedi, non direttamente
sui topic finali dello slave:

```yaml
raw_master_pose_topic: /teleop/<pair>/raw_master_pose
raw_master_twist_topic: /teleop/<pair>/raw_master_twist
```

Il nodo mapper pubblica poi:

```yaml
target_pose_topic: /teleop/<pair>/target_pose
feedforward_twist_topic: /teleop/<pair>/feedforward_twist
mapped_slave_feedback_pose_topic: /teleop/<pair>/slave_actual_pose_mapped
```

Il master haptic deve ricevere `slave_actual_pose_topic` dal topic
`slave_actual_pose_mapped`, non dal feedback grezzo dello slave.

### Anchor

Parametri consigliati:

```yaml
scaling:
  enabled: true
  translation_scale: 2.0
  rotation_scale: 1.0
  anchor_mode: auto_on_first_valid_sample  # oppure manual/service
  reset_service: reset_anchor
```

Semantica:

- `translation_scale = 2.0`: 50 cm master diventano 25 cm slave.
- `rotation_scale = 2.0`: 20 gradi master diventano 10 gradi slave.
- valori <= 0 devono essere rifiutati e sostituiti con 1.0, con warning.

Anchor da salvare:

```text
T_M0: posa iniziale master aggregata nel common_master_frame
T_S0: posa iniziale slave nel slave_target_frame
```

In `auto_on_first_valid_sample`, il nodo deve aspettare di avere:

- posa/twist master fresca;
- posa TCP slave da TF o feedback fresco;
- TF disponibili tra frame configurati.

Poi cattura `T_M0` e `T_S0`. Un servizio `std_srvs/Trigger` deve permettere di
ricatturare gli anchor senza riavviare i nodi.

### Formula traslazionale

Con le pose gia espresse nei frame corretti:

```text
p_target_slave = p_S0 + (p_master - p_M0) / translation_scale
v_target_slave = v_master / translation_scale
```

Se serve una rotazione statica tra frame master comune e frame slave:

```text
p_master_mapped = R_ms * p_master
p_M0_mapped     = R_ms * p_M0
p_target_slave  = p_S0 + (p_master_mapped - p_M0_mapped) / translation_scale
v_target_slave  = R_ms * v_master / translation_scale
```

### Formula rotazionale

Non usare differenze Euler ne scaling delle componenti quaternion.

Procedura robusta:

```text
q_master_mapped = R_ms * q_master
q_delta         = inverse(q_M0_mapped) * q_master_mapped
rotvec_delta    = log(q_delta)
rotvec_scaled   = rotvec_delta / rotation_scale
q_target_slave  = q_S0 * exp(rotvec_scaled)
w_target_slave  = R_ms * w_master / rotation_scale
```

Dettagli importanti:

- allineare sempre il segno dei quaternion prima di log/slerp;
- normalizzare ogni quaternion in uscita;
- se `rotation_scale=1`, il risultato deve coincidere con il mapping attuale;
- per angoli vicini a zero usare soglia numerica e restituire identita.

### Feedback inverso per la molla master

Il feedback slave->master deve usare l'inversa della stessa mappa:

```text
p_slave_equiv_master =
  p_M0 + translation_scale * inverse(R_ms) * (p_slave_actual - p_S0)

q_slave_equiv_master =
  q_M0 * exp(rotation_scale * log(inverse(q_S0) * q_slave_actual))
```

Questo e il topic che il master haptic deve usare come
`slave_actual_pose_topic`. In questo modo la molla continua a misurare l'errore
nel sistema percepito dal master, non nello spazio fisico ridotto dello slave.

## Fase 2: due master, uno slave

Dopo la Fase 1, la Fase 2 diventa un'estensione naturale dello stesso nodo.

### Ingressi

Parametri suggeriti:

```yaml
masters:
  - name: master_left
    pose_topic: /teleop/<pair>/master_left/raw_pose
    twist_topic: /teleop/<pair>/master_left/raw_twist
    input_frame: <frame>
  - name: master_right
    pose_topic: /teleop/<pair>/master_right/raw_pose
    twist_topic: /teleop/<pair>/master_right/raw_twist
    input_frame: <frame>

common_master_frame: <frame comune tra i due master>
slave_target_frame: <base frame dello slave>
```

Ogni master haptic deve pubblicare su topic intermedi distinti. Nessun master
deve pubblicare direttamente sul topic finale dello slave.

### Media delle pose

Prima trasformare entrambe le pose nel `common_master_frame`.

Per la posizione:

```text
p_avg = (p_1 + p_2) / 2
```

Per l'orientamento, con due quaternion, usare `slerp(q1, q2, 0.5)` dopo aver
allineato il segno (`dot(q1, q2) >= 0`). Per estendere a N master, usare la
media Markley, ma per due master la slerp a meta e sufficiente e piu semplice.

Per il twist:

```text
v_avg = (v_1 + v_2) / 2
w_avg = (w_1 + w_2) / 2
```

dopo aver ruotato entrambi i twist nel frame comune.

Poi applicare la stessa pipeline della Fase 1:

```text
master_aggregated_pose/twist -> scaling -> target slave
```

Quindi l'ordine consigliato e:

```text
trasforma nel frame comune -> media -> scaling -> pubblicazione slave
```

Non fare scaling separato per ciascun master prima della media, almeno nella
prima implementazione: introduce piu casi limite e rende meno chiara la
semantica degli anchor.

### Freshness e sicurezza

Per due master consiglio una policy esplicita:

```yaml
stale_policy: require_all  # default robusto
input_timeout_s: 0.1
on_stale: hold_pose_zero_twist
```

`require_all` evita che lo slave scatti verso un solo master se l'altro perde
pacchetti. Il comportamento piu sicuro e:

- mantenere l'ultima posa target valida;
- pubblicare twist feedforward zero;
- loggare warning throttled;
- non aggiornare anchor durante stale.

Una policy opzionale `use_available` puo essere utile in laboratorio, ma non la
metterei come default.

## Integrazione nei launch

Per preservare le funzionalita attuali, non cambierei i launch esistenti come
primo passo. Aggiungerei launch nuovi o arg disabilitati di default.

Per single-master scaling:

```text
master_slave_mur620b_real_scaled.launch
```

oppure arg nel launch esistente:

```xml
<arg name="use_target_mapper" default="false" />
```

Quando `use_target_mapper=false`, il sistema resta identico.

Quando `use_target_mapper=true`:

- `teleop_master_haptic_controller` pubblica su topic raw/intermedi;
- `teleop_target_mapper` pubblica sui topic finali dello slave;
- `teleop_slave_twist_outer_loop` continua a leggere i topic finali;
- `teleop_slave_twist_outer_loop` pubblica feedback grezzo;
- `teleop_target_mapper` pubblica feedback rimappato;
- `teleop_master_haptic_controller` legge feedback rimappato.

Per due master:

```text
dual_master_single_slave.launch
```

con due istanze `teleop_master_haptic_controller`, due coppie raw pose/twist e
una sola istanza `teleop_target_mapper` in modalita `average`.

## File da aggiungere o modificare

Prima iterazione consigliata:

- nuovo nodo: `teleoperation/src/teleop_target_mapper_node.cpp`;
- utility quaternion/pose mapping in header riusabile, per esempio
  `teleoperation/include/teleoperation/core/pose_mapping.hpp`;
- aggiornamento `teleoperation/CMakeLists.txt` per compilare il nodo;
- nuovo config:
  `teleoperation/config/target_mapper_scaled_single_master.yaml`;
- nuovo launch o arg:
  `teleoperation/launch/master_slave_mur620b_real_scaled.launch`;
- documentazione rapida con topic e servizi.

Non modificherei `teleop_slave_twist_outer_loop_node.cpp` nella prima fase.
Eventuali modifiche a `teleop_master_haptic_controller_node.cpp` dovrebbero
essere minime e solo se serve rendere piu chiara la pubblicazione su topic raw.
Gia oggi il nodo permette di cambiare i topic pubblicati via parametro.

## Debug e diagnostica

Il mapper dovrebbe pubblicare, almeno opzionalmente:

```text
~debug/anchor_master_pose
~debug/anchor_slave_pose
~debug/aggregated_master_pose
~debug/scaled_target_pose
~debug/scaled_target_twist
~debug/state
```

`~debug/state` puo essere `std_msgs/String` o una piccola `Float64MultiArray`
coerente con il resto del pacchetto. Informazioni minime:

- anchor validi/non validi;
- numero master freschi;
- eta degli input;
- scala traslazionale/rotazionale effettiva;
- norm degli spostamenti master/slave.

## Test minimi

Unit test o test piccoli su funzioni pure:

- `translation_scale=2`: delta master 0.50 m -> delta slave 0.25 m;
- `rotation_scale=2`: delta master 30 deg -> delta slave 15 deg;
- `scale=1`: output identico al comportamento precedente;
- reset anchor: dopo reset, delta iniziale torna zero;
- quaternion con segno opposto: nessun salto;
- media due master: posizioni simmetriche producono centro corretto;
- stale input: posa mantenuta e twist zero.

Test runtime/manuali:

- avviare sistema con `use_target_mapper=false` e verificare topic invariati;
- avviare single-master con scala 1 e confrontare target con launch attuale;
- avviare single-master con scala 2 e verificare su RViz/rosbag il rapporto tra
  delta master e delta slave target;
- verificare che il feedback a molla non tiri il master quando master e slave
  sono coerenti nello spazio scalato.

## Ordine consigliato

Procederei cosi:

1. Implementare il mapper single-master con scaling e feedback inverso.
2. Validarlo con scala 1 per garantire retrocompatibilita.
3. Validarlo con scala traslazionale diversa da 1 e rotazionale ancora 1.
4. Abilitare e testare anche la scala rotazionale.
5. Estendere lo stesso mapper a due input master con media pose/twist.
6. Solo dopo, introdurre un launch definitivo per due master e uno slave.

Questo ordine riduce il rischio perche la Fase 1 crea gia quasi tutta la
meccanica necessaria alla Fase 2: frame comune, anchor, scaling, freshness,
pubblicazione target e feedback inverso.

