# Whole-Body Print Tuning Guide

Questa guida spiega come fare il tuning dei file:

- `config/print_path_demo.yaml`
- `config/whole_body_print_demo.yaml`

Il contesto e' il launch:

- `launch/mur620_whole_body_print_mvp.launch`

La guida e' scritta per la configurazione attuale del codice:

- un solo nodo `whole_body_print_controller` gestisce path, preposition, base e riferimento del braccio;
- il braccio riceve un riferimento continuo su `target_state`;
- in preposition il braccio viene tenuto fermo e la base fa il riallineamento;
- in tracking il controller whole-body genera la parte di moto che deve fare il braccio rispetto alla base.

## 1. Obiettivo del tuning

L'obiettivo non e' "muovere tutto un po'".

L'obiettivo corretto e' questo:

1. il frame del path viene catturato correttamente;
2. la base raggiunge una configurazione utile rispetto al primo punto del path;
3. il sistema entra in tracking senza fermarsi in preposition;
4. durante il tracking il TCP segue la traiettoria senza che il braccio vada a saturazione troppo presto;
5. la base compensa il moto lento e il braccio fa la parte veloce e locale;
6. il comportamento resta stabile, ripetibile e leggibile nei topic di debug.

## 2. Prima di toccare i parametri

Prima del tuning verifica sempre:

1. il TF `mur620b/odom -> mur620b/base_link` esiste ed e' stabile;
2. il frame `mur620b/print_origin` viene pubblicato dal nodo whole-body;
3. il controller del braccio riceve `target_state` su:
   `/mur620b/cartesian_velocity_controller_r/target_state`
4. la base riceve `cmd_vel` su:
   `/mur620b/cmd_vel`
5. il topic debug esiste:
   `/mur620b/whole_body_print_controller/debug`

I segnali minimi da osservare durante il tuning sono:

- `state`
- `path_progress`
- `target_in_base`
- `arm_tracking_scale`
- `base_command`
- `base_in_tracking_zone`
- `base_linear_saturated`
- `base_angular_saturated`

## 3. Filosofia di tuning

Non conviene fare tuning di tutto insieme.

L'ordine corretto e' questo:

1. geometria del path
2. preposition
3. zona preferita TCP rispetto alla base
4. guadagni e limiti della base
5. allocazione base-braccio
6. velocita' del path
7. eventuale avoidance

Se l'ordine viene invertito, si finisce a compensare un problema geometrico con guadagni sbagliati.

## 4. Tuning di `print_path_demo.yaml`

Questo file definisce:

- frame del path
- origine congelata del path
- geometria della traiettoria
- orientazione del TCP
- velocita' nominale e limiti cinematici del riferimento

### 4.1 `frame_id`

Parametro:

- `frame_id: "mur620b/print_origin"`

Uso:

- e' il frame in cui i waypoint del path sono definiti;
- con la configurazione attuale coincide con un frame catturato all'avvio e poi mantenuto fisso.

Quando cambiarlo:

- quasi mai;
- cambialo solo se cambi il nome logico del frame origine.

Sintomi di problema:

- il path in RViz appare "attaccato" al robot in modo sbagliato;
- il primo punto cambia apparentemente quando la base si muove;
- il target in base risulta incoerente.

### 4.2 `path_origin`

Parametri:

- `enabled`
- `capture`
- `parent_frame`
- `source_frame`
- `frame_id`
- `capture_timeout`

Uso:

- definiscono come congelare il frame `print_origin`;
- in pratica si cattura la posa della base in `odom` e la si usa come origine del path.

Regola pratica:

- lascia `parent_frame: mur620b/odom`;
- lascia `source_frame: mur620b/base_link`;
- usa `capture: node_start` se vuoi che il frame sia fissato al lancio;
- usa `capture: start` solo se vuoi fissarlo al momento del resume/start.

Tuning:

- `capture_timeout` va aumentato solo se all'avvio i TF arrivano in ritardo.

Sintomi di problema:

- stato `waiting_origin`;
- marker del path non pubblicati;
- warning TF al lancio.

### 4.3 `waypoint_mode`

Valore attuale:

- `first_absolute_then_relative`

Uso:

- il primo waypoint e' assoluto nel frame del path;
- i successivi sono offset rispetto al primo punto.

Quando e' utile:

- per descrivere traiettorie locali senza dover riscrivere coordinate assolute lunghe.

Rischio tipico:

- dimenticare che i punti dopo il primo sono offset, e mettere coordinate assolute pensando che lo siano.

Controllo:

- in RViz il marker del path deve coincidere con il disegno atteso;
- se il path e' "piu' lungo" o "spostato" del previsto, il primo sospetto e' questo parametro.

### 4.4 `waypoints`

Uso:

- definiscono la geometria reale del path del TCP.

Regola di tuning fondamentale:

- prima si definisce una traiettoria corta e facile;
- poi si allunga;
- solo alla fine si aumenta la velocita'.

Procedura consigliata:

1. usa un primo punto vicino alla zona utile della base;
2. imposta un path rettilineo corto, per esempio 0.2 m o 0.4 m;
3. fai il tuning della base su quel caso semplice;
4. solo dopo estendi il path.

Perche':

- se il primo punto e' troppo lontano, il sistema puo' spendere tutto il tempo in preposition;
- se il path e' subito lungo, non capisci se il problema e' la geometria o il tracking.

Nel tuo file attuale:

- il primo punto e' `position: [2.5, -1.0, 1.3]`

Questo e' aggressivo per una prima taratura. Per un tuning serio conviene partire piu' vicino.

### 4.5 `orientation_rpy`

Uso:

- definisce l'orientazione del TCP lungo tutto il path.

Valore attuale:

- `[pi, 0, 0]`

Quando toccarlo:

- solo se l'utensile deve cambiare orientazione nominale;
- non usarlo per compensare errori di tracking o problemi della base.

Sintomi di problema:

- il TCP raggiunge i punti ma con assetto utensile sbagliato;
- il braccio sembra "contorcersi" pur avendo target di posizione plausibili.

### 4.6 `speed`

Uso:

- velocita' nominale di avanzamento lungo il path.

Regola di tuning:

- e' uno degli ultimi parametri da aumentare;
- non partire da qui se la base non segue.

Procedura:

1. inizia con un valore basso, per esempio `0.02` o `0.03`;
2. verifica che base e braccio completino il path;
3. aumenta gradualmente fino al punto in cui compaiono:
   - saturazione della base
   - errore TCP persistente
   - `arm_tracking_scale` che cala troppo spesso

Segnale chiave:

- se abbassando `speed` tutto diventa stabile, il problema non e' la geometria ma la dinamica.

### 4.7 `max_linear_velocity`, `max_linear_acceleration`, `max_linear_jerk`

Uso:

- limitano il profilo del riferimento lungo il path.

Interpretazione:

- `speed` e' la velocita' nominale desiderata;
- questi parametri definiscono quanto velocemente il riferimento puo' accelerare e cambiare accelerazione.

Procedura:

- tienili conservativi all'inizio;
- altrimenti il path manager genera una dinamica troppo "pronta" per base e braccio.

Consiglio pratico:

- se il moto parte con colpi o cambia troppo aggressivamente, riduci prima accelerazione e jerk, non solo `speed`.

### 4.8 `blend_tolerance`

Uso:

- ammorbidisce gli spigoli tra segmenti.

Effetto:

- maggiore blending:
  - traiettoria piu' liscia
  - meno richieste impulsive alla base
  - meno errori nei cambi di segmento
- troppo blending:
  - il path reale si discosta dal disegno nominale

Procedura:

1. tuning iniziale con valori piccoli;
2. aumenta solo se vedi che gli spigoli generano rallentamenti o oscillazioni.

## 5. Tuning di `whole_body_print_demo.yaml`

Questo file definisce:

- comportamento del nodo whole-body
- logica di preposition
- allocazione base-braccio
- limiti e guadagni della base
- avoidance della base
- lifter

### 5.1 `start_paused`

Uso:

- se `false`, il nodo parte pronto a eseguire;
- se `true`, devi fare resume esplicito.

Per tuning:

- usare `true` e' spesso meglio se vuoi controllare quando catturare la situazione iniziale;
- usare `false` e' comodo quando l'ambiente e' gia' pronto.

### 5.2 `publish_target_pose`, `publish_target_state`, `arm_target_state_topic`

Uso:

- definiscono come il whole-body controller manda il riferimento al braccio.

Configurazione corretta per questa applicazione:

- `publish_target_pose: false`
- `publish_target_state: true`

Non riattivare `target_pose` in questo flusso.

Motivo:

- riporta dentro la vecchia semantica da planner a waypoint;
- confonde il comportamento del braccio;
- rende il debugging meno leggibile.

### 5.3 `preposition`

Parametri:

- `enabled`
- `reached_mode`
- `base_target_tolerance`
- `position_tolerance`
- `dwell_s`

Uso:

- regolano quando il sistema considera completato il riallineamento iniziale.

#### `reached_mode`

Valori:

- `base_target`
- `tcp`

Per questa applicazione consiglio:

- `base_target`

Motivo:

- vuoi che la base porti il target TCP nella zona preferita rispetto al robot;
- e' il criterio piu' coerente per avviare il tracking whole-body.

#### `base_target_tolerance`

Uso:

- tolleranza nel frame base rispetto a `preferred_tcp_x/y`.

Tuning:

- troppo piccolo:
  - il sistema resta in preposition troppo a lungo;
  - la base sembra "non finire mai";
- troppo grande:
  - il tracking parte quando la base e' ancora fuori posizione;
  - il braccio deve compensare troppo presto.

Procedura:

1. parti largo, per esempio `0.20` o `0.25`;
2. verifica che il sistema entri in tracking;
3. poi stringi gradualmente.

#### `position_tolerance`

Uso:

- conta solo se `reached_mode: tcp`.

Per il tuo caso ha priorita' bassa.

#### `dwell_s`

Uso:

- tempo di stabilizzazione dopo che il preposition e' stato considerato raggiunto.

Tuning:

- aumenta se vedi transizioni brusche tra preposition e tracking;
- riduci se il sistema perde solo tempo fermo.

### 5.4 `rates`

Parametri:

- `path`
- `base`
- `lifter`
- `debug`

Uso:

- definiscono la frequenza dei sottosistemi nel nodo whole-body.

Regola:

- non usare i rate come primo strumento di tuning dinamico;
- prima sistema geometria e guadagni.

Quando toccarli:

- se la base reagisce troppo lentamente pur con guadagni corretti;
- se il debug e' troppo raro;
- se l'avoidance o il lifter richiedono frequenze diverse.

### 5.5 `tracking.kp_position`

Uso:

- e' il termine proporzionale che trasforma l'errore TCP in una richiesta di velocita' del sistema esterno base+lifter.

Effetto:

- alto:
  - la base reagisce in modo piu' aggressivo agli errori
  - piu' rischio di saturazioni e oscillazioni
- basso:
  - la base segue in ritardo
  - il braccio compensa troppo

Procedura:

1. con path corto e `speed` bassa, regola questo parametro finche' la base segue in modo deciso ma non nervoso;
2. controlla in debug:
   - `base_command`
   - `base_linear_saturated`
   - `base_angular_saturated`

Se saturi spesso, il problema puo' essere:

- `kp_position` troppo alto
- limiti base troppo bassi
- `preferred_tcp_x/y` mal scelti

### 5.6 Zona base-braccio

Parametri:

- `arm_full_x_error`
- `arm_full_y_error`
- `arm_start_x_error`
- `arm_start_y_error`
- `arm_far_scale`
- `arm_gate_only_preposition`

Uso:

- definiscono come il controllo divide il lavoro tra base e braccio.

Interpretazione:

- dentro `full_*`: il braccio segue completamente il target;
- fuori `start_*`: il braccio viene limitato secondo `arm_far_scale`;
- in mezzo: blending progressivo.

#### `arm_full_x_error`, `arm_full_y_error`

Uso:

- definiscono la zona in cui il target e' "comodamente" raggiungibile dal braccio senza chiedere troppo alla base.

Tuning:

- se troppo piccoli:
  - il sistema delega troppo presto alla base;
  - il braccio lavora poco;
- se troppo grandi:
  - il braccio compensa troppo;
  - la base segue in ritardo e arriva tardi.

Procedura:

1. scegli una zona che rappresenti davvero la zona comoda del TCP rispetto a `base_link`;
2. guarda in RViz il marker `preferred_tcp_marker`;
3. osserva `target_in_base` e `base_in_tracking_zone`.

#### `arm_start_x_error`, `arm_start_y_error`

Uso:

- definiscono quando il gating del braccio diventa forte.

Regola:

- devono essere chiaramente piu' grandi di `arm_full_*`.

Effetto:

- aumentandoli, il braccio resta attivo su una zona piu' ampia;
- riducendoli, deleghi di piu' alla base.

#### `arm_far_scale`

Uso:

- scala minima del contributo del braccio quando il target e' molto fuori zona.

Scelte pratiche:

- `0.0`: il braccio viene praticamente congelato fuori zona;
- `0.1 - 0.3`: il braccio puo' fare una piccola compensazione anche fuori zona.

Per il tuo caso:

- partire da `0.0` e' la scelta piu' pulita.

#### `arm_gate_only_preposition`

Uso:

- se `true`, il gating pesante del braccio vale solo nel preposition;
- durante il tracking il braccio resta pienamente attivo.

Per la tua applicazione questa scelta e' sensata:

- preposition base-only;
- tracking whole-body vero.

### 5.7 `solver`

Parametri:

- `damping`
- `task_weight_z`

#### `damping`

Uso:

- regolarizzazione del problema di allocazione base/lifter.

Effetto:

- alto:
  - soluzione piu' smorzata
  - meno aggressivita'
  - piu' bias verso i riferimenti secondari
- basso:
  - risposta piu' pronta
  - piu' sensibilita' al rumore

Cambialo solo dopo aver sistemato i guadagni principali.

#### `task_weight_z`

Uso:

- riduce il peso della dinamica verticale nel controllo whole-body esterno.

Se il lifter e' disabilitato, questo parametro ha impatto limitato.

### 5.8 `base`

Questa e' la sezione piu' importante dopo la geometria del path.

#### `preferred_tcp_x`, `preferred_tcp_y`

Uso:

- definiscono dove vuoi vedere il target TCP nel frame `base_link`.

Interpretazione:

- `x`: avanti/dietro
- `y`: laterale sinistra/destra

Questo e' il cuore del tuning geometrico.

Procedura:

1. porta il robot in una posa comoda di lavoro;
2. osserva dove il TCP lavora bene rispetto alla base;
3. scegli quella posizione come target preferito;
4. poi costruisci il path attorno a quella zona.

Errore tipico:

- usare `preferred_tcp_x/y` per "forzare" la base a compensare un path mal definito.

Non farlo.

Prima si sistema il path, poi si definisce la zona preferita.

#### `max_linear_velocity`, `max_angular_velocity`

Uso:

- limiti di velocita' della base.

Tuning:

- se troppo bassi:
  - la base non segue mai e il braccio si ferma o compensa troppo;
- se troppo alti:
  - la base risponde in modo nervoso o poco sicuro.

Procedura:

1. usa valori conservativi ma realistici per la piattaforma;
2. controlla se `base_linear_saturated` o `base_angular_saturated` restano spesso veri;
3. se saturi sempre, o riduci `speed` oppure aumenti questi limiti entro margini sicuri.

#### `max_linear_acceleration`, `max_angular_acceleration`

Uso:

- limitano il rateo di cambiamento del comando base.

Effetto:

- troppo bassi:
  - la base parte tardi
  - sembra "seduta"
- troppo alti:
  - il moto diventa brusco

Questi parametri sono molto importanti quando il robot "si muove un po' e poi resta indietro".

#### `weight_linear`, `weight_angular`

Uso:

- pesi della regolarizzazione del solver per base lineare e base angolare.

Interpretazione pratica:

- valori piu' alti penalizzano il movimento di quella componente;
- valori piu' bassi la rendono piu' disponibile a muoversi.

Se la base non ruota abbastanza:

- puo' servire ridurre `weight_angular`
- oppure aumentare `k_lateral` o `k_heading`

Se la base non avanza abbastanza:

- puo' servire ridurre `weight_linear`
- oppure aumentare `k_preferred_x`

#### `k_preferred_x`

Uso:

- spinge la base avanti/indietro per riportare il target sulla coordinata `preferred_tcp_x`.

Sintomi:

- troppo basso:
  - la base resta arretrata
  - il braccio compensa troppo in x
- troppo alto:
  - la base accelera troppo in avanti
  - overshoot in x

#### `k_lateral`

Uso:

- corregge l'errore laterale del target nel frame base.

Sintomi:

- troppo basso:
  - il target resta fuori asse lateralmente;
- troppo alto:
  - la base ruota troppo o zigzaga.

#### `k_heading`

Uso:

- allinea la base alla tangente del path.

Effetto:

- alto:
  - la base cerca di orientarsi rapidamente lungo il path
- basso:
  - la base segue la traiettoria ma resta mal orientata

Se il path e' rettilineo, questo parametro si vede poco.
Se il path curva o cambia direzione, diventa importante.

#### `allow_reverse`

Uso:

- se `false`, la base non va indietro.

Per tuning iniziale e sicurezza:

- conviene lasciarlo `false`

Riattivarlo solo se sai che la manovra richiede davvero retromarcia.

#### `align_to_path`

Uso:

- abilita il termine di heading basato sulla tangente del path.

Per percorsi di stampa in avanti:

- di solito conviene `true`

#### `target_filter_tau`

Uso:

- filtra il target usato dalla base.

Effetto:

- alto:
  - base piu' lenta ma piu' liscia
- basso:
  - base piu' reattiva ma piu' nervosa

Questo parametro e' utile quando:

- il braccio segue bene ma la base appare in ritardo costante;
- oppure la base oscilla cercando di inseguire troppo fedelmente il target.

### 5.9 `base_avoidance`

Per ora e' disabilitata:

- `enabled: false`

Consiglio:

- non fare tuning di questa parte finche' non hai un tracking nominale pulito senza avoidance.

Solo dopo:

1. abilita avoidance;
2. parti con limiti conservativi;
3. verifica che non interferisca con il preposition.

### 5.10 `lifter`

Per ora e' disabilitato.

Non va toccato in questa fase.

## 6. Procedura pratica di tuning consigliata

### Fase A - Path semplice

Imposta:

1. primo punto vicino alla zona utile;
2. traiettoria corta e rettilinea;
3. `speed` bassa;
4. avoidance disabilitata;
5. lifter disabilitato.

Obiettivo:

- entrare in tracking e completare il path.

### Fase B - Preposition

Tocca solo:

- `preferred_tcp_x`
- `preferred_tcp_y`
- `base_target_tolerance`
- eventualmente il primo waypoint

Obiettivo:

- il sistema deve uscire dal preposition in modo ripetibile.

Se non esce:

1. allarga `base_target_tolerance`;
2. avvicina il primo waypoint;
3. verifica che `preferred_tcp_x/y` siano realistici.

### Fase C - Inseguimento base

Tocca solo:

- `kp_position`
- `k_preferred_x`
- `k_lateral`
- `k_heading`
- `target_filter_tau`
- limiti/accelerazioni base

Obiettivo:

- la base segue senza saturare in modo permanente.

### Fase D - Allocazione base-braccio

Tocca solo:

- `arm_full_*`
- `arm_start_*`
- `arm_far_scale`

Obiettivo:

- il braccio lavora bene nella zona utile;
- la base non arriva sempre tardi;
- il TCP non si "strappa".

### Fase E - Aumento prestazioni

Solo adesso tocca:

- `speed`
- `max_linear_velocity`
- `max_linear_acceleration`
- `blend_tolerance`

Obiettivo:

- aumentare produttivita' senza perdere stabilita'.

## 7. Sintomi tipici e contromisure

### Sintomo: resta in preposition troppo a lungo

Controlla:

- primo waypoint troppo lontano
- `preferred_tcp_x/y` irrealistici
- `base_target_tolerance` troppo stretta

Azioni:

- avvicina il primo punto
- allarga `base_target_tolerance`
- correggi `preferred_tcp_x/y`

### Sintomo: il braccio parte ma la base resta indietro

Controlla:

- `base_command`
- saturazioni base
- `speed`
- `k_preferred_x`
- `k_lateral`
- `target_filter_tau`

Azioni:

- abbassa `speed`
- aumenta reattivita' base
- riduci filtro se troppo lento
- verifica limiti base troppo conservativi

### Sintomo: la base si muove nervosamente

Controlla:

- `kp_position`
- `k_lateral`
- `k_heading`
- `target_filter_tau`
- accelerazioni base

Azioni:

- riduci guadagni
- aumenta `target_filter_tau`
- riduci accelerazioni massime

### Sintomo: il braccio compensa troppo e arriva al limite

Controlla:

- `arm_full_*`
- `arm_start_*`
- `preferred_tcp_x/y`
- ritardo della base

Azioni:

- restringi la zona di tracking pieno del braccio
- rendi la base piu' efficace
- sposta il target preferito in una zona piu' comoda

## 8. Configurazione minima per partire bene

Per il primo tuning sul robot vero conviene:

1. accorciare molto il path;
2. ridurre `speed` a un valore molto basso;
3. usare `base_target_tolerance` relativamente larga;
4. tenere `arm_far_scale: 0.0`;
5. lasciare avoidance e lifter disabilitati.

Questo non e' il setup finale.
E' il setup corretto per capire il sistema.

## 9. Checklist finale

Una configurazione ben tarata per questa applicazione deve soddisfare tutte queste condizioni:

1. il frame origine viene catturato senza warning TF;
2. il sistema entra in `preposition`;
3. il sistema passa a `dwell`;
4. il sistema entra in `tracking`;
5. `path_progress` cresce in modo regolare;
6. `base_command` non resta saturato costantemente;
7. `arm_tracking_scale` non crolla spesso a zero durante il tracking;
8. il TCP completa il path senza fermarsi a meta'.

## 10. Consiglio finale

Se devi cambiare spesso sia il path sia la dinamica, salva sempre una configurazione "baseline" che sai gia' funzionare:

- path corto
- `speed` bassa
- preposition robusto
- base conservativa

Quella baseline ti serve come test di regressione.

Se una modifica rompe tutto, torni li' e capisci subito se il problema e':

- geometrico
- dinamico
- di allocazione base-braccio
- di frame/TF

