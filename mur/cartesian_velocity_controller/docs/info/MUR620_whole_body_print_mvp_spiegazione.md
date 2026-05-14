# MUR620 Whole-Body Print MVP

Questo documento spiega il funzionamento dell'applicazione avviata da:

`mur/cartesian_velocity_controller/launch/mur620_whole_body_print_mvp.launch`

Il focus e' concettuale: come il TCP segue un percorso preimpostato, come la base mobile aiuta il braccio, e come la base evita ostacoli reali e simulati. I moduli non attivi nella configurazione non sono descritti, tranne il lifter a cui e' dedicata una breve sezione finale.

## 1. Configurazione effettivamente usata

Con i valori di default del launch:

- robot namespace: `/mur620b`
- braccio usato: destro, quindi controller `cartesian_velocity_controller_r`
- nodo principale: `/mur620b/whole_body_print_controller`
- percorso TCP: `config/print_path_demo.yaml`
- parametri whole-body: `config/whole_body_print_demo.yaml`
- comando base: `/mur620b/cmd_vel`
- feedback base: `/mur620b/mobile_base_controller/odom`
- laser scan per ostacoli reali: `f_scan`, `b_scan` e `scan`
- ostacoli simulati: abilitati nel file del percorso
- lifter: configurato ma non attivo (`enabled: false`)

Il nodo parte in pausa (`start_paused: true`). Dopo il servizio `resume`, aspetta 10 s, porta il TCP sul primo punto del percorso, attende 0.3 s, poi inizia l'avanzamento lungo il path.

Schema di avvio:

```text
roslaunch
   |
   +--> cartesian_velocity_controller_r
   |       riceve target_state e comanda le velocita' dei giunti UR10_r
   |
   +--> whole_body_print_controller
           legge il path
           genera il riferimento TCP
           comanda la base
           modifica la base per evitare ostacoli
           invia al braccio il riferimento cartesiano compensato
```

## 2. Idea generale

L'applicazione lavora in velocita'. Non pianifica ogni volta una nuova traiettoria completa del robot: genera un punto TCP desiderato lungo un percorso fisso, poi corregge continuamente le velocita' di base e braccio.

Il concetto e':

```text
Percorso preimpostato
        |
        v
Target TCP p_d(s), tangente t(s), velocita' lungo path
        |
        v
Whole-body controller
        |
        +--> base: si muove per tenere il TCP in una zona preferita
        |
        +--> braccio: segue il TCP desiderato e compensa il moto della base
        |
        +--> obstacle avoidance: modifica solo cmd_vel della base
```

La base non cambia il percorso TCP: il percorso resta quello definito nel file YAML. L'avoidance modifica il modo in cui la base si muove sotto al braccio, mentre il braccio riceve una compensazione per continuare a inseguire il TCP.

## 3. Frame principali

I frame importanti sono:

- `mur620b/odom`: frame fisso odometrico della base.
- `mur620b/base_link`: frame solidale alla base mobile.
- `mur620b/print_origin`: frame del percorso di stampa.
- `UR10_r/tool0`: TCP del braccio destro.

Nel file `print_path_demo.yaml` il path origin e' abilitato:

```yaml
path_origin:
  enabled: true
  capture: "node_start"
  parent_frame: "mur620b/odom"
  source_frame: "mur620b/base_link"
  frame_id: "mur620b/print_origin"
```

Quindi, all'avvio del nodo, `print_origin` viene catturato dalla posa corrente di `base_link` rispetto a `odom`. Da quel momento `print_origin` resta fisso in `odom`.

Schema dei frame:

```text
mur620b/odom
    |
    +-- mur620b/print_origin   frame fisso del path
    |
    +-- mur620b/base_link      frame mobile della base
             |
             +-- UR10_r/.../tool0   TCP reale
```

Ogni ciclo di controllo usa trasformazioni TF del tipo:

```text
p_B = R_BA p_A + t_BA
v_B = R_BA v_A
```

dove `p_A` e' un punto espresso nel frame A, `p_B` lo stesso punto espresso nel frame B, `R_BA` e' la rotazione tra i frame e `t_BA` la traslazione.

## 4. Percorso TCP preimpostato

Il path e' definito in `print_path_demo.yaml`.

Parametri principali:

```yaml
frame_id: "mur620b/print_origin"
waypoint_mode: "first_absolute_then_incremental"
speed: 0.25
max_linear_velocity: 0.40
max_linear_acceleration: 0.80
corner_radius: 0.2
orientation_rpy: [pi, 0, 0]
```

Il primo waypoint e' assoluto nel frame `print_origin`. I successivi sono incrementi rispetto al punto precedente.

Nella configurazione attuale:

```text
P0 = [0.6, -1.0, 1.3]
P1 = P0 + [0.0, 0.0, 0.0]
P2 = P1 + [2.0, 0.0, 0.0]
poi arco circolare:
  end incrementale = [1.50, 4.50, 0.0]
  raggio = 3.0 m
  lato = right
```

Il percorso viene parametrizzato con una coordinata scalare `s`, cioe' la distanza percorsa lungo la linea/arco:

```text
p_d = p(s)
t = dp/ds
```

dove:

- `p_d` e' il punto TCP desiderato.
- `t` e' la tangente unitaria al percorso.
- `s = 0` e' l'inizio del percorso.
- `s = L` e' la fine del percorso.

Il riferimento avanza nel tempo secondo:

```text
s_dot = v_path
s(k+1) = s(k) + v_path dt
```

La velocita' lungo path non salta istantaneamente al valore nominale. Viene limitata in accelerazione:

```text
v_target = min(v_nominale, sqrt(2 a_max d_remaining))

v_path(k+1) = v_path(k) +
              clamp(v_target - v_path(k), -a_max dt, +a_max dt)
```

Con questa logica:

- il TCP accelera gradualmente;
- vicino alla fine frena in base alla distanza rimanente;
- se il robot resta troppo indietro, il path non scappa in avanti.

I blocchi di sicurezza sull'avanzamento sono:

```text
||p_d - p_tcp|| <= 0.20 m
|x_target_base - x_preferito| <= 0.75 m
|y_target_base - y_preferito| <= 0.45 m
```

Se uno di questi errori supera la soglia, `s_dot` viene portato a zero e il riferimento aspetta il robot.

Schema del path:

```text
print_origin

 y
 ^
 |
 |                         arco r=3.0
 |                    .----------------.
 |                 .-'
 |              .-'
 | P0/P1 ---- P2
 |
 +------------------------------------> x
```

## 5. Preposizionamento iniziale

Prima di avanzare sul path, il nodo porta il TCP sul primo punto.

Sequenza:

```text
resume
  |
  +-- attesa 10 s
  |
  +-- preposition:
  |      target = primo punto path
  |      base + braccio riducono l'errore TCP
  |
  +-- quando ||target - TCP|| <= 0.05 m
  |
  +-- dwell 0.3 s
  |
  +-- tracking del percorso
```

Nel preposizionamento il riferimento non avanza lungo `s`: resta fermo sul primo punto. Questo evita che il TCP inizi il percorso mentre base e braccio non sono ancora allineati.

## 6. Come il TCP segue il percorso

Durante il tracking, il controller calcola una velocita' cartesiana desiderata del TCP:

```text
v_des = v_path t + Kp (p_track - p_tcp)
```

dove:

- `v_path t` e' il feed-forward lungo la tangente del percorso.
- `Kp (p_track - p_tcp)` e' la correzione proporzionale dell'errore TCP.
- `Kp = 2.0`.
- `p_track` e' il target filtrato usato dalla base.
- l'asse Z pesa meno: `task_weight_z = 0.7`.

Il termine feed-forward fa avanzare il TCP lungo il path. Il termine proporzionale corregge gli errori dovuti a ritardi, moto base, limiti e tracking non perfetto.

Schema:

```text
target path p_d(s) ---- tangente t(s) ----> v_path t
        |                                      |
        |                                      v
        +-- errore TCP p_track - p_tcp --> Kp * errore
                                               |
                                               v
                                      v_des TCP totale
```

## 7. Come base e braccio si dividono il lavoro

La base e' trattata come un sistema planare con:

```text
u_base = [v_x, omega_z]
```

dove:

- `v_x` e' la velocita' avanti/indietro della base;
- `omega_z` e' la velocita' di rotazione yaw;
- non viene comandata una velocita' laterale diretta.

La cinematica planare ideale della base e':

```text
x_dot     = v_x cos(theta)
y_dot     = v_x sin(theta)
theta_dot = omega_z
```

Per un punto TCP a distanza `r` dall'origine della base, il moto della base produce una velocita':

```text
v_tcp_base = v_base + omega x r
```

Nel path frame, il Jacobiano semplificato della base e':

```text
J_base = [ base_x , yaw_col ]

base_x  = asse X della base espresso nel frame path
yaw_col = [-r_y, r_x, 0]^T
```

Quindi:

```text
v_tcp_generata_dalla_base = J_base [v_x, omega_z]^T
```

Il controller sceglie `v_x` e `omega_z` risolvendo un problema ai minimi quadrati regolarizzato:

```text
min_u || W^(1/2) (J u - v_des) ||^2
    + || R^(1/2) (u - u_ref) ||^2
```

La soluzione usata e':

```text
(J^T W J + R) u = J^T W v_des + R u_ref
```

dove:

- `u` contiene i comandi della base, e in futuro anche il lifter.
- `W = diag(1, 1, task_weight_z)`.
- `R` pesa quanto il comando deve restare vicino a un riferimento preferito.
- `u_ref` spinge la base a tenere il TCP in una zona comoda.

## 8. Punto TCP preferito rispetto alla base

La base cerca di mantenere il TCP vicino a un punto preferito nel proprio frame:

```yaml
preferred_tcp_x: 0.45
preferred_tcp_y: -1.1
```

Con la convenzione ROS tipica:

- `x > 0`: davanti alla base;
- `y > 0`: sinistra della base;
- `y < 0`: destra della base.

Quindi il sistema cerca di tenere il TCP circa:

```text
0.45 m davanti alla base
1.10 m a destra della base
```

Questa e' la logica che rende il sistema whole-body: la base non segue semplicemente il path come un robot mobile autonomo, ma si muove per mantenere il braccio in una regione utile di lavoro.

Schema visto dall'alto:

```text
base_link

             x avanti
                ^
                |
                |
                o  base
                 \
                  \
                   *  TCP preferito
                      x=0.45, y=-1.10

 y sinistra <--------------------> y destra
```

Il riferimento della base e':

```text
v_x_ref     = k_x (x_tcp_base - x_pref)
omega_ref   = k_y (y_tcp_base - y_pref)
            + k_heading heading_error
```

con:

```yaml
k_preferred_x: 1.2
k_lateral: 0.8
k_heading: 0.8
allow_reverse: true
align_to_path: true
```

`allow_reverse: true` permette alla base di andare anche indietro se serve.

## 9. Compensazione del moto della base nel braccio

La base si muove mentre il braccio sta inseguendo il TCP. Se il braccio ignorasse questo moto, il TCP reale tenderebbe a spostarsi rispetto al path.

Per questo il nodo calcola la velocita' del TCP causata dalla base:

```text
v_ext = R_path_base [v_x, v_y, 0]^T
      + (R_path_base [0, 0, omega_z]^T) x r
```

Poi invia al braccio:

```text
v_arm = v_des - v_ext
```

In parole semplici:

```text
velocita' che voglio per il TCP
-
velocita' gia' prodotta dal moto della base
=
velocita' che deve fare il braccio
```

La compensazione usa una fusione tra comando base finale e odometria:

```text
v_comp = (1 - w) v_cmd + w v_odom
```

con:

```yaml
odom_weight: 0.5
odom_timeout: 0.25
```

Quindi, se l'odometria e' fresca, il sistema usa meta' comando e meta' velocita' misurata. Questo aiuta quando la base reale non segue perfettamente il comando.

## 10. Controller cartesiano del braccio

Il whole-body controller pubblica su:

```text
/mur620b/cartesian_velocity_controller_r/target_state
```

Il messaggio contiene:

- posa TCP desiderata;
- velocita' cartesiana feed-forward;
- flag `active`.

Il controller del braccio lavora a 250 Hz. Per ogni ciclo:

```text
target_state
   |
   v
errore posizione + errore orientamento
   |
   v
PID cartesiano con feed-forward
   |
   v
twist TCP desiderato
   |
   v
Jacobiano inverso smorzato
   |
   v
velocita' giunti UR10_r
```

L'errore di posizione e':

```text
e_p = p_target - p_tcp
```

L'errore di orientamento e' rappresentato come asse-angolo:

```text
e_R = angle_axis(q_target q_tcp^-1)
```

Il PID cartesiano usa:

```text
v_cmd = Kp e + Ki integral(e dt) + Kd de/dt + Kff v_ff
```

Nella configurazione del braccio destro:

```yaml
pid_controller:
  position:
    kp: 2.0
    ki: 0.5
    kd: 0.0
    kff: 0.95
  orientation:
    kp: 2.0
    ki: 0.5
    kd: 0.0
    kff: 0.95
```

La conversione da velocita' TCP a velocita' dei giunti usa la cinematica differenziale:

```text
V_tcp = J(q) q_dot
q_dot = J# V_tcp
```

`J#` e' una pseudo-inversa pesata e smorzata:

```text
J# = Wq^(-1/2) (J Wq^(-1/2))#_lambda
```

Per ogni valore singolare `sigma`:

```text
sigma# = sigma / (sigma^2 + lambda^2)
```

Lo smorzamento `lambda` aumenta vicino alle singolarita'. Questo evita comandi di giunto troppo grandi quando il Jacobiano perde rango.

Dopo la pseudo-inversa, il comando passa attraverso:

- filtro di velocita' giunti;
- limiti di velocita';
- limiti di accelerazione;
- guardrail sui limiti articolari.

Il risultato viene pubblicato su:

```text
/mur620b/UR10_r/joint_group_vel_controller/command
```

## 11. Evitamento ostacoli della base

L'evitamento ostacoli e' attivo nella sezione:

```yaml
base_avoidance:
  enabled: true
  use_laser_scans: true
  distance_mode: "rounded_front"
```

Questa avoidance agisce solo sul comando della base. Non sposta il path TCP e non attiva la repulsione del braccio.

Il comando nominale della base viene prima calcolato dal whole-body controller:

```text
cmd_nom = [v_x_nom, omega_nom]
```

Poi l'avoidance lo modifica:

```text
v_x_final   = speed_scale * v_x_nom
omega_final = clamp(omega_nom + omega_avoid, -omega_max, +omega_max)
```

Quindi l'ostacolo puo':

- rallentare o fermare l'avanzamento lineare;
- aggiungere una rotazione per allontanarsi;
- lasciare invariato il percorso TCP desiderato.

## 12. Zone di reazione

Con `distance_mode: rounded_front`, la base considera una zona davanti a se', con lati rettilinei e fronte arrotondato.

Parametri attivi:

```yaml
influence_distance: 1.2
slowdown_distance: 0.8
stop_distance: 0.7

influence_lateral_window: 0.9
slowdown_lateral_window: 0.5
stop_lateral_window: 0.4

front_min_x: -0.20
```

Schema visto dall'alto:

```text
                 x avanti
                    ^
                    |
        influence   |        inizia a sterzare
      .-------------+-------------.
     /                            \
    |   slowdown                  |  rallenta
    |    .-------------------.    |
    |   /                     \   |
    |  |  stop                 |  |  ferma v_x
    |  |   .-------------.     |  |
    |  |   |             |     |  |
    +--+---o base_link---+-----+----> y
```

Per un punto laser espresso in `base_link`:

```text
p = [x, y, 0]
```

il punto viene considerato solo se:

```text
x >= front_min_x
|y| <= lateral_window
d(p) <= distance
```

Nel modo `rounded_front`, la distanza usata dal codice e':

```text
d(p) = sqrt(max(0, x^2 + y^2 - lateral_window^2))
```

Per un ostacolo simulato circolare di raggio `r`, il codice usa la distanza dalla superficie:

```text
d_surface = sqrt(max(0, (max(0, hypot(x,y) - r))^2
                       - lateral_window^2))
```

Questo fa si' che l'ostacolo reagisca come un volume, non come un singolo punto.

## 13. Ostacoli reali da laser scan

Con `use_laser_scans: true`, il nodo legge i laser scan configurati.

Per ogni misura valida:

```text
raggio laser r
angolo laser alpha

p_scan = [r cos(alpha), r sin(alpha), 0]
```

Se il frame dello scan non e' `base_link`, il punto viene trasformato:

```text
p_base = T_base_scan p_scan
```

Poi il punto viene testato nelle tre zone:

```text
influence -> puo' generare sterzata
slowdown  -> riduce v_x
stop      -> porta v_x a zero
```

La forza dell'effetto cresce quando l'ostacolo si avvicina:

```text
h = clamp((D_influence - d) /
          (D_influence - D_stop), 0, 1)
```

La sterzata generata dal punto e':

```text
omega_i = -sign(y) h^2
```

Interpretazione:

- ostacolo a sinistra (`y > 0`) -> rotazione negativa, quindi verso destra;
- ostacolo a destra (`y < 0`) -> rotazione positiva, quindi verso sinistra.

Tutti i punti attivi vengono mediati:

```text
omega_avoid_raw =
  clamp(k_omega * sum(omega_i) / sum(h),
        -max_omega, +max_omega)
```

con:

```yaml
k_omega: 0.8
max_omega: 0.25
```

La riduzione della velocita' lineare e':

```text
speed_scale = 0                         se in stop
speed_scale = clamp((d - D_stop) /
                    (D_slowdown - D_stop), 0, 1)
                                             se in slowdown
speed_scale = 1                         fuori slowdown
```

Se arrivano dati da piu' scan, il nodo combina le sorgenti:

- prende la minima distanza;
- prende lo `speed_scale` piu' restrittivo;
- pesa di piu' le sorgenti piu' vicine.

I dati vecchi vengono scartati dopo:

```yaml
stale_timeout: 0.5
```

Infine l'effetto viene filtrato nel tempo:

```text
alpha = dt / (tau + dt)
omega_f(k+1) = omega_f(k) + alpha (omega_raw - omega_f(k))
scale_f(k+1) = scale_f(k) + alpha (scale_raw - scale_f(k))
```

con:

```yaml
filter_tau: 0.3
```

## 14. Ostacoli simulati

Gli ostacoli simulati sono definiti nello stesso file del percorso:

```yaml
obstacles:
  enabled: true
  relative_to_first_waypoint: true
  relative_z: false
  items:
    - name: "sim_obstacle_1"
      position: [3.10, 1.4, 0.0]
      radius: 0.10
```

Poiche' `relative_to_first_waypoint: true`, la posizione XY e' relativa al primo waypoint.

Con il primo waypoint:

```text
P0 = [0.6, -1.0, 1.3]
```

l'ostacolo simulato risulta circa:

```text
center_xy = [0.6 + 3.10, -1.0 + 1.4]
          = [3.70, 0.40]

radius = 0.10 m
```

Il valore Z resta assoluto perche':

```yaml
relative_z: false
```

Dal punto di vista dell'avoidance, un ostacolo simulato entra nella stessa pipeline degli ostacoli reali:

```text
ostacolo simulato in print_origin
        |
        v
trasformazione in base_link
        |
        v
test zone influence/slowdown/stop
        |
        v
omega_avoid + speed_scale
        |
        v
cmd_vel finale della base
```

La differenza principale e':

- ostacolo reale: arriva come punti da LaserScan;
- ostacolo simulato: arriva come cerchio con centro e raggio.

Il comportamento finale della base e' lo stesso.

## 15. Schema completo del controllo

```text
                       +----------------------+
                       | print_path_demo.yaml |
                       +----------+-----------+
                                  |
                                  v
                          p_d(s), t(s)
                                  |
                                  v
                   +--------------+---------------+
                   | whole_body_print_controller  |
                   +--------------+---------------+
                                  |
        +-------------------------+-------------------------+
        |                         |                         |
        v                         v                         v
  base solver              arm reference             obstacle avoidance
  [v_x, omega]             pose + velocity           scan + simulated obs
        |                         |                         |
        v                         v                         |
 acceleration limit       compensation base          omega_avoid, scale
        |                         |                         |
        +-----------+             |                         |
                    |             |                         |
                    v             v                         |
               cmd_vel final   target_state                 |
                    |             |                         |
                    v             v                         |
              mobile base   cartesian_velocity_controller_r |
                                  |                         |
                                  v                         |
                           UR10_r joint velocities <--------+
```

## 16. Principi fisici e matematici usati

L'applicazione e' principalmente cinematica, non dinamica. Non modella masse, inerzie o forze di contatto. Usa invece:

1. Cinematica rigida tra frame:

```text
p_B = R_BA p_A + t_BA
v_B = R_BA v_A
```

2. Cinematica di corpo rigido:

```text
v_P = v_O + omega x r
```

3. Cinematica della base differenziale:

```text
x_dot = v cos(theta)
y_dot = v sin(theta)
theta_dot = omega
```

4. Controllo proporzionale del TCP:

```text
v_des = v_feedforward + Kp e
```

5. Minimi quadrati regolarizzati per dividere il moto tra base e lifter:

```text
(J^T W J + R) u = J^T W v_des + R u_ref
```

6. Cinematica differenziale del braccio:

```text
V_tcp = J(q) q_dot
q_dot = J# V_tcp
```

7. Pseudo-inversa smorzata per robustezza vicino alle singolarita':

```text
sigma# = sigma / (sigma^2 + lambda^2)
```

8. Obstacle avoidance di tipo cinematico/potenziale:

```text
h = clamp((D_influence - d) / (D_influence - D_stop), 0, 1)
omega_avoid ~ -sign(y) h^2
```

9. Filtri del primo ordine:

```text
x_f(k+1) = x_f(k) + alpha (x_target - x_f(k))
alpha = dt / (tau + dt)
```

## 17. Lifter

Nel file attuale:

```yaml
lifter:
  enabled: false
```

Quindi il lifter non partecipa al controllo.

Se venisse abilitato, il lifter diventerebbe un grado di liberta' verticale aggiuntivo. Il vettore dei comandi whole-body diventerebbe:

```text
u = [v_x_base, omega_z_base, v_lifter]
```

e il Jacobiano usato dal solver avrebbe una colonna verticale:

```text
J_lifter = [0, 0, 1]^T
```

Il lifter aiuterebbe quindi a generare velocita' TCP lungo Z:

```text
v_tcp_z += v_lifter
```

Il comando non sarebbe una velocita' diretta pubblicata senza limiti: il nodo integra una velocita' limitata dentro un target di posizione:

```text
z_lifter_target(k+1) =
  clamp(z_lifter_target(k) + v_lifter dt,
        min_position,
        max_position)
```

con:

```yaml
min_position: 0.0
max_position: 0.5
max_velocity: 0.015
```

In pratica il lifter servirebbe a scaricare parte del lavoro verticale dal braccio, ma nella configurazione attuale e' solo predisposto.

## 18. Lettura rapida del comportamento

In una frase: il TCP segue un percorso fisso in `print_origin`; la base si muove per mantenere il TCP in una zona comoda rispetto a `base_link`; il braccio corregge l'errore residuo e compensa il moto reale/comandato della base; gli ostacoli non cambiano il path, ma riducono e ruotano il comando `cmd_vel` della base.

Schema finale:

```text
PATH TCP FISSO
     |
     v
TCP desiderato
     |
     +--> base tiene il TCP vicino al punto preferito
     |
     +--> braccio segue il TCP e compensa la base
     |
     +--> ostacoli modificano solo la base:
              rallenta + sterza
```

Questo e' il motivo per cui il sistema riesce a stampare/inseguire il percorso anche con una base mobile: il path resta un riferimento geometrico stabile, mentre base e braccio si coordinano cinematicamente per realizzarlo.
