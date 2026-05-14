# La mia richiesta

Stai lavorando in **ROS Noetic** su un sistema di **teleoperazione manuale HMI** con due master e due slave, dove gli slave sono due **UR10e**. Il controllo è cartesiano, quindi non vuoi concentrarti sulla cinematica interna dei robot, ma sulla logica di accoppiamento tra i TCP.

Attualmente i due master vengono mossi manualmente tramite un controllo in ammettenza. I due UR10e replicano i movimenti dei master. Quando i TCP degli slave entrano in contatto con una scatola, il feedback di forza viene riflesso sui master. A quel punto l’operatore può “stringere” i master, e i due slave afferrano la scatola.

Il problema che vuoi risolvere nasce **dopo l’afferraggio**: vuoi accoppiare i due slave come se tra i loro TCP esistesse un **link virtuale rigido**, che rappresenta la scatola. Quindi vuoi mantenere costante il vincolo geometrico tra i due TCP slave durante il movimento.

Parallelamente, lato master, vorresti calcolare un **frame medio** tra i due TCP dei master. Questo frame medio dovrebbe comandare il movimento del sistema accoppiato lato slave, cioè della media dei due TCP slave vincolati. In altre parole:

```text
media dei TCP master
        ↓
comanda
        ↓
media dei TCP slave / frame virtuale della scatola
```

La tua domanda principale è: **da quale frame conviene partire per definire questa nuova trasformazione/frame virtuale?**

Tu stavi pensando di lavorare nel frame di base di uno dei due master, calcolare la posizione dell’altro frame rispetto a questo riferimento, costruire un frame medio e poi usarlo come riferimento vincolato rispetto al primo robot, applicando successivamente i twist.

Il dubbio è quindi come strutturare correttamente i frame e le trasformazioni per evitare errori, inconsistenze o instabilità, specialmente quando passi dalla teleoperazione indipendente dei due bracci alla modalità accoppiata in cui i due slave devono muoversi come se stessero manipolando rigidamente lo stesso oggetto.


# Una possibile soluzione

Sì, ti sei spiegato abbastanza bene. Il punto chiave è questo: **non partirei dal frame di base di uno dei due master come frame principale del vincolo**, perché rischi di rendere asimmetrico un sistema che fisicamente è simmetrico: due master, due slave, due TCP e un oggetto virtualmente rigido tra i due TCP slave.

Io imposterei il problema così.

## 1. Il frame importante non è il frame di un robot, ma il frame dell’oggetto

Quando i due UR10e afferrano la scatola, tu stai creando virtualmente un corpo rigido. Quindi il riferimento naturale non è:

```text
base_slave_1
```

né:

```text
base_slave_2
```

ma un nuovo frame:

```text
object_frame
```

oppure:

```text
grasp_frame
```

che rappresenta il centro della scatola, o comunque il centro cinematico del vincolo tra i due TCP.

Quindi hai:

```text
slave_1_tcp  ---- rigid link ----  object_frame  ---- rigid link ----  slave_2_tcp
```

Nel momento dell’accoppiamento salvi due trasformazioni costanti:

```text
T_object_tcp1
T_object_tcp2
```

oppure, in forma equivalente:

```text
T_tcp1_object
T_tcp2_object
```

Queste trasformazioni diventano il tuo **vincolo rigido**.

Da quel momento in poi, se muovi `object_frame`, i due TCP slave devono muoversi mantenendo sempre lo stesso offset relativo rispetto all’oggetto.

---

## 2. Non definire il frame medio rispetto al master 1

La tua idea di lavorare nel frame base di uno dei due master è comprensibile, però secondo me è meglio evitarla come riferimento principale.

Il frame del master 1 può essere usato per leggere le pose, ma non dovrebbe diventare il frame “fisico” del vincolo.

Meglio avere un frame comune di controllo, per esempio:

```text
world
```

oppure:

```text
teleop_world
```

oppure:

```text
hmi_base
```

Dentro questo frame comune esprimi:

```text
master_1_tcp
master_2_tcp
slave_1_tcp
slave_2_tcp
object_frame
```

Così puoi ragionare in modo pulito.

La struttura TF ideale sarebbe qualcosa del tipo:

```text
teleop_world
├── master_1_base
│   └── master_1_tcp
├── master_2_base
│   └── master_2_tcp
├── slave_1_base
│   └── slave_1_tcp
├── slave_2_base
│   └── slave_2_tcp
└── object_frame
```

Naturalmente nella realtà `object_frame` lo puoi pubblicare tu come frame virtuale.

---

## 3. Al momento dell’aggancio salvi il vincolo tra i due TCP slave

Quando decidi che la scatola è afferrata, diciamo all’istante `t0`, hai le pose dei due TCP slave:

```text
T_W_S1(t0)
T_W_S2(t0)
```

dove `W` è il tuo frame comune, per esempio `teleop_world`.

A quel punto definisci il frame oggetto. La posizione può essere semplicemente la media:

```text
p_obj = 0.5 * (p_s1 + p_s2)
```

Per l’orientamento hai varie opzioni. La più semplice, spesso sufficiente, è usare l’orientamento di uno dei due TCP:

```text
R_obj = R_s1
```

Però è asimmetrico.

Una scelta migliore è definire l’asse principale dell’oggetto come la direzione tra i due TCP:

```text
x_obj = normalize(p_s2 - p_s1)
```

poi scegli gli altri assi in modo coerente, per esempio usando un asse verticale del mondo o una media degli orientamenti dei TCP.

Quindi ottieni:

```text
T_W_O(t0)
```

dove `O` è `object_frame`.

Poi calcoli e salvi:

```text
T_O_S1 = inverse(T_W_O) * T_W_S1
T_O_S2 = inverse(T_W_O) * T_W_S2
```

Questi due sono costanti finché l’oggetto è afferrato.

Questo è il cuore dell’accoppiamento.

---

## 4. Durante il moto, controlli l’oggetto, non direttamente i due TCP

Dopo l’aggancio, non dovresti più dire:

```text
master_1 muove slave_1
master_2 muove slave_2
```

ma piuttosto:

```text
i due master comandano object_frame
object_frame genera le pose desiderate dei due slave
```

Quindi la logica diventa:

```text
master_1_tcp + master_2_tcp
        ↓
calcolo del frame medio master
        ↓
comando del frame oggetto slave
        ↓
ricostruzione di slave_1_tcp_des e slave_2_tcp_des
```

In formule:

```text
T_W_S1_des = T_W_O_des * T_O_S1
T_W_S2_des = T_W_O_des * T_O_S2
```

Questa è la parte importante: **il vincolo rigido è garantito perché entrambi i TCP derivano dallo stesso frame oggetto**.

Non stai più inseguendo due target indipendenti.

---

## 5. Il frame medio dei master va usato come comando dell’oggetto

Sul lato master fai una cosa analoga.

Hai:

```text
T_W_M1
T_W_M2
```

Calcoli un frame medio master:

```text
T_W_Mavg
```

La posizione può essere:

```text
p_mavg = 0.5 * (p_m1 + p_m2)
```

Per l’orientamento, anche qui puoi scegliere:

* orientamento del master 1;
* media tra quaternioni;
* frame costruito sulla linea tra i due master;
* orientamento di un master dominante;
* orientamento filtrato dell’oggetto.

La scelta dipende da cosa vuoi controllare: solo traslazione dell’oggetto o anche rotazione?

Per iniziare io farei una cosa semplice:

```text
p_mavg = media delle posizioni dei due master
R_mavg = orientamento del master dominante oppure orientamento iniziale + incrementi
```

Poi lavori per incrementi, non in assoluto.

---

## 6. Usa trasformazioni relative, non pose assolute

Questo è un altro punto molto importante.

Quando avviene il coupling, salva:

```text
T_W_Mavg(t0)
T_W_O(t0)
```

Poi durante il moto calcoli il delta del master medio:

```text
Delta_M = inverse(T_W_Mavg(t0)) * T_W_Mavg(t)
```

e lo applichi all’oggetto slave:

```text
T_W_O_des(t) = T_W_O(t0) * scaled(Delta_M)
```

Questa è molto più robusta rispetto a dire “la media dei master deve essere uguale assolutamente alla media degli slave”.

Perché? Perché master e slave possono vivere in spazi diversi, con scale diverse, offset diversi e orientamenti diversi.

Quindi la mappatura migliore è:

```text
variazione del frame medio master
        ↓
variazione del frame oggetto slave
```

non:

```text
posa assoluta master medio = posa assoluta oggetto slave
```

Concettualmente:

```text
object_pose_des = object_pose_at_grasp + motion_increment_from_master_average
```

---

## 7. Schema completo consigliato

Io implementerei così.

### Prima del contatto

Teleoperazione indipendente:

```text
master_1 → slave_1
master_2 → slave_2
```

Ogni master comanda il proprio UR10e in cartesiano/admittance.

---

### Contatto con la scatola

Quando rilevi contatto/forza:

```text
slave_1_contact = true
slave_2_contact = true
```

ma ancora non sei necessariamente accoppiato.

---

### Grasp confermato

Quando decidi che la scatola è afferrata:

```text
coupled_mode = true
```

allora salvi:

```text
T_W_S1_0
T_W_S2_0
T_W_M1_0
T_W_M2_0
```

calcoli:

```text
T_W_O_0
T_W_Mavg_0
T_O_S1
T_O_S2
```

---

### Dopo il coupling

A ogni ciclo:

```text
leggi T_W_M1
leggi T_W_M2

calcola T_W_Mavg

calcola Delta_M = inverse(T_W_Mavg_0) * T_W_Mavg

calcola T_W_O_des = T_W_O_0 * Delta_M

calcola T_W_S1_des = T_W_O_des * T_O_S1
calcola T_W_S2_des = T_W_O_des * T_O_S2

manda T_W_S1_des al controller cartesiano di slave 1
manda T_W_S2_des al controller cartesiano di slave 2
```

Questo mantiene il vincolo.

---

## 8. Il vincolo tra i due slave non va mantenuto con una “media” ogni volta

Attenzione: una possibile trappola è questa.

Potresti pensare:

```text
calcolo sempre la media tra slave_1_tcp e slave_2_tcp
poi correggo i due TCP
```

Questo può funzionare come diagnostica, ma non lo userei come struttura principale di comando.

Meglio:

```text
un solo frame oggetto comanda entrambi i TCP
```

Perché così il vincolo è cinematicalmente imposto dalla generazione delle pose desiderate.

La distanza tra i due TCP rimane automaticamente:

```text
T_S1_S2 = constant
```

mentre se comandi i due TCP indipendentemente e poi “aggiusti” la media, rischi drift, conflitti di controllo e instabilità.

---

## 9. Dove applicare i twist?

Se lavori a twist invece che a pose, la logica equivalente è questa:

1. calcoli il twist medio dei master;
2. lo interpreti come twist dell’oggetto;
3. da quel twist dell’oggetto ricavi i twist desiderati dei due TCP slave.

Se l’oggetto ha twist:

```text
V_O = [v_O, ω_O]
```

allora il TCP slave 1, che si trova a offset `r_1` rispetto all’oggetto, deve avere velocità lineare:

```text
v_S1 = v_O + ω_O × r_1
```

e velocità angolare:

```text
ω_S1 = ω_O
```

Analogamente:

```text
v_S2 = v_O + ω_O × r_2
ω_S2 = ω_O
```

Questa è la formulazione più pulita se il tuo controller lavora in velocità cartesiana.

Quindi:

```text
master average twist
        ↓
object twist
        ↓
slave TCP twists compatibili con corpo rigido
```

Non mandare lo stesso twist lineare puro a entrambi se vuoi anche rotazioni dell’oggetto, perché per una rotazione rigida i due TCP devono avere velocità lineari diverse.

---

## 10. Frame consigliato per esprimere i twist

Per evitare casino, scegli una convenzione e mantienila ovunque.

Io suggerirei:

```text
tutti i twist espressi in teleop_world
```

Quindi:

```text
V_O^W
V_S1^W
V_S2^W
```

dove `W` è il frame comune.

Allora la formula è semplice:

```text
v_Si^W = v_O^W + ω_O^W × r_i^W
ω_Si^W = ω_O^W
```

con:

```text
r_i^W = p_Si^W - p_O^W
```

Se invece esprimi i twist nei frame locali TCP, devi usare adjoint transformations, e la probabilità di errore aumenta parecchio.

Per una prima implementazione: **tutto in world/base comune**.

---

## 11. Risposta diretta alla tua domanda

> Da quale frame parto per definire il nuovo frame?

Io partirei da un frame comune esterno, non dalla base di uno dei due master.

Idealmente:

```text
teleop_world
```

Dentro `teleop_world` definisci:

```text
master_average_frame
slave_object_frame
```

Il frame nuovo lato slave è:

```text
object_frame = frame virtuale della scatola
```

calcolato inizialmente dai due TCP slave al momento del grasp.

Il frame nuovo lato master è:

```text
master_average_frame = frame medio dei due master
```

calcolato dai due TCP master.

Poi non imponi che siano lo stesso frame assoluto, ma imponi che il moto relativo del master average muova il frame oggetto slave.

Quindi:

```text
Delta master_average → Delta object_frame
```

Questa secondo me è la scelta più pulita.

---

## 12. Architettura ROS pratica

In ROS Noetic potresti avere un nodo tipo:

```text
dual_arm_coupling_node
```

che fa:

### Sottoscrive

```text
/master_1/tcp_pose
/master_2/tcp_pose
/slave_1/tcp_pose
/slave_2/tcp_pose
/slave_1/wrench
/slave_2/wrench
/coupling_enable
```

### Pubblica

```text
/slave_1/cartesian_pose_command
/slave_2/cartesian_pose_command
/tf/object_frame
/tf/master_average_frame
```

oppure, se lavori in velocità:

```text
/slave_1/cartesian_twist_command
/slave_2/cartesian_twist_command
```

### Stato interno

```text
mode = INDEPENDENT | CONTACT | COUPLED
```

Nel passaggio a `COUPLED`, salvi tutte le trasformazioni iniziali.

---

## 13. Pseudocodice

```python
if not coupled:
    slave1_cmd = map_master_to_slave(master1)
    slave2_cmd = map_master_to_slave(master2)

else:
    T_W_M1 = get_pose("master_1_tcp")
    T_W_M2 = get_pose("master_2_tcp")

    T_W_Mavg = compute_average_frame(T_W_M1, T_W_M2)

    Delta_M = inverse(T_W_Mavg_0) @ T_W_Mavg

    T_W_O_des = T_W_O_0 @ Delta_M

    T_W_S1_des = T_W_O_des @ T_O_S1
    T_W_S2_des = T_W_O_des @ T_O_S2

    publish_slave1_command(T_W_S1_des)
    publish_slave2_command(T_W_S2_des)
```

Se lavori a twist:

```python
if coupled:
    V_Mavg = compute_average_twist(master1_twist, master2_twist)

    V_O = scale_twist(V_Mavg)

    r1 = p_slave1_des - p_object_des
    r2 = p_slave2_des - p_object_des

    v1 = v_O + cross(w_O, r1)
    v2 = v_O + cross(w_O, r2)

    w1 = w_O
    w2 = w_O

    publish_twist_slave1(v1, w1)
    publish_twist_slave2(v2, w2)
```

---

## 14. Cosa farei io per una prima versione stabile

Per non fare casini, partirei semplice:

1. **accoppiamento solo traslazionale**;
2. distanza tra i TCP slave congelata al momento del grasp;
3. `object_frame` in mezzo ai due TCP slave;
4. orientamento oggetto inizialmente uguale a quello di `slave_1_tcp` oppure fisso rispetto al mondo;
5. comando dell’oggetto tramite spostamento medio dei master;
6. niente rotazioni dell’oggetto all’inizio;
7. poi aggiungi rotazioni quando la parte traslazionale è stabile.

Quindi nella prima versione:

```text
p_object_des = p_object_0 + scale * (p_master_avg - p_master_avg_0)
```

e:

```text
R_object_des = R_object_0
```

Poi:

```text
T_W_S1_des = T_W_O_des * T_O_S1
T_W_S2_des = T_W_O_des * T_O_S2
```

Questo ti permette di testare subito il vincolo senza complicarti con orientamenti, quaternioni medi, asse tra TCP, rotazioni rigide ecc.

---

La frase chiave è:

**non accoppiare direttamente i due slave tra loro a ogni ciclo; crea un frame oggetto virtuale al momento del grasp, salva gli offset dei due TCP rispetto a quel frame, e poi comanda solo quel frame oggetto usando il moto relativo del frame medio dei master.**


# Note aggiuntive per spiegare lo stato attuale 

Al momento comandi e pose vengono inviati nel frame base dei rispettivi robot base_link_inertia, che è anche il frame in cui va inviato il twist di velocità, come succede ora.
Al momento eseguo un override dei frame tra master e slave, perché per quanto mi riguarda è come se coincidessero, tranne che per una piccola rotazione che viene calcolata e tenuta in considerazione.
Il twist che è applicato al tool0 dei rispettivi slave deve essere espresso nel frame base_link_inertia.
Se possibile questa cosa la terrei perchè funziona.

# Mie considerazioni

A me piace l'idea di creare un frame virtuale che sia la media tra i due frame slave per il lato slave e tra i due frame master per il lato master e poi il frame master controlla il frame slave, eventualmente mantenendo l'eventuale scalatura del moto presente tra master e slave. 
Bisogna decidere qual è il frame di base in cui viene espresso questo nuovo frame virtuale. Tra i master e gli slave non esiste una trasformazione comune, infatti faccio l'override dei frame perché per quanto mi riguarda è come se coincidessero. Caso diverso invece è per i due slave o i due master: Esiste una trasformazione che collega i frame base_link_inertia dei due robot slave, come ne esiste una che collega i due frame base_link_inertia dei due master. A questo punto io potrei esprimere il frame virtuale nel frame base_link_inertia di uno dei due master e poi fare l'override e applicare la rotazione per pubblicare lo stesso frame nel base_link_inertia dello slave corrispondente. Cosa ne pensi?
In questo modo i due master mappano un frame nel loro spazio di lavoro e in questo modo posso inviare la posa e il twist di velocità allo slave come già faccio, ma verso un frame virtuale. Da tale frame si possono ricavare i due comandi singoli che gli slave devono eseguire.


La condizione di accoppiamento la vorrei abilitare tramite una chiamata a un servizio e la vorrei disabilitare sempre tramite una chiamata a un servizio.
per quanto riguarda il punto neutro in caso di scalatura ne possiamo discutere di come gestirlo.

Altro punto da tenere in considerazione è il feedback di forza. Nel momento in cui sono accoppiato vorrei che il feedback non ci fosse, ma che non sparisse completamente all'improvviso, ma in modo graduale, in modo che l'operatore possa percepirlo e regolare la sua forza di conseguenza. Quello che vorrei tenere invece è il valore di massa e smorzamento dinamico che dipende dal carico applicato alle celle di carico dei robot slave.