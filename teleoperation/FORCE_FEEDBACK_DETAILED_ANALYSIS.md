# Analisi Dettagliata del Sistema di Controllo e Force Feedback

In base all'analisi dei nodi sorgente (`teleop_master_haptic_controller_node.cpp` e `teleop_slave_twist_outer_loop_node.cpp`) e dei relativi file YAML presenti all'interno del tuo pacchetto, ecco nel dettaglio come funziona l'attuale implementazione del tuo sistema bilaterale di teleoperazione.

L'architettura presente è un sistema ibrido bilateriale che mescola un **Controllo ad Ammettenza (Master)** con un accoppiamento basato su **Molla Virtuale ed Errore di Posizione** e un parametro di **Riflessione Diretta della Forza**.

Vedremo subito perché, allo stato attuale (con i parametri che hai settati nel file launch e nei YAML), **non riesci a percepire l'avvenuto contatto dello slave**.

---

## 1. Modello Matematico del MASTER (Admittance/Haptic Control)

Sul lato del braccio comandato dall'operatore (Master), il sistema genera una cinematica virtuale comportandosi come un sistema Massa-Smorzatore tirato ed ostacolato da varie forze.

L'equazione differenziale che governa l'accelerazione lineare comandata al Master $\dot{v}_m$ è la seguente:

$$ M_m \cdot \dot{v}_m + D_m \cdot v_m = F_{hand} - F_{feedback} $$

Dove:
*   $M_m$ è la matrice delle masse/inerzie virtuali (nel tuo config: `mass_linear = 3.0` kg).
*   $D_m$ è lo smorzamento virtuale che previene instabilità e fughe in avanti (`damping_linear = 10.0` Ns/m).
*   $F_{hand}$ è la forza vera applicata dalla mano dell'operatore letta dal sensore, filtrata con passa-basso e a cui viene sottratta una deadband di `2.0` N (`force_deadband_enter`). Sotto i 2 Newton la forza è azzerata.
*   **$F_{feedback}$ è il cuore scatenante del problema**. Questa grandezza si compone di base su due rami:

$$ F_{feedback} = \underbrace{K_{master\_spring}(p_{master} - p_{slave}) + B_{master\_spring}(v_{master} - v_{slave})}_{\text{Molla Virtuale di Accoppiamento (Position-based)}} - k_{force\_refl} \cdot F_{slave\_ext} $$

Dai tuoi file di configurazione (`master_real_mur620b_ur10_l.yaml`) notiamo questi pesi critici:
*   `spring_stiffness_linear`: **2000.0** N/m ($K_{master\_spring}$)
*   `force_reflection_scale` ($k_{force\_refl}$): **0.03**

---

## 2. Modello Matematico dello SLAVE (Compliance e Inseguimento)

Il braccio remoto che esegue il compito (Slave) riceve la posa asintotica del Master, ed esegue un calcolo indipendente in anello chiuso:

$$ v_{cmd,s} = k_{ff} v_m + K_{slave\_spring} (p_m - p_s) + K_{slave\_int} \int (p_m - p_s) dt + k_{adm,s} \cdot F_{slave\_ext} $$

Guardando i parametri di (`slave_real_mur620b_ur10_r.yaml`):
*   Il controllo in uso è lo *Spring Control* (`use_spring_control: true`).
*   $K_{slave\_spring}$ è attiva (`spring_stiffness_linear = 2.0`).
*   $k_{ff}$ è a regime (`k_ff = 1.0`).
*   Il termine di compliance **$k_{adm,s}$ è spento** (`k_adm_linear = 0.0`).

---

## 3. L'Acquisizione e il Filtraggio della Cella di Carico (Load Cell)

Sia il Master che lo Slave leggono i valori misurati dai sensori di forza/coppia (celle di carico) tramite il topic ROS strutturato su messaggi di tipo `geometry_msgs/WrenchStamped`. Queste letture, prima di entrare nelle equazioni sopra descritte, subiscono una pipeline di elaborazione che ne garantisce l'usabilità nel sistema di controllo.

### Fase A: Trasformazione di Sistema di Riferimento (TF)
Le celle di carico misurano generalmente le forze nel sistema di riferimento del TCP (Tool Center Point), ovvero la flangia finale a cui sono attaccate. Tuttavia, la dinamica calcolata dal robot è solitamente espressa rispetto alla base del manipolatore.
Nel codice (sia in `teleop_master_haptic_controller_node.cpp` tramite `wrenchMsgToWrench3()` che nel callback dello slave locale `wrenchCb()`) viene effettuata una **rotazione del vettore** della forza da `wrench_source_frame_override` (solitamente `tool0`) a `wrench_target_frame` (solitamente `base_link_inertia`). Questo permette alle forze lette dalla cella (ad esempio "spinta lungo la Z del TCP") di essere ruotate ed espresse coerentemente rispetto al braccio base per sottrarle/sommarle correttamente alle direzioni cartesiane globali.

### Fase B: Timeout ("Staleness")
Prima di autorizzare la lettura della forza nei calcoli, il nodo interroga il timestamp del pacchetto: se l'informazione del sensore non viene aggiornata ad almeno un certo timeout di sicurezza (nel tuo YAML `wrench_timeout_s: 0.2` sec), **il sistema setta un errore di "Stale wrench"** e le velocità comandate vengono forzate a zero per prevenire che l'algoritmo calcoli un'accelerazione su un dato obsoleto o bloccato, comportando pericoli notevoli.

### Fase C: Pipeline di Elaborazione Indipendente dei Segnali (Filtri)
I segnali, dopo il controllo di timeout, attraversano un blocco di filtraggio e saturazione (tramite la funzione unificata `teleoperation::filterClampDeadbandWrenchNorm()`). **Rispondendo alle tue deduzioni, ti confermo che è esattamente come hai intuito: esistono tre pipeline di filtraggio separate con parametri dedicati:**

1. **Il Nodo Master e la sua Cella locale (Master Wrench)**:
   Il nodo Master legge **direttamente** la sua cella di carico locale e la filtra. Questo filtro passa-basso sfrutta un set di parametri yaml specifici: `master_wrench_filter_cutoff_hz` e `master_wrench_filter_alpha` (se non presenti, usa il fallback generico `wrench_filter_cutoff_hz` del master). L'obiettivo qui è eliminare il rumore introdotto dalla mano dell'operatore.
2. **Il Nodo Master e la Cella dello Slave remota (Feedback Wrench)**:
   Il nodo Master **legge direttamente anche il topic della cella di carico dello Slave** via rete. Non usa i dati già processati dallo slave, ma prende il segnale grezzo dello slave e **lo filtra lui stesso internamente** prima di rifletterne la forza. Per farlo, applica dinamicamente un altro set di parametri hardware distinti che trovi nel file del Master: `feedback_wrench_filter_cutoff_hz` e `feedback_wrench_filter_alpha`.
3. **Il Nodo Slave e la sua Cella (Slave Local Wrench per Compliance)**:
   Il nodo Slave, in totale autonomia (in un loop asincrono), legge **il proprio sensore di forza** dal bus e applica **il suo proprio filtro passa-basso** prima di alimentare l'equazione di cedevolezza meccanica locale (ammettenza dello slave). I parametri di questo terzo filtro si trovano nel config YAML dello slave: `wrench_filter_cutoff_hz` e `wrench_filter_alpha`.

Tutti e tre questi filtri indipendenti calcolano un coefficiente dinamico $\alpha$ dipendente dal proprio step temporale ($dt$): $F_{filtrata}(t) = (1 - \alpha) F_{filtrata}(t-1) + \alpha F_{letta}(t)$.

### Fase D: Deadband con Isteresi (Hysteresis Soft-Deadband)
Applicata separatamente a valle di ciascun filtro, questa fase serve a tagliare fuori (annullare a zero) le spinte involontarie residue (deriva e peso dell'end effector) calcolando un "Thresholding morbido" applicato **alla norma** euclidea del vettore ($|F|$). 
- Se entri in uno stato senza contatto, ignora forze sotto il valore "Enter" (`force_deadband_enter = 2.0` N per il master).
- Solo quando si superano i 2 N, la forza viene accettata. Ed una volta attivata, rimarrà presente anche scendendo fino al limite "Exit" (es. `1.0` N) evitando l'effetto *chattering* (vibrazione on-off).

### Fase E: Clamping di Saturazione di Sicurezza
A completamento delle rispettive pipeline, alla norma del vettore di forza totale in uscita viene forzato un limite massimo dinamico indipendente con `teleoperation::clampNorm3()`. Il master taglia forzatamente i valori limitandoli entro `max_force = 60.0` N per la sua mano, mentre lo slave fa un clamp indipendente al valore `120.0` N per proteggere i circuiti algoritmici.

---

## 4. L'Analisi del Problema: Perché non "senti" il contatto dello Slave?

Ci sono simultaneamente **due criticità di parametrizzazione** che distruggono il feeling tattile (force feedback) dal master, aggravati dalla combinazione di cui sopra (sezione di acquisizione):

### Causa 1: La Riflessione Diretta della Forza è stata spenta (3%)
Il modo più naturale affinché un uomo percepisca l'urto tra lo slave e un tubo di alluminio o un banco, è riportare l'onda dell'impatto sul master proporzionalmente.
Attualmente nel master hai:
`force_reflection_scale = 0.03`
Significa che l'equazione del master tiene conto **solo del 3%** della forza d'urto reale prodotta all'end-effector dello slave. Se lo slave urta un ostacolo esprimendo $30$ N di violenza (o la subisce dall'ambiente), il master subisce un contrasto al moto di ben $30 \cdot 0.03 = 0.9$ N ($F_{slave\_ext}$).
Questo piccolo delta viene ricevuto, ma cosa gli succede? Abbiamo visto nella sezione 3 che c'è la **Deadband**. 
Siccome il Master ha una deadband in uscita-ingresso della mano pari a `2.0` N (`force_deadband_enter`), l'impatto passivo originato dallo slave da $0.9$ Newton non fa in tempo a smuovere dinamicamente gran parte delle resistenze virtuali impostate ($M=3.0$, $D=10.0$) superando matematicamente i pesi contrastanti. **Non la senti affatto.**

### Causa 2: Lo Slave è rigido e il Master aspetta che sposti l'aria (Molla Spugnosa)
Mancando la riflessione netta, l'intero compito di avvisarti dell'urto grava sul termine di posizione $F_{spring}$ ($K=2000$). La teoria dice: *se lo slave si ferma urlando contro il muro, ma l'operatore continua a spingere con la sua mano in avanti un po' di millimetri, lo scarto di coordinate $p_r - p_s$ cresce al punto che i 2000 N/m della costante producono tanti Newton contro il master.*

**Cosa accade nella realtà:**
1. Poiché lo **SLAVE è programmato in modalità rigidissima** (avendo azzerato a $0.0$ la compliance `k_adm_linear`), quando tocca un muro **non cede**. Si affida alla tolleranza del posizionatore in giunto UR10e inferiore tentando attivamente di sfondare l'urto per annullare $(p_m - p_s)$. 
2. Sul lato Master: non senti mai l'urto "secco", perché la forza sale proporzionalmente all'errore di posizione. E' l'effetto "molla nel burro": senti una spugnosità ritardata che cresce linearmente e se perdi di colpo la presa (mollando la maniglia manuale del master), il joystick robotico partirà fortissimo tirato contro-indietro verso lo slave causando un feeling brutto e incontrollato per un operatore teleoperante.

---

## 5. La Soluzione Implementativa: Come ottenere il "Click" e il feeling di rigidità

Esistono di rito due Architetture perfette in Master-Slave. Date le tue formule già scritte nel sorgente C++, disponi di un'**Architettura FP Modificata** (Force-Position), che possiamo accendere intervenendo solo nei file YAML.

Per sentire bene l'ambiente dovrai usare la forza dello Slave non mediata (o al massimo filtrata via low-pass) e ridurre la dipendenza quasi insensibile dei tracking via "molla di posizione".

### **Piano di Modifica dei Parametri (Step-by-Step):**

**Nel file del Master (`master_real_mur620b_ur10_l.yaml`):**
1. Alza massivamente il termine di riflessione diretta, portalo quasi al rapporto 1:1 con l'umano se riesci a dominare le instabilità, e disabbassa pesantemente la molla compensatrice:
   ```yaml
   # Porta pesantemente su il bilancio alla forza diretta:
   force_reflection_scale: 0.6    # o 0.8 per sentire l'80% vivo del contatto
   torque_reflection_scale: 0.6   # (se attivo)
   
   # Spengi o depotenzia vertiginosamente la molla per evitare oscillazioni a pendolo asincrone! 
   # Diventerà solo una molle di 'ricentramento rimosso al riposo' contro derive.
   spring_stiffness_linear: 100.0  # e NON più 2000! 
   spring_damping_linear: 2.0
   ```
2. Abbassa leggermente la deadband del Master per farlo diventare più fine, per esempio portala da `2.0` a `1.2`. Costerà far scattare un po' il braccio sui touch, ma aumenta sensibilità (questo farà passare le micro forze di riflessione calcolate e le forze leggere della mano):
   ```yaml
   force_deadband_enter: 1.2
   force_deadband_exit: 0.8
   ```

**Nel file dello Slave (`slave_real_mur620b_ur10_r.yaml`):**
3. Poiché il Master ora "sente" direttamente lo Slave in loop puro, **è obbligatorio accendere l'ammettenza dello slave**. Se non cede all'ostacolo, il feedback di forza salterà in loop e ti strapperà il braccio master generando risonanza. Lo slave deve cedere leggermente l'inerzia quando assorbe la botta per dar tempo al master di fermare l'inseguimento del riferimento target.
   ```yaml
   k_adm_linear: 0.005  # Questo indica che su 10 N di contatto, lo slave mollerà rallentando la sua velocità di marcia di 50 mm/s a favore di compensazione meccanica.
   ```

### Risultato atteso:
L'istante in cui lo slave collide un ostacolo, la sua cella di carico subisce l'impatto ma lo slave (grazie ad una lieve ammettenza) ammortizza fermando morbidamente la traslazione dei giunti per non entrare in protezione. Parte del segnale di forza grezzo misurato al TCP, viene ruotato, filtrato dal filtro IIR passa-basso (che pulisce il segnale d'urto ad alta frequenza tenendo la curva "calma") e sottratto ad arte alla Deadband del Master per farti sentire nel manovratore i Newton precisi scalati che l'oggetta rimbalza sul robot. Questo causerà un'improvvisa forza contro-resistente sulla tua mano impedendoti di proseguire, ricreando un vero e tangibile sense-of-touch.
