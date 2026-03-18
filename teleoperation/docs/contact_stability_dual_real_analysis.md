# Analisi della stabilita durante il contatto

## Contesto

Questa analisi e basata sul setup attuale definito in:

- `launch/master_slave_mur620b_mur620d_dual_real.launch`
- `config/master_real_mur620b_ur10_l.yaml`
- `config/slave_real_mur620b_ur10_r.yaml`
- `src/teleop_master_haptic_controller_node.cpp`
- `src/teleop_slave_twist_outer_loop_node.cpp`
- `src/teleop_bimanual_coupling_node.cpp`

Il problema descritto e coerente con una situazione in cui:

- il contatto sullo slave genera una riflessione di forza sul master;
- il loop diventa poco dissipativo quando l'oggetto e rigido;
- in presa bimanuale due loop bilaterali indipendenti vengono accoppiati dallo stesso oggetto.

## Lettura del setup attuale

### Master

Ogni `teleop_master_haptic_controller` fa tre cose importanti:

1. legge il wrench locale del master;
2. legge il wrench dello slave e lo riflette nel master;
3. aggiunge una molla virtuale basata sulla posa reale dello slave.

In forma compatta, dal codice:

```text
F_feedback = F_spring - kf * F_slave + F_coupling
M * dv/dt + D * v = F_hand - F_feedback
```

Osservazioni rilevanti nel tuo YAML:

- `force_reflection_scale: 0.40`
- `spring_stiffness_linear: 10.0`
- `spring_damping_linear: 0.0`
- `dynamic_damping.enabled: true`
- `dynamic_mass.enabled: true`

### Slave

Ogni `teleop_slave_twist_outer_loop` e in `use_spring_control: true`, quindi usa:

```text
v_cmd = k_ff * v_ff + K_s * e + Ki * integral(e) + v_comp
```

Nel tuo YAML, pero:

- `k_adm_linear: 0.00`
- `force_limiter.enabled: false`
- `torque_limiter.enabled: false`
- `hard_guard_action: retreat`, ma solo oltre `150 N`

Quindi, in pratica, lo slave non ha una vera cedevolezza di contatto nel range normale di lavoro: continua a inseguire il target finche non interviene la protezione dura.

### Aspetto bimanuale

Nel `launch` hai due coppie master/slave indipendenti, una per il braccio sinistro e una per il destro. In piu:

- `coupling_wrench_topic` e vuoto su entrambi i master;
- il nodo `teleop_bimanual_coupling` esiste nel package, ma non viene lanciato.

Questo significa che, quando afferri un solo oggetto con due bracci, il coordinamento tra i due lati non e esplicito: avviene solo attraverso la rigidezza dell'oggetto e le due catene di controllo.

## Cause probabili dell'instabilita

### 1. Riflessione diretta del wrench slave sul master

Questa e la causa piu probabile del "rimbalzo" che senti.

Quando lo slave tocca un oggetto rigido, il wrench misurato viene rimandato al master come termine di feedback. Questo canale ha inevitabilmente:

- ritardo di misura;
- filtraggio;
- ritardo TF;
- dinamica del controller twist;
- ritardo meccanico del robot.

Un loop forza-forza con ritardo e facilmente non passivo: invece di dissipare energia, puo iniettarla a certe frequenze. Il risultato tipico e proprio un master che vibra o "rimbalza" al contatto.

Nel tuo caso il rischio cresce perche:

- la riflessione e attiva su entrambi i bracci;
- `force_reflection_scale` e relativamente alto;
- il feedback di forza si somma alla molla virtuale.

Nota importante: nello stesso YAML del master e scritto che, con la molla virtuale attiva, il contributo FT dovrebbe restare basso, indicativamente `0.01-0.05`. Il valore attuale `0.40` e molto piu aggressivo.

### 2. Molla virtuale senza smorzamento relativo

Hai una molla virtuale attiva (`spring_stiffness_linear: 10.0`), ma il termine dissipativo dedicato alla differenza di velocita e spento (`spring_damping_linear: 0.0`).

Questo significa che il master sente la divergenza di posa rispetto allo slave, ma non hai un termine esplicito che smorzi il moto relativo master/slave nel canale della molla.

Con un contatto rigido il comportamento tipico e:

- il master avanza;
- lo slave si ferma o rallenta contro l'oggetto;
- l'errore di posa cresce;
- la molla restituisce forza;
- senza damping relativo, la correzione tende a essere piu oscillatoria.

### 3. Lo slave e troppo rigido al contatto

Questo punto e fondamentale.

Nel setup attuale:

- `k_adm_linear` e zero;
- il `force_limiter` e disabilitato;
- il `hard_guard` entra solo a soglie molto alte.

Quindi lo slave, quando tocca l'oggetto, non si "ammorbidisce": continua a inseguire il target tramite feedforward + spring + integrale. Il risultato e che l'energia viene accumulata sul contatto e poi scaricata nel loop bilaterale.

In presa con due bracci questo peggiora ancora, perche entrambi gli slave continuano a stringere o a inseguire la posa imposta, e l'oggetto diventa il solo elemento che assorbe la differenza.

### 4. Due loop bilaterali indipendenti accoppiati dallo stesso oggetto

Questo spiega bene perche il problema peggiora quando afferri con entrambi i bracci.

Se i due lati non sono coordinati esplicitamente, basta che:

- un braccio tocchi leggermente prima;
- uno abbia piu attrito;
- una FT abbia piu rumore o bias;
- una catena abbia piu ritardo.

Da li nasce facilmente questo meccanismo:

1. un lato entra in contatto e inizia a riflettere forza;
2. l'altro lato continua a muoversi;
3. l'oggetto ruota o si precarica;
4. il secondo lato entra a sua volta in contatto;
5. i due loop si eccitano attraverso l'oggetto.

Il fenomeno viene percepito come instabilita diffusa, presa nervosa, rimbalzo del master e peggioramento netto nel grasp bimanuale.

### 5. Rischio concreto nella gestione dei frame del wrench

Nel master esiste un solo parametro `wrench_source_frame_override`, e il codice lo usa per tutti i wrench in ingresso: master, slave e coupling.

Nel tuo `launch`, questo override viene impostato al tool frame del master. Quindi il wrench dello slave puo essere ruotato come se provenisse dal tool del master, non da quello reale dello slave.

Questo e un punto molto delicato: se i due tool non sono perfettamente allineati proprio nel momento del contatto, la direzione del feedback puo risultare errata o poco dissipativa.

In free space il problema puo restare poco visibile; al contatto, invece, puo diventare determinante.

### 6. Parametri identici sui due lati, ma bracci reali non perfettamente identici

Nel `launch` entrambi i master caricano lo stesso YAML, e anche gli slave usano lo stesso YAML. Questo e comodo, ma nel dual-arm reale ci sono sempre piccole asimmetrie:

- tare FT non identica;
- attriti diversi;
- differenze di cablaggio o timing;
- mounting o compliance meccanica diversi.

Se i parametri sono troppo aggressivi, anche piccole asimmetrie vengono amplificate e si manifestano soprattutto nel grasp a due bracci.

## Soluzioni possibili

## A. Soluzioni ad alto impatto senza cambiare codice

### A1. Ridurre drasticamente o disattivare la riflessione diretta della forza

Questa e la prima prova da fare.

Obiettivo:

- capire se il canale `slave_wrench -> master` e il responsabile principale;
- far lavorare il feedback principalmente tramite la molla virtuale.

Possibili azioni:

- portare `force_reflection_scale` a `0.0` per un test pulito;
- se il comportamento migliora molto, reintrodurlo solo in piccola misura;
- come ordine di grandezza, restare vicino al range gia suggerito nel file: `0.01-0.05`.

Vantaggi:

- intervento semplice;
- isola subito la causa;
- spesso elimina gran parte del rimbalzo.

Svantaggi:

- si perde parte della "texture" del contatto;
- il feedback diventa piu posizionale che puramente forza-forza.

### A2. Dare smorzamento alla molla virtuale

Nel tuo setup la molla lineare c'e, ma `spring_damping_linear` e zero. Vale la pena introdurre un damping relativo positivo.

Effetto atteso:

- meno oscillazione della differenza master/slave;
- transizione di contatto piu viscosa;
- minor tendenza al bounce.

Indicazione pratica:

- tenere `spring_stiffness_linear` bassa o moderata;
- introdurre `spring_damping_linear` gradualmente;
- se il sistema resta nervoso, ridurre prima la rigidita e poi alzare lo smorzamento.

### A3. Usare il damping dinamico come stabilizzatore principale e la massa dinamica con cautela

Nel master hai gia:

- `dynamic_damping`
- `dynamic_mass`

Il damping aiuta a dissipare energia. La massa invece rallenta e rende il master piu "pesante", ma non dissipa direttamente.

Per il problema che descrivi, in genere conviene:

- privilegiare il damping dinamico;
- usare la massa dinamica con moderazione;
- se il feeling diventa troppo "inerziale" o rimbalzante in rilascio, ridurre `dynamic_mass` o disabilitarla durante il tuning iniziale.

In altre parole: per il contatto la leva migliore e quasi sempre il damping, non la massa.

### A4. Abbassare la banda del feedback di forza, se il rumore e ad alta frequenza

I cutoff a `12 Hz` non sono estremi, ma se il contatto eccita vibrazioni meccaniche conviene comunque valutare un filtraggio piu conservativo sul wrench riflesso.

Questa soluzione da sola raramente basta, ma puo aiutare se:

- il rumore FT e elevato;
- il contatto genera picchi rapidi;
- il problema e piu vibratorio che quasi-statico.

## B. Rendere lo slave piu cedevole al contatto

Questa e probabilmente la seconda famiglia di interventi piu importante dopo la riduzione del feedback diretto.

### B1. Abilitare il `force_limiter`

Il codice dello slave ha gia una logica utile:

- stima la componente di forza resistiva lungo la direzione di moto;
- puo ridurre solo la componente che spinge dentro il contatto;
- puo entrare in modalita `retreat` se la forza supera una soglia.

Questa e molto adatta al tuo caso, perche:

- lascia piu liberi gli scorrimenti tangenziali;
- impedisce di continuare a spingere "contro" l'oggetto;
- riduce l'energia immessa nel loop bilaterale.

Per un primo tuning, il comportamento piu naturale e spesso:

- `metric: parallel_resistive`
- `mode: scale_parallel`

Se il contatto e particolarmente duro, si puo valutare anche `retreat`.

### B2. Introdurre una piccola compliance reale sullo slave

Oggi `k_adm_linear` e nullo. Un piccolo valore positivo puo aiutare lo slave a cedere leggermente quando la forza cresce, invece di insistere sulla traiettoria imposta.

Da usare con prudenza:

- partire molto bassi;
- limitare sempre la velocita della componente compliant;
- evitare di far diventare lo slave "molle" in free space.

Questa soluzione e molto utile se vuoi che il contatto venga assorbito a monte, invece di lasciare tutto il lavoro al master.

### B3. Ridurre il contributo integrale e la rigidita lato slave

Nel controllo attuale, durante il contatto l'integrale si congela oltre una soglia, ma il termine proporzionale della molla lato slave continua a spingere.

Se il grasp bimanuale resta nervoso, ha senso valutare:

- ridurre `spring_ki_linear`;
- ridurre `spring_stiffness_linear` lato slave;
- eventualmente introdurre uno scarico o una dissipazione aggiuntiva del termine integrale quando il contatto persiste.

Questa ultima parte richiede codice, ma spesso basta gia alleggerire `K` e `Ki`.

## C. Soluzioni specifiche per la presa con due bracci

### C1. Attivare un coordinamento bimanuale esplicito

Nel package hai gia `teleop_bimanual_coupling`, ma nel `launch` non e usato.

Questo nodo aggiunge un wrench virtuale basato sulla differenza tra:

- distanza fra i due master;
- distanza fra i due slave.

Puo aiutare a:

- evitare che un braccio "scappi" rispetto all'altro;
- stabilizzare la presa su un oggetto rigido;
- ridurre i precarichi differenziali tra i due lati.

Limite:

- non e un vero controller di oggetto;
- lavora sulla distanza relativa L/R, non sulla distribuzione completa delle forze interne.

Per questo motivo e una buona soluzione intermedia, non la soluzione teoricamente migliore.

### C2. Ridurre il feedback per singolo braccio in modalita bimanuale

Quando entrambi i lati vanno in contatto, stai di fatto sommando due canali di feedback. Se ogni master riflette il suo contatto con la stessa aggressivita usata in modalita mono-braccio, il sistema totale diventa piu rigido e piu eccitabile.

Una strategia semplice e:

- usare guadagni di forza piu bassi quando la task e bimanuale;
- mantenere il feedback posizionale via molla;
- usare il feedback FT solo come rifinitura.

### C3. Soluzione migliore ma piu complessa: controllo in frame oggetto

Per presa stabile di un singolo oggetto, la soluzione piu pulita e spostare il controllo dal livello "due bracci indipendenti" al livello "oggetto virtuale":

- definire un frame oggetto comune;
- comandare il moto dell'oggetto;
- separare moto dell'oggetto e forze interne di presa;
- dissipare o limitare esplicitamente le forze interne.

Questo approccio e quello che meglio evita i rimbalzi da grasp bimanuale, ma richiede una revisione architetturale piu profonda.

## D. Correzioni strutturali sulla coerenza dei frame

### D1. Verifica prioritaria: gestione del `wrench_source_frame_override`

Se i messaggi FT hanno gia `header.frame_id` corretto e coerente in TF, la soluzione piu robusta e:

- non usare un override unico per master e slave;
- lasciare che ogni wrench venga ruotato dal proprio frame reale.

Se invece l'override ti serve davvero per il wrench del master, conviene separare i parametri:

- `master_wrench_source_frame_override`
- `slave_wrench_source_frame_override`

Nel codice attuale questo non e possibile senza modifica.

Questa e una correzione molto importante, perche un feedback di forza ruotato male diventa facilmente anti-dissipativo.

### D2. Verificare i frame delle pose nel canale di molla virtuale

Nel launch stai usando una logica di frame override basata su equivalenza numerica tra master e slave. Se questa equivalenza e solo approssimata, il feedback di posa puo contenere errori sistematici.

Se in free space il sistema funziona bene, probabilmente non e la causa principale. Pero in dual-arm e al contatto vale la pena ricontrollare:

- coerenza di `slave_base_frame`, `slave_tcp_frame` e `slave_frame_id_override`;
- coerenza del feedback `master_feedback_frame_override`;
- eventuale presenza di offset numerici diversi fra sinistra e destra.

## E. Soluzioni avanzate di stabilizzazione

### E1. Passivity controller / energy tank

Se vuoi mantenere un feedback di forza piu fedele senza rinunciare del tutto alla trasparenza, una soluzione avanzata e introdurre uno strato di passivita sul canale di feedback.

Varianti tipiche:

- `passivity observer / passivity controller`;
- `energy tank`;
- limitazione adattativa del feedback in funzione dell'energia netta iniettata.

Vantaggi:

- piu robustezza teorica ai ritardi;
- feedback di forza piu sicuro.

Svantaggi:

- maggiore complessita;
- tuning piu delicato;
- implementazione non banale.

### E2. Wave variables / scattering

Se il ritardo di comunicazione o di sincronizzazione e una componente dominante, la soluzione classica piu robusta e usare variabili di scattering.

E la strada piu "accademicamente solida" per teleoperazione con ritardi, ma comporta un redesign piu ampio del canale bilaterale.

## Strategia consigliata in pratica

Se l'obiettivo e migliorare la stabilita con il minimo rischio, io seguirei questo ordine:

1. testare il sistema con `force_reflection_scale = 0.0`;
2. introdurre `spring_damping_linear > 0` sul master;
3. ridurre o sospendere `dynamic_mass` e usare il `dynamic_damping` come leva principale;
4. rendere lo slave contact-aware con `force_limiter` e/o piccola compliance;
5. introdurre un coordinamento bimanuale esplicito;
6. solo dopo, se serve, recuperare un po' di feedback FT diretto;
7. se il problema resta strutturale, passare a una soluzione di passivita.

## Matrice di test consigliata

### Test 1 - Isolare il canale piu sospetto

Modifiche:

- `force_reflection_scale = 0.0`

Aspettativa:

- se il rimbalzo cala molto, la causa dominante e il feedback diretto di forza.

### Test 2 - Rendere piu dissipativa la molla virtuale

Modifiche:

- `spring_damping_linear > 0`
- eventualmente `spring_stiffness_linear` leggermente piu bassa

Aspettativa:

- meno oscillazione sull'errore master/slave;
- sensazione piu viscosa al contatto.

### Test 3 - Evitare che lo slave continui a spingere

Modifiche:

- `force_limiter.enabled = true`
- oppure piccolo `k_adm_linear > 0`

Aspettativa:

- minore accumulo di energia nel contatto;
- meno precarico sull'oggetto;
- grasp bimanuale piu stabile.

### Test 4 - Verificare il guadagno bimanuale

Modifiche:

- guadagni di forza piu bassi in modalita dual-arm;
- eventuale attivazione di `teleop_bimanual_coupling`

Aspettativa:

- meno lotta tra i due lati;
- oggetto meno compresso e meno torcente.

### Test 5 - Verifica frame wrench

Modifiche:

- eliminare l'override unico del frame del wrench oppure separarlo per master/slave

Aspettativa:

- feedback piu coerente come direzione;
- meno eccitazione anomala soprattutto fuori asse.

## Conclusione

Le due cause piu probabili, nel tuo setup attuale, sono:

1. feedback diretto della forza dello slave troppo aggressivo rispetto al resto del loop;
2. assenza di vera cedevolezza lato slave, che rende il contatto troppo "duro" e trasferisce tutta l'energia al canale bilaterale.

Il fatto che il problema peggiori molto nel grasp bimanuale indica inoltre che manca un coordinamento esplicito tra i due lati: oggi i due loop si accoppiano principalmente attraverso l'oggetto, che e la situazione meno favorevole dal punto di vista della stabilita.

Se dovessi scegliere una direzione unica, partirei da questa:

- meno forza riflessa diretta;
- piu damping relativo nel master;
- piu gestione del contatto lato slave;
- coordinamento bimanuale esplicito quando afferri un oggetto con entrambi i bracci.
