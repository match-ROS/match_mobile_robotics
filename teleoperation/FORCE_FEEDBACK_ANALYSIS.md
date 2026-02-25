# Analisi del Sistema di Teleoperazione con Force Feedback

## Il Problema: Forte Accoppiamento Bidirezionale
Hai notato che applicando una forza sul master, lo slave si muove (comportamento corretto e desiderato), ma **applicando una forza sullo slave, anche il master si muove**. 

Questo comportamento non è un bug nel tuo codice, ma una conseguenza diretta di come è strutturata l'architettura di controllo che hai implementato. L'architettura è una **Teleoperazione Admittanza-Admittanza** o **Posizione-Forza (PF)** con ritrasmissione diretta della forza al master.

### Analisi Matematica del Comportamento
Nel file `teleop_master_haptic_controller_node.cpp`, la dinamica del master è governata dall'equazione dell'admittanza:
```cpp
const Eigen::Vector3d F_feedback = (-kf_force_ * slave_filt_.f) + coupling_filt_.f;
const Eigen::Vector3d rhs_lin = (F_hand - F_feedback) - D_lin.cwiseProduct(v_lin_cmd_);
a_lin_des_normed = rhs_lin.cwiseQuotient(M_lin);
```
Semplificando l'equazione per il moto traslazionale (ignorando il coupling di una seconda interfaccia per chiarezza), otteniamo un'equazione del tipo:
$$ M \ddot{x}_{master} + D \dot{x}_{master} = F_{hand} + k_{f} F_{slave} $$

Questa equazione spiega in modo esatto cosa accade nel sistema:
1. **Spinta sul master**: Se l'operatore applica una forza sul master ($F_{hand} > 0$) e lo slave è libero senza contatti ($F_{slave} \approx 0$), il master accelera in base alle masse/smorzamenti virtuali. Lo slave, essendo in un inner-loop o in un outer-loop che insegue la posa (target) generata dal master, **si muove di conseguenza**. (Funzionamento corretto).
2. **Spinta sullo slave**: Se l'operatore rilascia il master ($F_{hand} = 0$) e qualcuno (o un impatto) spinge l'end-effector dello slave, il sensore F/T dello slave rileva una forza ($F_{slave} > 0$). L'equazione di admittanza del master diventa:
   $$ M \ddot{x}_{master} + D \dot{x}_{master} = k_{f} F_{slave} $$
   A questo punto il master calcola una variazione di velocità non nulla e **inizia muoversi fisicamente** obbedendo alla forza riflessa. Dal momento che il master si muove mandando una nuova target pose allo slave, **lo slave segue a sua volta il master**. Questa è una risposta perfettamente "retroazionabile" (o una perfetta backdrivability, tipica della Position-Force architecture) che unisce entrambi i lati del sistema. Nel momento in cui nessuno tiene il master "fermo", qualsiasi forza sullo slave mette in movimento l'intero sistema da ambo le parti.

Senza l'operatore che chiude il loop trattenendo meccanicamente l'asse del master e imponendo una rigidezza manuale, ed essendo la rigidezza nel controllo nulla (Virtual Spring $K = 0$), il braccio è libero di allontanarsi indefinitamente se percepisce forze dal lato ricevente.

---

## Suggerimenti Architetturali e Miglioramenti

Questo comportamento è considerato un pregio per compiti in cui serve altissima "trasparenza", ma risulta poco pratico se non si sta impugnando strettamente il master, soprattutto prima di un'interazione, o se lo slave muove carichi elevati che inducono disturbi sul load cell. Ecco alcune strategie per migliorarlo.

### 1. "Deadman Switch" (Uomo-Presente) - Fortemente Consigliato
Il modo più sicuro per mantenere le qualità di trasparenza del teleoperatore, senza averlo che "fugge", è far sì che il force feedback sia attivo o si manifesti, **soltanto se l'operatore sta attivamente impugnando e muovendo il master**.
* **Come fare**: Se l'handle del master non ha un pulsante hardware per chiudere il loop, puoi inserire una semplice deadband su logica condizionale. Ad esempio, se la norma di $|F_{hand}|$ è stabilmente al di sotto di una certa soglia (l'operatore ha lasciato la presa), puoi annullare `kf_force_` e `kf_torque_`, impedendo alla forza riflessa di disturbare il robot lasciato a sé stesso, e al contempo alzare il parametro della frizione virtuale per non farlo strisciare per semplice inezia.

### 2. Damping Variabile / Frenata Attiva
Nel tuo file `master_real_mur620b_ur10_l.yaml` applichi uno smorzamento lineare costante (`damping_linear: 10.0`). 
Sarebbe ideale modulare questa frizione:
* Quando la forza del master è trascurabile, il robot deve sembrare frenato e pesante. (e.g. `damping_linear = 80.0`)
* Appena l'operatore inizia a stringere il sensore, scala questo valore linearmente verso `10.0`, restituendo un feeling leggero finché trattiene l'handle e lo pilota. Se a quel punto un muro spingesse contro lo slave, lo sentiresti scuotere il braccio (la spinta tornerebbe in gioco, ma non fuggirebbe fuori controllo perché tu imposteresti un bilanciamento forte o faresti scattare i limiti del load cell).

### 3. Disaccoppiare i Filtri (e Deadband) di Master e Slave
All'interno di `tick`, stai usando le variabili di configurazione:
```cpp
slave_filt_.f = softDeadzoneNormWithHysteresis(slave_filt_.f, force_deadband_enter_, force_deadband_exit_, f_slave_active_);
```
In pratica riutilizzi la **stessa variabile `force_deadband_enter_` (che su yaml è 2.0 N)** per eseguire una cut-off sia sul controller del master che dello slave. 
Si raccomanda fortemente di **creare configurazioni separate** per il Master e lo Slave in questo nodo. Una spinta di 3 N fatta con un dito su una scrivania (Slave) è sufficiente per avvertire la forza al Master. Il Load cell dello Slave necessiterebbe verosimilmente di una deadband molto più aggressiva del Load Cell che ha la tua mano. Un `slave_force_deadband` = `10.0 N` filtrerebbe tutto quel fastidioso e piccolo feedback, permettendo solo alle forze decise (come i veri urti della base o dei prelievi pesanti) di tornare in cascata nelle mani del pilota. E' il modo più facile e pratico in assoluto in cui limitare la backdrivability.

### 4. Gravity e Inertia Compensation (Dinamica Inversa) Configurate sullo Slave
Ogni volta che il braccio dello slave oscilla a vuoto e accellera la pinza o il tool montato sulla giuntura polso del sensore, **i carichi della pinza subiscono un accellerazione e generano un offset in Lettura**. Lo slave crede che tu stai per scontrarti e "piega" virtualmente indietro la pinza al master di riflesso provocando dei colpi, che spesso evolvono in forti oscillazioni di sistema.
Sia la cella di carico sinistra che quella di destra del MUR620b devono implementare una pre-compensazione dinamica del tool collegato all'end-effector (massa, baricentro, ecc..). Verifica che la massa della pinza sia azzerata dal F/T sensor in ROS.

## Sintesi
Il sistema che hai codificato funziona esattamente come è stato matematicamente ideato. La **perfetta stazionarietà e accoppiamento bidirezionale** dei due bracci sono il sintomo di come la legge di controllo sia coerente con l'approccio Posizione-Forza implementato in un blocco di feedback Admittativo. Aggiungere una **frenata in assenza di comando manuale (Deadman logic)** o implementare una **Deadband superiore asimmetrica dedicata allo Slave**, dovrebbero essere il tuo prossimo passo in ROS per sopperire al sistema non smorzato che stai riscontrando.
