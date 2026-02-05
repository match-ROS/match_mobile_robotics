# Architettura Software per Evitamento Ostacoli su Manipolatore Mobile (UR10e)

Questo documento riassume la strategia architetturale definita per l'implementazione di un algoritmo di **Obstacle Avoidance** basato su Campi di Potenziale Artificiali (APF) utilizzando mappe 3D dinamiche in ambiente ROS Noetic (C++).

---

## 1. Panoramica dell'Architettura
L'obiettivo è passare da un approccio puramente reattivo (calcolo vettoriale istantaneo su primitive semplici) a un approccio ibrido **Map-Based**. Il sistema è diviso in due macro-blocchi disaccoppiati per garantire performance Real-Time:

1.  **Pipeline di Percezione (Bassa Frequenza ~10Hz):** Elabora i dati dei sensori e costruisce la rappresentazione del mondo.
2.  **Pipeline di Controllo (Alta Frequenza ~100Hz+):** Legge la mappa, calcola le distanze, genera le forze repulsive e comanda il robot.

---

## 2. Pipeline di Percezione (Nodelet Manager)
La gestione delle nuvole di punti (Point Clouds) è computazionalmente onerosa. Per evitare colli di bottiglia dovuti alla serializzazione dei messaggi via TCP/IP, tutti i nodi di questa catena devono essere implementati come **Nodelets**.

### Flusso Dati:
1.  **Sensor Driver:** Genera la nuvola di punti grezza (es. Realsense/Lidar).
2.  **Downsampling (VoxelGrid Filter):**
    * *Scopo:* Ridurre il numero di punti mantenendo la forma degli ostacoli.
    * *Configurazione:* Leaf size di 0.05m (5cm). Questo riduce drasticamente il carico sulla CPU per gli step successivi.
3.  **Region of Interest (PassThrough):**
    * *Scopo:* Tagliare via pavimento, soffitto e zone irraggiungibili dal robot.
4.  **Robot Self-Filtering:**
    * *Scopo:* Rimuovere i punti che appartengono al robot stesso.
    * *Criticità:* Senza questo, il robot "vedrebbe" il proprio braccio come un ostacolo e si bloccherebbe. Usare pacchetti come `robot_body_filter`.
5.  **Generazione Mappa (Octomap Server):**
    * *Scopo:* Integrare le nuvole filtrate in una mappa probabilistica di occupazione 3D (Octree).
    * *Output:* Topic `/octomap_binary` (più leggero della full map).

**Nota sul Clustering:** Per un puro algoritmo APF, il clustering (distinguere "tazza" da "libro") è superfluo e costoso. L'Octomap necessita solo dell'informazione "occupato/libero".

---

## 3. Gestione della Mappa (Octomap vs Grid)
Invece di una griglia fissa ($100 \times 100 \times 100$), si utilizza una **Octomap**.

### Perché Octomap?
* **Struttura Sparsa:** Memorizza solo i nodi occupati. Lo spazio vuoto non consuma memoria.
* **Multi-Risoluzione:** Permette di avere dettagli fini vicino agli oggetti e grossolani altrove.
* **Efficienza di Rete:** Pubblicare una `octomap_msgs` binaria è molto più efficiente che trasmettere matrici dense.

### Strategia di Aggiornamento
* Il pacchetto Percezione pubblica la mappa a **5-10 Hz**.
* Il pacchetto Controllo riceve la mappa e la memorizza localmente.
* *Non è necessario* che la mappa sia sincrona col ciclo di controllo (100Hz). Gli ostacoli non si muovono così velocemente da invalidare la pianificazione in 10ms.

---

## 4. Algoritmo di Controllo (Whole-Body Avoidance)
Applicare l'APF solo all'End Effector (EE) è pericoloso per un braccio robotico ("Floating Hand Problem"). Il gomito o la spalla potrebbero colpire ostacoli mentre l'EE li evita.

### Soluzione: Skeleton Approach
Si definiscono $N$ punti di controllo lungo la catena cinematica del robot (es. gomito, polso, centro dei link).

### Implementazione Matematica
Nel ciclo di controllo ad alta frequenza (100Hz):

1.  **Calcolo Distanze (DynamicEDT3D):**
    Utilizzare la libreria `dynamicEDT3D` (parte di Octomap).
    * Aggiorna la *Euclidean Distance Transform* basandosi sull'ultima Octomap ricevuta.
    * Per ogni punto di controllo $p_i$ sul robot, interroga `getDistance()` e `getClosestObstacle()`.
    * Questo evita di dover calcolare gradienti complessi su tutta la griglia. Il vettore distanza è il gradiente.

2.  **Calcolo Forze Repulsive:**
    Per ogni punto $p_i$, se la distanza $d < d_{min}$, genera una forza cartesiana:
    $$\vec{F}_{rep, i} = \eta \left( \frac{1}{d} - \frac{1}{d_{min}} \right) \frac{1}{d^2} \nabla d$$

3.  **Proiezione nello Spazio dei Giunti (Jacobiano):**
    Convertire le forze cartesiane (sui vari link) in velocità dei giunti usando la trasposta dello Jacobiano.
    
    $$\dot{q}_{cmd} = J_{ee}^{\dagger} (\vec{v}_{att}) + \sum_{i=1}^{N} J_{i}^{T} (\vec{F}_{rep, i})$$

    * $J_{ee}^{\dagger} (\vec{v}_{att})$: Componente attrattiva (Main Task) che guida l'EE verso il goal (usando Pseudo-Inversa Smorzata).
    * $\sum J_{i}^{T} (\vec{F}_{rep, i})$: Componente repulsiva (Secondary Task) che "spinge via" i link dagli ostacoli modificando la configurazione del braccio nel Null Space (o sommando velocità se non ridondante).

---

## 5. Sintesi delle Best Practices Implementative

### Da Fare (Do's)
* **Usa Nodelets:** Obbligatorio per la pipeline PointCloud -> Octomap.
* **Usa `dynamicEDT3D`:** Non reinventare l'algoritmo di calcolo distanza. Questa libreria è ottimizzata per aggiornamenti incrementali su Octomap.
* **Filtra il Robot:** Assicurati che il robot non veda se stesso.
* **Interpolazione:** Se possibile, implementa un filtro o interpolazione trilineare sulle distanze lette dalla mappa per evitare movimenti "a scatti" (jerky) quando si attraversano i voxel.

### Da Non Fare (Don'ts)
* **Non usare Griglie Dense:** Una matrice $100^3$ è inefficiente per ambienti ampi e sparsi.
* **Non inviare mappe dense via Topic:** Uccide la banda e la CPU.
* **Non applicare APF solo all'End Effector:** Rischio collisioni con il corpo del robot.
* **Non fare Clustering (per ora):** Aggiunge latenza inutile se l'obiettivo è solo evitare "cose solide".

---

## 6. Struttura dei Pacchetti ROS

**Pacchetto A: `my_robot_perception`** (Nodelets)
* `launch/perception.launch`:
    * Driver Sensori.
    * Nodelet Manager.
    * VoxelGrid Filter Nodelet.
    * Self Filter Nodelet.
    * Octomap Server Nodelet.

**Pacchetto B: `my_robot_control`** (Nodo C++ Standard)
* Sottoscrive: `/octomap_binary`, `/joint_states`.
* Classe `ObstacleAvoidance`:
    * Gestisce `DynamicEDT3D`.
    * Calcola Jacobiani ($J$ e $J^{\dagger}$).
    * Esegue loop a 100Hz -> Pubblica `/joint_group_vel_controller/command`.