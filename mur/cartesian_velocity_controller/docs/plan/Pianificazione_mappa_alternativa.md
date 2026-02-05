# Architettura di Navigazione e Obstacle Avoidance per Manipolatore Mobile
**Data:** 15 Gennaio 2026
**Contesto:** Manipolatore Mobile (es. UR10e su base mobile)
**Obiettivo:** Evitamento ostacoli reattivo in 3D usando APF (Artificial Potential Fields) e controllo cinematico.

---

## 1. Panoramica dell'Architettura
L'approccio scelto è **ibrido**: disaccoppia la percezione (lenta, pesante) dal controllo (veloce, reattivo). L'architettura si divide in due pacchetti software principali comunicanti via ROS.

### I Principi Chiave
1.  **Mappa Dinamica 3D:** Uso di **Octomap** per rappresentare l'ambiente sparso in modo efficiente.
2.  **Whole-Body Avoidance:** Non proteggiamo solo l'End-Effector, ma tutto il corpo del robot (gomito, spalla, polso) usando uno "scheletro" di punti di controllo.
3.  **Distanza Reale (EDT):** Uso della *Dynamic Euclidean Distance Transform* per calcolare distanze e gradienti in tempo reale.
4.  **Ottimizzazione Dati:** Uso di **Nodelets** per la pipeline di percezione per azzerare la latenza di copia memoria.

---

## 2. Pacchetto A: Percezione e Mapping (The Eye)
*Responsabilità: Acquisire dati dai sensori, filtrarli e mantenere una mappa 3D aggiornata.*
*Frequenza target:* 5-15 Hz.
*Tecnologia:* ROS Nodelets (C++).

### Pipeline dei Nodelets
I dati fluiscono in questa sequenza all'interno di un unico `nodelet_manager`:

1.  **Sensor Driver:** (es. Realsense/Lidar) → Pubblica `PointCloud2` grezza.
2.  **VoxelGrid Filter:** Downsampling della nuvola (es. foglia da 5cm). Riduce il numero di punti da processare.
3.  **PassThrough / CropBox:** Taglia i punti fuori dall'area di interesse (es. soffitto, pavimento lontano).
4.  **Robot Self-Filter:** (Critico) Rimuove i punti che appartengono al corpo del robot stesso usando l'URDF. Evita che il robot rilevi se stesso come ostacolo.
5.  **Octomap Server:**
    * Input: Nuvola filtrata.
    * Logica: Costruisce incrementalmente l'occupazione (Occupato/Libero/Ignoto).
    * Output: `octomap_msgs/Octomap` (preferibilmente BinaryMap per leggerezza).

---

## 3. Pacchetto B: Controllo e Pianificazione (The Brain)
*Responsabilità: Calcolare la cinematica inversa e le forze repulsive per guidare il robot.*
*Frequenza target:* 100 Hz (Real-time).
*Tecnologia:* Nodo C++ Standard.

### Logica Interna
Il nodo mantiene un oggetto `DynamicEDT3D` che viene aggiornato ogni volta che arriva una nuova Octomap (callback asincrona).

#### A. Gestione della Mappa (Lenta)
Alla ricezione del topic `/octomap_binary`:
1.  Deserializza l'Octree.
2.  Aggiorna l'oggetto `DynamicEDT3D` (calcola la distanza di ogni voxel dall'ostacolo più vicino entro un raggio $R_{max}$).
3.  *Nota:* Questo avviene in un thread separato per non bloccare il loop di controllo.

#### B. Loop di Controllo (Veloce - 100Hz)
Ad ogni ciclo:
1.  **Forward Kinematics:** Calcola la posizione cartesiana dei punti dello "Scheletro" del robot (es. $p_{gomito}, p_{polso}, p_{ee}$).
2.  **Interrogazione Mappa:** Per ogni punto dello scheletro:
    * Ottieni distanza $d$ e vettore direzione ostacolo $\vec{n}$ dalla `DynamicEDT3D`.
    * Se $d < d_{safe}$, calcola forza repulsiva $\vec{F}_{rep}$.
3.  **Calcolo Velocità Giunti (DLS):**
    Combina l'attrazione verso il goal (solo EE) e la repulsione dagli ostacoli (tutti i link) proiettando tutto nello spazio dei giunti.

---

## 4. Formulazione Matematica (Control Law)

La velocità di riferimento ai giunti $\dot{q}_{cmd}$ è calcolata come:

$$
\dot{q}_{cmd} = J_{ee}^{\dagger}(\vec{v}_{att}) + \sum_{i \in skeleton} J_{i}^{T} \vec{F}_{rep, i}
$$

Dove:
* $J_{ee}^{\dagger}$: Pseudo-inversa smorzata (Damped Least Squares) dello Jacobiano all'End-Effector.
    $$J^{\dagger} = J^T (JJ^T + \lambda^2 I)^{-1}$$
* $\vec{v}_{att}$: Velocità cartesiana desiderata verso il target (Goal).
* $J_{i}^{T}$: Trasposta dello Jacobiano calcolato nel punto di controllo $i$ (es. gomito).
* $\vec{F}_{rep, i}$: Vettore forza repulsiva cartesiana applicata al punto $i$.

**Interpolazione:** Per evitare movimenti a scatti ("jerky"), le forze repulsive derivate dalla mappa voxelizzata devono essere passate attraverso un'interpolazione trilineare o un filtro passa-basso temporale.

---

## 5. Dettagli Implementativi e Best Practices

### Perché Octomap + DynamicEDT?
* **Efficienza:** Octomap è sparso (non spreca memoria per lo spazio vuoto).
* **Robustezza:** DynamicEDT gestisce il calcolo della distanza in modo efficiente, aggiornando solo i voxel modificati.
* **Separazione:** Permette di cambiare la formula del potenziale ($1/d$, $1/d^2$, esponenziale) nel controller senza dover ricalcolare la mappa.

### Perché i Nodelets?
Le nuvole di punti sono pesanti (MBs). Passarle tra nodi via topic standard richiede serializzazione (copia profonda) che usa molta CPU e introduce latenza.
* **Soluzione:** I Nodelets girano come thread nello stesso processo.
* **Risultato:** Zero Copy Transport (passaggio di puntatori). La latenza diventa trascurabile.

### Configurazione MoveIt
Se si utilizza MoveIt in parallelo:
* Non pubblicare la griglia di potenziale.
* Usare il `PlanningSceneMonitor` per sincronizzare la conoscenza degli ostacoli se necessario per il Global Planner, ma mantenere il controllo reattivo (APF) leggero come descritto sopra.

---

## 6. Tabella di Marcia (Next Steps)

1.  **Setup Nodelets:** Creare il launch file per driver camera + filtri PCL + Octomap Server.
2.  **Verifica Percezione:** Visualizzare in RViz la `/octomap_binary` e assicurarsi che il robot non veda se stesso (tuning del Self-Filter).
3.  **Integrazione Libreria:** Includere `dynamicEDT3D` nel nodo controller.
4.  **Test Statico:** Posizionare un ostacolo, muovere il robot manualmente vicino ad esso e stampare a video i vettori repulsivi calcolati.
5.  **Chiusura del Loop:** Attivare il comando motori sommando le velocità repulsive.