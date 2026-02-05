# Analisi dello Stato dell'Arte (SOTA) per Local Obstacle Avoidance

**Data:** 19 Gennaio 2026  
**Contesto:** Confronto tra l'architettura proposta (Voxel Grid + EDT) e le tecnologie di frontiera.

---

## 1. Definizione di "Stato dell'Arte"

Nel campo della robotica mobile e della manipolazione, il concetto di SOTA si divide in due livelli distinti:

1.  **SOTA Industriale/Applicativo (High-End):**
    * Tecnologie robuste, mature e affidabili.
    * Utilizzate in prodotti commerciali avanzati e ricerca applicata (es. Boston Dynamics, logistica avanzata).
    * *La tua architettura (Voxel Grid + EDT su CPU) rientra pienamente in questa fascia.*

2.  **SOTA "Bleeding Edge" (Ricerca Pura):**
    * Algoritmi sperimentali che spingono i limiti delle performance.
    * Spesso richiedono hardware dedicato (GPU potenti) e sono complessi da integrare/mantenere.
    * *Esempi:* NVIDIA cuRobo, Voxblox (ETH Zurich).

---

## 2. Le Tre Grandi Differenze Tecnologiche

Ecco cosa differenzia la tua soluzione dalle tecniche di ricerca più estreme.

### 2.1. La Mappa: Da CPU EDT a GPU ESDF

La tua architettura rigenera la mappa EDT sulla CPU (calcolo $O(N)$) ad ogni ciclo (10-20 Hz).

**Lo Stato dell'Arte (SOTA):**
* **ESDF Incrementale (es. Voxblox):** Non rigenera tutto. Aggiorna *solo* i voxel che sono cambiati rispetto al frame precedente. Risparmia CPU e permette mappe più grandi.
* **GPU Acceleration (es. NVIDIA cuRobo):** Sposta l'intera pipeline (Voxelizzazione → EDT → Pianificazione) su GPU (CUDA).
* **Risultato:** Frequenze di aggiornamento mappa > 500 Hz con risoluzioni millimetriche (es. voxel da 5mm), contro i 10-20 Hz a 5cm della soluzione CPU.

### 2.2. Il Controllo: Da Reattivo a Predittivo (MPC)

Il tuo approccio usa **Campi di Potenziale Artificiale (APF)**. È una logica *reattiva*: "Vedo un ostacolo vicino → Spingo via subito".

**Lo Stato dell'Arte (SOTA):**
* **Model Predictive Control (MPC):** Il controller risolve un problema di ottimizzazione in tempo reale per prevedere il futuro (es. i prossimi 0.5 - 1.0 secondi).
* **Logica:** Invece di frenare all'ultimo (reattivo), l'MPC vede che tra 0.5s avverrà una collisione e inizia a modificare la traiettoria dolcemente *ora*.
* **Risultato:** Movimenti estremamente fluidi e ottimali, ma richiede solver matematici complessi e molta potenza di calcolo.

### 2.3. La Percezione: Da Statica a Dinamica

Nella tua mappa, un ostacolo che si muove è trattato come una serie di ostacoli statici che appaiono e scompaiono.

**Lo Stato dell'Arte (SOTA):**
* **Prediction & Tracking:** Il sistema riconosce gli oggetti (es. "persona") e ne stima la velocità e direzione.
* **Evitamento Futuro:** Il robot evita non solo dove l'ostacolo è *adesso*, ma dove *sarà* tra un secondo.

---

## 3. Tabella di Confronto: Architettura Proposta vs. SOTA

| Caratteristica | Architettura Proposta (Voxel + EDT) | Stato dell'Arte (SOTA - es. cuRobo/Voxblox) |
| :--- | :--- | :--- |
| **Generazione Mappa** | Ricostruzione completa (CPU) | Incrementale (CPU) o Parallela (GPU) |
| **Rappresentazione** | Voxel Grid + EDT | TSDF (Truncated Signed Distance Field) / Hash Map |
| **Frequenza Mappa** | 10-20 Hz | 30-100+ Hz (fino a 500Hz su GPU) |
| **Logica Controllo** | Reattiva (Potential Fields) | Predittiva (NMPC - Nonlinear MPC) |
| **Fluidità** | Ottima (con interpolazione trilineare) | Perfetta (ottimizzata nel tempo) |
| **Complessità** | Medio-Alta (Gestibile da 1 sviluppatore) | Altissima (Richiede team/librerie complesse) |

---

## 4. Verdetto Finale

La tua soluzione si posiziona come **ottimo standard industriale**.

* È superiore alle soluzioni accademiche di base (che usano 2D Lidar o Pointcloud grezze).
* Risolve i problemi critici di real-time safety (tramite Double Buffering) e fluidità (tramite EDT).
* Passare al livello successivo (SOTA GPU/MPC) richiederebbe un cambio radicale di hardware (Jetson/GPU dedicata) e una complessità software esponenzialmente maggiore, spesso non necessaria per le velocità operative di un cobot come l'UR10e.

**Conclusione:** L'architettura proposta è il **compromesso tecnico ideale** per questo progetto.