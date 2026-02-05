# Analisi Tecnica: Implementazione Rolling Window Grid per Manipolatore Mobile

Questo documento analizza la fattibilità tecnica e le strategie di ottimizzazione per l'implementazione di un sistema di *Obstacle Avoidance* basato su **Egocentric Rolling Voxel Grid** (come descritto nella Proposta 1), in contrasto con l'approccio Octomap (Proposta 2).

Il sistema è pensato per girare su **ROS Noetic (C++)**.

---

## 1. Analisi di Fattibilità e Carico Computazionale

L'approccio a "finestra scorrevole" è lo standard industriale per la navigazione reattiva ad alta frequenza (usato in guida autonoma e droni) grazie al determinismo dei tempi di accesso.

### Dimensionamento
Basandosi sulle specifiche di progetto ($100 \times 100 \times 50$ voxels):
* **Totale celle:** $500.000$ elementi.
* **Occupazione Memoria:**
    * `uint8_t` (costo 0-255): **~0.5 MB**.
    * `float` (costo continuo): **~2 MB**.
* **Impatto Hardware:** L'intera mappa rientra comodamente nella **Cache L3** (e spesso L2) delle moderne CPU (i7/i9).

**Conclusione:** La lettura dei dati è quasi istantanea. Il collo di bottiglia potenziale risiede solo nelle operazioni di scrittura e scorrimento (shift), se non ottimizzate.

---

## 2. Strategie di Ottimizzazione C++ (Performance Critical)

Per garantire frequenze di controllo $>100\text{Hz}$, è necessario evitare copie di memoria inutili.

### A. Gestione della Memoria: Ring Buffer (Circular Array)
Non spostare i dati, sposta l'origine. Eseguire una `memcpy` su 500k celle ogni volta che il robot si muove di 10cm è inefficiente.

* **Soluzione:** Utilizzare indici "virtuali" con logica circolare.
* **Accesso:** L'indice della cella $(x,y,z)$ viene calcolato come:
    $$Index = ((x + offset\_x) \% 100) + ((y + offset\_y) \% 100) \cdot W + \dots$$
* **Aggiornamento:** Quando il robot avanza, si aggiorna solo l'integer `offset_x` e si esegue il `memset` a 0 (pulizia) solo sulla "fetta" di nuovi dati che entra nella griglia.
* **Vantaggio:** Costo computazionale $O(\text{fetta})$ invece di $O(\text{tutta la mappa})$.

### B. Somma dei Layer (Master Grid)
La costruzione della mappa finale richiede la somma dei vari layer (Sensori, Inflation, Predizione Persone).
* **Soluzione:** Utilizzare istruzioni vettoriali **SIMD/AVX**.
* **Implementazione:** Usare flag di compilazione `-O3 -march=native` o librerie come **Eigen** mappate sui buffer raw. Questo permette di sommare blocchi di 8 o 16 float in un singolo ciclo di clock.

### C. Algoritmo di Navigazione: EDT vs Wavefront
Un algoritmo Wavefront completo (Grassfire) può essere costoso se eseguito ad alta frequenza su tutta la griglia.
* **Soluzione:** Per l'evitamento reattivo, è spesso sufficiente l'**Inflation** o una **Euclidean Distance Transform (EDT)** lineare (es. algoritmo di Meijster).
* **Logica:** Il robot non necessita di un pathfinding globale perfetto in questo stadio, ma del gradiente locale ($\nabla U$) per "scivolare" via dagli ostacoli.

---

## 3. Architettura Software Suggerita (Multithreading)

Per non bloccare il loop di controllo real-time, l'architettura del nodo C++ deve essere divisa in due thread distinti.

### Thread 1: Mapper (Bassa Frequenza ~15-20 Hz)
Si occupa del carico pesante di elaborazione dati.
1.  Riceve PointCloud (filtrata) e Tracking Persone.
2.  Gestisce il **Rolling** (aggiornamento offset circolari).
3.  Scrive i dati nei **Layer A** (sensori) e **Layer C** (persone fittizie).
4.  Calcola il **Layer B** (Inflation/EDT) e somma tutto nella **Master Grid**.
5.  Esegue lo swap del puntatore (Double Buffering) per rendere la mappa disponibile in lettura.

### Thread 2: Controller (Alta Frequenza ~100-500 Hz)
Deve essere leggero e privo di allocazioni dinamiche nel loop.
1.  Legge l'ultimo puntatore valido della **Master Grid**.
2.  Aggiorna la cinematica del braccio (**Skeleton Points**).
3.  Per ogni punto dello scheletro:
    * Calcola l'indice nella griglia circolare.
    * Estrae il valore e calcola il gradiente $\nabla U$ (differenza finita con i vicini).
4.  Calcola le forze repulsive e le converte in comandi ai giunti ($\tau$ o $\dot{q}$).

---

## 4. Confronto Tecnico: Rolling Grid vs Octomap

Perché la Rolling Grid (Proposta 1) è superiore all'Octomap (Proposta 2) per questo specifico caso d'uso:

| Caratteristica | Rolling Grid (Tua Scelta) | Octomap + DynamicEDT3D |
| :--- | :--- | :--- |
| **Accesso Memoria** | **Coerente (Cache Friendly)**. Scansione lineare dell'array. | **Frammentato (Pointer Chasing)**. Salti continui in RAM, molti *cache miss*. |
| **Aggiornamento** | Costo fisso e predicibile (tempo costante). | Costo variabile (inserimento nodi nell'albero). |
| **Calcolo Distanze** | EDT su array (molto veloce). | DynamicEDT deve attraversare la struttura ad albero. |
| **Overhead** | Implementazione custom necessaria (ma snella). | Librerie pronte (`octomap`), ma computazionalmente pesanti per update continui. |

---

## 5. Sintesi
L'utilizzo di una **Rolling Voxel Grid** implementata come **Ring Buffer** in C++ è la soluzione ottimale per garantire un controllo *whole-body* reattivo e sicuro. Permette di mantenere i tempi di ciclo del controller stabili (<1ms) indipendentemente dalla complessità dell'ambiente, cosa non garantita dalle strutture ad albero come Octomap.