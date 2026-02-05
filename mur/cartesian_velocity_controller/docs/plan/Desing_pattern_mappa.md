# Design Pattern: Architettura Dati Ibrida per Rolling Voxel Grid

Questo documento formalizza la strategia di gestione della memoria e dei tipi di dato per il sistema di navigazione del Manipolatore Mobile.
L'obiettivo è bilanciare l'efficienza della cache (Memory Bandwidth) con la fluidità del controllo (Precisione Matematica).

---

## 1. Il Principio: Separazione tra Storage e Calcolo
Invece di utilizzare un unico tipo di dato per tutto il sistema, si adotta un approccio ibrido:

1.  **Input & Storage (I/O Bound):** Ottimizzati per la densità. Si usa `uint8_t`.
2.  **Processing & Control (CPU Bound):** Ottimizzati per la fisica. Si usa `float`.

### Analisi dei Vantaggi
* **Riduzione Traffico RAM:** Leggere una mappa di `uint8` è **4 volte più veloce** che leggere `float`. Poiché il collo di bottiglia è spesso la banda di memoria e non la CPU, questo accelera il ciclo di aggiornamento.
* **Cache Hit Rate:** In una singola linea di cache L1 (64 byte) entrano 64 celle `uint8` contro sole 16 celle `float`.
* **Nessuna Perdita di Precisione:** La conversione `uint8` $\to$ `float` è "lossless" (senza perdita), poiché la mantissa del float (23 bit) è molto più ampia dell'intero byte (8 bit).

---

## 2. Strategia per Singolo Layer

### A. Layer Sensori (Raw Input)
* **Tipo:** `std::vector<uint8_t>`
* **Valori:**
    * `0`: Spazio Libero.
    * `255`: Ostacolo (Muro/Oggetto).
    * `127`: Spazio Ignoto (opzionale).
* **Motivazione:** I sensori forniscono dati binari o ternari (c'è/non c'è). Usare 4 byte per dire "Sì/No" è uno spreco.

### B. Layer Predittivo (Tracked Persons)
* **Tipo:** `std::vector<uint8_t>`
* **Valori:** `0` a `255` (Mappa di Probabilità).
* **Logica:** Rappresenta la scia futura dell'ostacolo.
    * `255`: Centro della predizione (Alta probabilità).
    * `100-50`: Coda della predizione (Bassa probabilità).
* **Motivazione:** Le predizioni sono intrinsecamente rumorose. La precisione decimale del float è inutile qui.

### C. Layer Inflation (Safety Padding)
* **Tipo:** **Nessuno (Calcolato al volo)**.
* **Logica:** Non memorizzare una mappa statica per l'inflazione.
* **Implementazione:** Durante la costruzione della *Master Grid*, si applica una funzione di costo (es. esponenziale o lineare) basata sulla distanza dall'ostacolo nel *Layer Sensori*.

### D. Layer Goal (Attrazione)
* **Tipo:** **Nessuno (Analitico)**.
* **Logica:** Non usare RAM per creare una discesa lineare verso il target.
* **Implementazione:** Calcolo matematico diretto nel loop di controllo:
    $$U_{att}(x) = \frac{1}{2} k_{att} \cdot ||x - x_{goal}||^2$$

### E. Master Grid (Output per il Controllo)
* **Tipo:** `std::vector<float>`
* **Logica:** È l'unica griglia ad alta precisione.
* **Contenuto:** Somma pesata di tutti i layer precedenti convertiti + Inflazione.
* **Motivazione:** Il controllore necessita di calcolare il gradiente locale ($-\nabla U$). Se usassimo interi/byte, la derivata sarebbe a "scalini" (0 o max), causando movimenti a scatti e vibrazioni sui motori. I `float` garantiscono gradienti fluidi.

---

## 3. Note sull'Implementazione Hardware (Ryzen 7 + AVX2)

### Perché la conversione non rallenta il sistema?
Si potrebbe temere che convertire `uint8` in `float` ad ogni ciclo sia costoso. Su architetture moderne non è così:

1.  **Instruction Hiding:** Mentre la CPU attende che i dati arrivino dalla RAM (latenza), esegue le istruzioni di conversione "gratis".
2.  **Vettorizzazione (AVX2):** Con le flag `-O3 -march=native`, il compilatore usa istruzioni SIMD che caricano e convertono 8 o 16 celle simultaneamente in un singolo ciclo di clock.

---

## 4. Struttura della Classe C++ (`RollingMap.h`)

```cpp
class RollingMap {
private:
    // --- STORAGE (Input Compatti) ---
    // Risparmio memoria e massimizzazione Cache L1
    std::vector<uint8_t> layer_sensors_;     // Layer A
    std::vector<uint8_t> layer_predictions_; // Layer C

    // --- CALCOLO (Master Grid Fluida) ---
    // Necessaria per gradienti lisci (Anti-Jerk)
    std::vector<float> master_grid_; 

public:
    // Eseguito dal Thread Mapper (~20Hz)
    void updateMasterGrid() {
        // Reset veloce (vettorizzato)
        std::fill(master_grid_.begin(), master_grid_.end(), 0.0f);

        // Loop unico su tutti i voxel
        #pragma omp parallel for // Opzionale: parallelismo se necessario
        for(int i=0; i<N_voxels; i++) {
            
            // 1. Leggi Sensore (byte) -> Calcola Inflazione (float)
            // L'accesso a layer_sensors_[i] è rapidissimo
            float val_obs = calculate_inflation_kernel(layer_sensors_, i);
            
            // 2. Leggi Predizione (byte) -> Converti e Pesa (float)
            // Conversione implicita gestita via AVX dal compilatore
            float val_pred = layer_predictions_[i] * (1.0f / 255.0f) * PESO_PREDIZIONE;

            // 3. Somma nella Master
            master_grid_[i] = val_obs + val_pred;
        }
    }

    // Eseguito dal Thread Controller (>100Hz)
    float getTotalPotential(int x, int y, int z) {
        // 1. Componente Repulsiva (dalla memoria)
        float U_rep = master_grid_[get_ring_index(x,y,z)];
        
        // 2. Componente Attrattiva (Calcolo puro, zero RAM)
        float U_att = calculate_analytical_goal(x,y,z); 

        return U_rep + U_att;
    }
};