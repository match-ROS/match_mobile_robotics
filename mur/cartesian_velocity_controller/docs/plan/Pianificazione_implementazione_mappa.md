# Proposta Architettonica: Navigazione Reattiva Unificata per Manipolatore Mobile in Ambienti Dinamici

## 1. Visione d'Insieme
L'obiettivo è realizzare un sistema di controllo per un **Manipolatore Mobile** (Base + Braccio) capace di:
1.  Navigare verso un goal in ambienti non strutturati.
2.  Evitare **ostacoli dinamici** (persone, altri robot) in tempo reale.
3.  Anticipare le collisioni tramite l'inserimento di **ostacoli fittizi** basati sulla predizione del movimento altrui.

La soluzione si basa sull'uso di **Campi di Potenziale Artificiali (Artificial Potential Fields)** calcolati su una griglia discretizzata 3D locale.

---

## 2. Struttura Dati: Egocentric Rolling Voxel Grid
Al posto di una Octomap (lenta nell'aggiornamento e nel calcolo dei vicini) o di una mappa globale statica, si utilizza una **griglia voxel a finestra scorrevole**.

* **Tipologia:** 3D Array a dimensione fissa (es. $100 \times 100 \times 50$ celle).
* **Logica:** La mappa è centrata sul robot. Quando il robot si muove, la mappa "scorre": i dati alle spalle vengono scartati, i nuovi dati frontali vengono inizializzati.
* **Vantaggi:**
    * Accesso alla memoria in tempo costante $O(1)$.
    * Ideale per calcoli matriciali ad alta frequenza.
    * Basso consumo di memoria RAM.

---

## 3. Modulo Builder: Generazione della Mappa di Costo
Questo modulo gira a media frequenza (es. 10-20 Hz) e costruisce la **Master Grid** sommando diversi "Layers" (livelli) di potenziale.

### I Livelli (Layers)
1.  **Layer A (Percezione Grezza):** Dati da camere RGBD/Lidar.
    * I voxels occupati valgono **100** (Muro).
    * Applicazione di un *Time Decay*: se un voxel non viene riconfermato dopo $t$ millisecondi, il suo valore torna a 0 (elimina le scie degli oggetti in movimento).
2.  **Layer B (Inflation & Sicurezza):**
    * Espansione degli ostacoli del Layer A.
    * Crea un gradiente di repulsione immediata attorno agli oggetti.
3.  **Layer C (Ostacoli Fittizi / Predittivi):**
    * **Input:** Tracking degli oggetti (es. Filtro di Kalman su persone rilevate).
    * **Logica:** Proiezione del vettore velocità dell'ostacolo.
    * **Output:** Generazione di un volume di voxels ad alto costo *davanti* alla persona (nella direzione del suo moto).
    * **Effetto:** Il robot percepisce lo spazio *futuro* occupato dalla persona come un muro invalicabile *oggi*.
4.  **Layer D (Goal Attrattivo):**
    * Gradiente lineare che decresce verso il target o verso il bordo della griglia più vicino al target globale.

### L'Algoritmo di Navigazione: Wavefront (Grassfire)
Sulla *Master Grid* risultante viene applicato l'algoritmo **Wavefront**:
1.  Si parte dal Goal (valore 0).
2.  Si propaga il valore ai vicini ($+1$ costo) come un'onda, aggirando gli ostacoli (reali e fittizi).
3.  **Risultato:** Una mappa $U(x,y,z)$ priva di minimi locali all'interno dell'area libera.

---

## 4. Modulo Driver: Strategia di Controllo (Whole-Body)
Il robot non pianifica un percorso geometrico complesso, ma reagisce istantaneamente al gradiente locale della mappa ($-\nabla U$). Base e Braccio sono coordinati tramite un controllore gerarchico (**Stack of Tasks**).

### A. La Base Mobile
* Legge una "fetta" o proiezione 2D della *Rolling Voxel Grid*.
* Segue la discesa più ripida verso il goal.
* Se un "Muro Fittizio" (Layer C) blocca la strada, il Wavefront ricalcola istantaneamente un percorso alternativo, facendo curvare la base prima che avvenga l'incontro.

### B. Il Braccio Robotico (Skeleton Repulsion)
Non si usa pianificazione lenta (RRT) per l'evitamento reattivo.
1.  Si definiscono dei **Control Points** lungo la catena cinematica del braccio (gomito, polso, ecc.).
2.  Ogni punto interroga la *Rolling Voxel Grid* per conoscere il potenziale nella sua posizione $xyz$.
3.  Se il potenziale è alto (vicinanza a ostacolo/persona), si genera una **Forza Repulsiva Cartesiana**.
4.  Le forze vengono mappate ai giunti tramite la **Matrice Jacobiana Trasposta**:
    $$
    \tau_{rep} = J^T(q) \cdot F_{rep}
    $$
    Questo fa sì che il braccio si "ritragga" o si sposti automaticamente se qualcuno si avvicina troppo.

---

## 5. Sintesi del Flusso Dati

```mermaid
graph TD
    A[Sensori RGBD] -->|Point Cloud| B[Voxel Grid Layer A (Ostacoli)]
    C[Tracking Persone] -->|Velocità & Posizione| D[Voxel Grid Layer C (Fittizi)]
    E[Goal Globale] --> F[Voxel Grid Layer D (Attrazione)]
    
    B & D & F --> G{Master Grid Sum}
    G -->|Algoritmo Wavefront| H[Mappa di Potenziale U]
    
    H --> I[Modulo Base]
    H --> L[Modulo Braccio]
    
    I -->|Gradiente 2D| M[Motori Ruote]
    L -->|Gradiente 3D su Scheletro| N[Motori Giunti]
    
    M & N --> O[Whole-Body Motion]
```