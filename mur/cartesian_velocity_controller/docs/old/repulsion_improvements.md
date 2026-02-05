# Proposte di Miglioramento: Gestione Ostacoli e Curve Repulsive

Questo documento raccoglie analisi e proposte di miglioramento per il sistema di generazione delle velocità repulsive, con particolare focus sulla gestione di ostacoli multipli e sulla fluidità della funzione di magnitudine.

## 1. Gestione di Ostacoli Multipli per POI

### Stato Attuale
Attualmente, il `RepulsionDataManager` filtra gli ostacoli in modo che ogni Point of Interest (POI) consideri **solamente l'ostacolo più vicino** (approccio *Winner-Takes-All*).

```cpp
// Logica attuale
if (nuovo_ostacolo.distanza < ostacolo_corrente.distanza) {
    ostacolo_corrente = nuovo_ostacolo;
}
```

**Problema:** Questo approccio causa discontinuità nel vettore velocità ("switching"). Se il robot si muove e l'ostacolo più vicino cambia improvvisamente (es. da destra a sinistra), la direzione della forza repulsiva cambia istantaneamente, causando scatti o vibrazioni nel movimento del robot.

### Soluzione Proposta: Somma Vettoriale
È fortemente consigliato permettere al robot di "sentire" tutti gli ostacoli contemporaneamente. Le forze repulsive dovrebbero sommarsi vettorialmente.

**Vantaggio:** Il movimento diventa fluido. Esempio: due ostacoli opposti a uguale distanza annullano le loro forze invece di far "rimbalzare" il robot tra l'uno e l'altro.

### Implementazione Ottimizzata (Basso Costo Computazionale)
Per evitare un aumento della complessità computazionale (in particolare il calcolo ripetuto dello pseudo-inverso del Jacobiano), si propone la seguente strategia:

1.  **RepulsionDataManager**: Rimuovere il filtro del "più vicino". Restituire una lista di tutte le coppie (POI-Ostacolo) che si trovano entro la `influence_distance`.
2.  **LocalPlanner**:
    *   **Step A (Raggruppamento)**: Raggruppare i dati per POI (es. raccogliere tutti gli ostacoli che influenzano il link "gomito").
    *   **Step B (Somma Cartesiana)**: Calcolare i vettori velocità repulsiva cartesiana ($\vec{v}_{cart, i}$) per ogni ostacolo $i$ e sommarli tra loro:
        $$ \vec{v}_{cart, tot} = \sum_{i} \vec{v}_{cart, i} $$
    *   **Step C (Proiezione Unica)**: Eseguire il calcolo del Jacobiano e della proiezione nello spazio dei giunti **una sola volta** per POI, utilizzando il vettore somma risultante:
        $$ \dot{q}_{rep} = J^{\dagger} \cdot \vec{v}_{cart, tot} $$

In questo modo, il costo computazionale aggiuntivo è trascurabile (solo somme di vettori 3D), mantenendo invariato il numero di operazioni matriciali pesanti.

---

## 2. Ottimizzazione della Funzione di Magnitudine

### Stato Attuale
La funzione attuale implementa una legge quadratica inversa ($1/d^2$) con uno smussamento ("tapering") manuale ai bordi.

Formule approssimative attuali:
*   Zona sicura: $v = v_{max}$
*   Zona intermedia: $v \propto \frac{1}{d^2}$
*   Zona esterna (taper): Moltiplicazione per un fattore lineare per raccordare a zero.

**Problema:** Sebbene funzionale, la curva è "ingegnerizzata" attraverso più stadi if/else e richiede un parametro di tapering manuale per evitare che la forza appaia improvvisamente entrando nella zona di influenza.

### Soluzione Proposta: Curve di Khatib o Polinomiali
Si consiglia di adottare una funzione standard per i campi potenziali che garantisca continuità matematica $C^1$ (velocità e derivata nulle al bordo), eliminando la necessità di logiche di tapering manuale.

#### Opzione Consigliata (Parabola "Distance-to-Boundary")
Questa funzione decresce in modo naturale man mano che ci si avvicina al bordo dell'influenza ($d_{inf}$).

$$
||\vec{v}_{rep}|| = v_{max} \cdot \left( \frac{d_{influence} - d}{d_{influence} - d_{min\_safe}} \right)^2 \quad \text{per } d > d_{min\_safe}
$$

**Vantaggi:**
1.  **Fluidità perfetta:** A $d = d_{influence}$, la velocità è esattamente 0.
2.  **Tangente orizzontale:** La derivata della curva al bordo è dolce, evitando il "calcio" iniziale quando l'ostacolo entra nel raggio di percezione.
3.  **Semplicità:** Rimuove la logica condizionale del "tapering" e l'uso di soglie arbitrarie (come `0.8 * influence`).

### Confronto Visivo

*   **Attuale:** Crescita rapida ($1/d^2$) ma richiede "pezze" software per essere liscia ai bordi.
*   **Proposta:** Crescita quadratica dolce che gestisce naturalmente l'ingresso e l'uscita dalla zona di pericolo.

