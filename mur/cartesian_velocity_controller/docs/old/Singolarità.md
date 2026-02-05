# Soluzione: Evitamento Singolarità tramite Pesi Dinamici

Questo documento descrive la strategia di **Dynamic Weighting** per mitigare il problema del "blocco" del manipolatore UR10e in configurazione di singolarità al gomito (braccio completamente esteso, $q_{elbow} \approx 0$).

## Il Problema

Il `JacobianSolver` attuale utilizza la *Damped Least Squares (DLS)* tramite SVD. Quando il robot si avvicina alla completa estensione:
1.  Il valore singolare minimo ($\sigma_{min}$) crolla verso zero.
2.  Il fattore di damping ($\lambda^2$) aumenta drasticamente per mantenere la stabilità numerica.
3.  **Risultato:** Le velocità calcolate ($\dot{q}$) tendono a zero. Il robot si "congela" e fatica a tornare indietro perché il controller rifiuta di generare velocità in quella configurazione instabile.

## La Soluzione: Pesi Dinamici (Dynamic Weighting)

Invece di affidarsi solo al damping globale, modifichiamo la "funzione di costo" dell'inversa cinematica penalizzando l'uso del giunto specifico (il gomito) quando si avvicina al suo limite pericoloso.

La funzione che il solver minimizza (implicitamente nella SVD pesata) è:

$$
\text{min} \left( \| J \dot{q} - \dot{x} \|^2 + \lambda^2 \| W \dot{q} \|^2 \right)
$$

Dove $W$ è la matrice diagonale dei pesi. Aumentando il peso $W_{elbow}$ vicino alla singolarità, rendiamo "costoso" muovere quel giunto. L'algoritmo preferirà:
* Usare altri giunti (se ridondante).
* Accettare un errore di tracking cartesiano piuttosto che forzare il gomito verso l'estensione completa.

### Vantaggi
* **Non invasivo:** Sfrutta il metodo `computeDampedWeightedPseudoInverse` già esistente in `jacobian_solver.cpp`.
* **Reversibile:** Poiché penalizza solo l'avvicinamento al limite, permette al robot di muoversi "via" dalla singolarità (poiché allontanandosi il peso diminuisce).

---

## Implementazione

La logica va inserita nel loop di controllo principale (es. `CartesianVelocityController`), **prima** di chiamare il solver.

### 1. Definizione Parametri

Aggiungi questi parametri (hardcoded o via YAML):

```cpp
// Indice del gomito per UR10e (spalla_pan, spalla_lift, gomito, ...)
const int ELBOW_JOINT_IDX = 2; 

// Soglia di attivazione (es. 0.2 rad ≈ 11.5 gradi)
// Inizia a pesare il giunto quando è a meno di 0.2 rad dall'estensione
const double SINGULARITY_BUFFER = 0.2; 

// Peso massimo aggiunto (quanto diventa "duro" il giunto)
const double MAX_WEIGHT_PENALTY = 50.0;