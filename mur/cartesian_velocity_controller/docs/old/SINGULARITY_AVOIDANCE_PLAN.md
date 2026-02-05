# Piano di Implementazione: Evitamento Singolarità tramite Pesi Dinamici

## 1. Analisi del Problema
Il manipolatore UR10e incontra singolarità al gomito (estensione completa, $q_{elbow} \approx 0$). In questa configurazione, la matrice Jacobiana perde rango, causando velocità articolari elevate o nulle (blocco) con i metodi standard (Damped Least Squares).
La soluzione proposta è il **Dynamic Weighting**: penalizzare il movimento del giunto "critico" (gomito) nella funzione di costo dell'IK quando si avvicina alla singolarità.

## 2. Architettura Proposta (Ottica "Future-Proof")
Per soddisfare la richiesta di futura estensibilità (modifica dinamica dei pesi per altri compiti), sconsiglio di inserire la logica "hardcoded" nel `CartesianVelocityController`.

Propongo invece la creazione di un nuovo componente dedicato: **`JointWeightManager`**.

### Responsabilità del `JointWeightManager`
1.  Mantenere il vettore dei pesi articolari corrente.
2.  Gestire diverse "strategie" di pesatura (es. `SingularityAvoidance`, `JointLimitAvoidance`, `PreferredConfiguration`).
3.  Fornire un'interfaccia unificata per calcolare i pesi finali da passare al `JacobianSolver`.

### Struttura delle Classi suggerita
```cpp
// Interfaccia base per strategie di pesatura
class WeightStrategy {
public:
    virtual void computeWeights(const Eigen::VectorXd& joint_positions, 
                              Eigen::VectorXd& weights) = 0;
};

// Implementazione specifica per singolarità gomito
class ElbowSingularityStrategy : public WeightStrategy {
    int elbow_index_;
    double buffer_zone_;
    double max_weight_;
public:
    void computeWeights(...) override;
};

// Manager che aggrega le strategie
class JointWeightManager {
    std::vector<std::shared_ptr<WeightStrategy>> strategies_;
    Eigen::VectorXd current_weights_;
public:
    void update(const Eigen::VectorXd& joint_positions);
    const Eigen::VectorXd& getWeights() const;
};
```

## 3. Piano di Implementazione Dettagliato

### Fase 1: Creazione Componente `JointWeightManager`
1.  Creare `cartesian_velocity_controller/include/cartesian_velocity_controller/components/joint_weight_manager.hpp` e `.cpp`.
2.  Implementare la logica di calcolo del peso per la singolarità:
    *   **Formula Smooth**: Invece di un "if" secco che potrebbe causare discontinuità nella derivata (jerk), usare una funzione liscia (es. polinomio cubico o funzione esponenziale) che va da 1.0 a `MAX_WEIGHT` man mano che ci si avvicina a 0.
    *   Formula suggerita: $w = 1 + K_{max} \cdot \max(0, 1 - \frac{|q - q_{sing}|}{q_{buffer}})^2$

### Fase 2: Integrazione in `CartesianVelocityController`
1.  Includere `joint_weight_manager.hpp`.
2.  Aggiungere `std::unique_ptr<JointWeightManager> weight_manager_;` ai membri privati.
3.  In `initializeComponents()`, istanziare il manager e caricare i parametri (indice gomito, soglie).
4.  In `executePipeline()`:
    ```cpp
    // 1. Get current joint positions
    Eigen::VectorXd current_joints;
    robot_state_->getCurrentJointPositions(current_joints);
    
    // 2. Update weights
    weight_manager_->update(current_joints);
    
    // 3. Pass weights to JacobianSolver
    Eigen::VectorXd weights = weight_manager_->getWeights();
    jacobian_solver_->computeDampedWeightedPseudoInverse(jacobian, weights);
    ```

### Fase 3: Configurazione
1.  Aggiornare `ControllerTuning.cfg` per permettere il tuning dinamico di `singularity_buffer` e `max_weight_penalty`.
2.  Aggiornare `controller_params.yaml` con i default.

## 4. Dubbi e Criticità Riscontrate

Durante l'analisi ho individuato i seguenti punti critici che dobbiamo affrontare:

1.  **Discontinuità della Velocità**:
    *   *Problema*: Se i pesi cambiano bruscamente (es. gradino), la soluzione dell'IK salterà, causando picchi di accelerazione/jerk.
    *   *Soluzione*: È **fondamentale** che la funzione di peso sia continua e derivabile ($C^1$ o $C^2$). La formula quadratica o cubica suggerita sopra risolve questo problema.

2.  **Conflitto con il Movimento Desiderato**:
    *   *Problema*: Se il target richiede di stendere il braccio (es. prendere un oggetto lontano), il peso alto combatterà contro il movimento. Il robot rallenterà molto o si fermerà prima di raggiungere il target.
    *   *Mitigazione*: Bisogna accettare questo compromesso. È una sicurezza. Tuttavia, se il peso è *troppo* alto, il robot potrebbe bloccarsi "troppo presto". Il parametro `max_weight` va tunato attentamente.

3.  **Debug e Visualizzazione**:
    *   Attualmente non vediamo i pesi. Sarà difficile capire se il sistema sta agendo.
    *   *Azione*: Aggiungere i pesi correnti al messaggio di debug `PipelineDebug` o pubblicarli su un topic separato (es. `/debug/joint_weights`).
    *   *Risposta*: Aggiungi i pesi al messaggio di debug

4.  **Interazione con `JacobianSolver`**:
    *   Il `JacobianSolver` attuale calcola $W^{-1/2}$. Se un peso è molto grande, $W^{-1/2}$ è molto piccolo. La colonna dello Jacobiano viene scalata quasi a zero. La SVD vedrà quel giunto come "inutile" e non lo userà. Questo è il comportamento corretto desiderato.

5.  **Definizione di "Gomito"**:
    *   L'indice del giunto varia a seconda del robot. Per UR10e standard è solitamente l'indice 2 (spalla_pan, spalla_lift, **gomito**, polso1, polso2, polso3). Questo indice deve essere parametrizzabile.

## 5. Prossimi Passi Operativi

Se approvi questo piano, procederò in questo ordine:
1.  Creerò i file per `JointWeightManager` (header e source).
2.  Implementerò la logica "Smooth Weighting".
3.  Modificherò `CartesianVelocityController` per integrare il manager.
4.  Aggiungerò i parametri al sistema di configurazione.
5.  Aggiungerò output di debug per monitorare i pesi in tempo reale.

