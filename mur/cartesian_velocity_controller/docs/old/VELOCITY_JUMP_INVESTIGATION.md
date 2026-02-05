# Report Investigativo: Salto di Velocità e Spike di Accelerazione

## 1. Analisi del Problema

### Sintomi Osservati
*   **Grafico:** Si osserva un "salto netto" (discontinuità) nel comando di velocità dei giunti, seguito da un picco (spike) nell'accelerazione stimata.
*   **Comportamento:** Il robot sembra subire una brusca frenata o un cambio repentino di moto quando viene inviato un nuovo target o modificato il percorso.
*   **Causa Immediata:** Il `JointSafetyLimiter` rileva una violazione dei limiti di accelerazione a causa della discontinuità nella velocità comandata e interviene drasticamente.

### Analisi del Codice

L'investigazione del codice sorgente ha rivelato che il problema risiede nella gestione del **reset dello stato interno** del controller quando viene assegnato un nuovo target (`setTargetPose`).

La catena di eventi è la seguente:

1.  Il robot è in movimento con una velocità $V_{robot} \neq 0$.
2.  Viene chiamato `setTargetPose` (o `setWaypoints`).
3.  Questo invoca internamente `resetVirtualTargetsToCurrentPose()`.
4.  La funzione reimposta i componenti interni per "agganciarli" alla posizione corrente del robot:
    ```cpp
    local_planner_->resetToPosition(current_tcp_pose);
    velocity_filter_->resetToPosition(current_tcp_pose); // <--- PUNTO CRITICO
    resetPIDControllers();
    ```
5.  **Il Problema:** Il metodo `velocity_filter_->resetToPosition(...)` azzera completamente lo stato dinamico del filtro (Velocità = 0, Accelerazione = 0).
6.  Al ciclo di controllo successivo (`executePipeline`):
    *   Il `LocalPlanner` calcola una nuova velocità desiderata (probabilmente non nulla se il nuovo target è lontano).
    *   Il `VelocityFilter`, partendo da zero, genera una rampa di velocità che inizia da valori molto bassi (vicini a 0).
    *   Il comando finale inviato ai giunti ($V_{cmd}$) crolla improvvisamente da $V_{robot}$ a $\approx 0$.
7.  Il `JointSafetyLimiter` confronta $V_{cmd}$ (≈0) con $V_{prev}$ ($V_{robot}$).
    *   Calcola l'accelerazione richiesta: $A_{req} \approx (0 - V_{robot}) / \Delta t$.
    *   Questa accelerazione è enorme e viola i limiti.
    *   Il limiter interviene scalando la velocità, ma poiché il comando è quasi zero, la capacità di controllo è compromessa ("frenata a 0 istantanea").

## 2. Soluzione Implementata

Per risolvere il problema, è stato necessario garantire la **continuità della velocità** durante il reset del controller.

### Modifiche Apportate

1.  **Estensione `CartesianVelocityFilter`**:
    È stato aggiunto il metodo `resetToState(...)` che permette di inizializzare il filtro non solo con la posizione, ma anche con la velocità corrente (lineare e angolare).
    
    *File: `velocity_filter.hpp` / `velocity_filter.cpp`*
    ```cpp
    void resetToState(const Eigen::Isometry3d& position,
                      const Eigen::Vector3d& linear_velocity,
                      const Eigen::Vector3d& angular_velocity);
    ```

2.  **Aggiornamento `CartesianVelocityController`**:
    La funzione `resetVirtualTargetsToCurrentPose()` è stata modificata per:
    *   Calcolare la velocità Cartesiana corrente del robot usando lo Jacobiano e le velocità dei giunti ($V = J \dot{q}$).
    *   Inizializzare il `VelocityFilter` con questa velocità invece che con zero.

    *File: `cartesian_velocity_controller.cpp`*
    ```cpp
    // Calcolo velocità corrente
    Eigen::VectorXd cart_vel = jacobian * joint_vels;
    
    // Inizializzazione con continuità
    if (velocity_filter_)
    {
      velocity_filter_->resetToState(current_tcp_pose, cart_vel.head<3>(), cart_vel.tail<3>());
    }
    ```

### Risultato Atteso

Con questa modifica, quando viene assegnato un nuovo target:
1.  Il filtro viene inizializzato alla velocità attuale del robot ($V_{filter} = V_{robot}$).
2.  Se il nuovo target richiede un cambio di direzione o stop, il filtro genererà una traiettoria che parte da $V_{robot}$ e decelera dolcemente rispettando i limiti di accelerazione e jerk configurati.
3.  Non ci sarà più il salto a zero nel comando.
4.  Il `JointSafetyLimiter` non vedrà più picchi di accelerazione spuri.

## 3. Raccomandazioni Ulteriori

Sebbene la soluzione software risolva la causa principale, si consiglia di verificare anche:

*   **Frequenza di Controllo:** Assicurarsi che il loop giri stabilmente (es. 100Hz o 500Hz). Jitter elevati nel $\Delta t$ possono comunque causare rumore nell'accelerazione calcolata ($A \approx \Delta V / \Delta t$).
*   **Limiti di Sicurezza:** I limiti in `joint_safety_limiter` (configurati in `controller_params.yaml`) devono essere fisicamente raggiungibili dal robot. Se sono troppo restrittivi rispetto alla dinamica reale, il limiter potrebbe intervenire anche durante movimenti normali.

