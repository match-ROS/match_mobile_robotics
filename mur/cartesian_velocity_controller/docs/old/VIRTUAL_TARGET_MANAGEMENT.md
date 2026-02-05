# Analisi e Soluzione: Gestione Stabilità e Overshoot del Target Virtuale

## 1. Risposta all'Osservazione sull'Overshoot

Hai notato che il **punto virtuale (target)** tende ad andare oltre il waypoint desiderato ("overshoot") o a posizionarsi in modo errato, anche se il robot alla fine raggiunge la posizione corretta. Hai ipotizzato che imporre una velocità massima potrebbe risolvere il problema.

### Analisi Tecnica
La tua osservazione è corretta ed è un sintomo dello stesso fenomeno di "Windup" discusso in precedenza. Ecco perché accade:
1.  **Generazione Velocità**: Il `LocalPlanner` calcola la velocità attrattiva basandosi sull'errore del **robot** ($P_{waypoint} - P_{robot}$), non sull'errore del punto virtuale.
2.  **Integrazione Cieca**: Finché il robot non è arrivato al waypoint, la velocità generata è diversa da zero. Il punto virtuale ($P_{virtual}$) integra questa velocità e continua a muoversi.
3.  **Risultato**: Se il robot è in ritardo, il punto virtuale continua ad avanzare, superando il waypoint ($P_{waypoint}$) perché la velocità che lo spinge non si azzera quando *lui* arriva, ma solo quando *il robot* arriva.

**Riguardo alla Velocità Massima:**
Il sistema ha già un limite di velocità massima (`max_linear_velocity`). Tuttavia, questo limita solo *quanto velocemente* avviene l'overshoot, non *se* avviene. Limitare la velocità non risolve il fatto che il punto virtuale non sa quando fermarsi se il robot è indietro.

### Conclusione sulla Soluzione Proposta
La soluzione **Hybrid Soft Leash** (Guinzaglio Virtuale + Reset) proposta **risolve anche questo problema**.
*   Vincolando la distanza massima tra $P_{virtual}$ e $P_{robot}$, impediamo fisicamente al punto virtuale di scappare via.
*   Quando il punto virtuale cerca di superare il waypoint mentre il robot è ancora indietro, la distanza $P_{virtual} - P_{robot}$ aumenta.
*   Il "guinzaglio" entra in azione, riducendo la velocità del punto virtuale a zero (o quasi), costringendolo ad "aspettare" il robot.
*   Risultato: Il punto virtuale non farà più overshoot significativi ma si fermerà poco davanti al robot lungo la traiettoria ideale.

---

## 2. Piano di Implementazione Dettagliato

Approvo l'approccio ibrido. Per un'implementazione pulita e robusta, la logica va inserita direttamente nel **`LocalPlanner`**, poiché è lui che gestisce lo stato `target_raw_`.

### A. Nuovi Parametri
Da aggiungere a `ControllerTuning.cfg` (dynamic reconfigure) e `controller_params.yaml`:

1.  `virtual_target_leash_enabled` (bool, default: true): Abilitazione funzione.
2.  `virtual_target_leash_start` (double, default: 0.10 m): Distanza Robot-TargetVirtuale a cui inizia il rallentamento.
3.  `virtual_target_leash_stop` (double, default: 0.20 m): Distanza a cui il TargetVirtuale si ferma completamente.
4.  `virtual_target_reset_threshold` (double, default: 0.50 m): Distanza di sicurezza per Hard Reset.

### B. Modifiche a `LocalPlanner`

#### 1. Aggiornamento `LocalPlanner::compute`
Prima di integrare la posizione (`integrateTarget`) e prima di restituire l'output, applichiamo lo scaling.

```cpp
// ... Calcolo v_combined_linear e v_combined_angular come ora ...

// --- INIZIO LOGICA SOFT LEASH ---
double scaling_factor = 1.0;
bool perform_reset = false;

if (leash_enabled_) {
    // Calcolo distanza attuale tra Target Virtuale e Robot
    // Nota: Usiamo target_raw_ (stato interno) vs current_pose
    double dist_virtual_robot = (target_raw_.translation() - current_pose.translation()).norm();
    
    // 1. Controllo Hard Reset (Safety)
    if (dist_virtual_robot > reset_threshold_) {
        perform_reset = true;
    } 
    // 2. Calcolo Soft Scaling
    else if (dist_virtual_robot > leash_start_dist_) {
        // Mappatura lineare inversa: 
        // start -> scale = 1.0
        // stop  -> scale = 0.0
        double denominator = std::max(1e-4, leash_stop_dist_ - leash_start_dist_);
        double ratio = (dist_virtual_robot - leash_start_dist_) / denominator;
        scaling_factor = std::clamp(1.0 - ratio, 0.0, 1.0);
    }
}

if (perform_reset) {
    // Hard Reset: Il target torna sul robot
    resetToPosition(current_pose);
    
    // Azzera velocità per questo ciclo per evitare scatti
    v_combined_linear.setZero();
    v_combined_angular.setZero();
    output.combined_linear.setZero();
    output.combined_angular.setZero();
    // Anche le componenti attrattive/repulsive in output dovrebbero essere azzerate o lasciate come debug?
    // Meglio azzerare l'output di comando effettivo.
} else {
    // Applica Soft Leash
    if (scaling_factor < 1.0) {
        v_combined_linear *= scaling_factor;
        v_combined_angular *= scaling_factor; // Scaliamo anche l'angolare per coerenza
        
        // Aggiorna anche l'output verso il PID/Filtro
        output.combined_linear = v_combined_linear;
        output.combined_angular = v_combined_angular;
    }
    
    // Integrazione normale (ora con velocità scalata)
    integrateTarget(v_combined_linear, v_combined_angular, dt);
}
// --- FINE LOGICA SOFT LEASH ---
```

### C. Modifiche a `CartesianVelocityController`

1.  **Caricamento Parametri**: In `initializeComponents` e `loadParameters` leggere i nuovi parametri da YAML.
2.  **Configurazione LocalPlanner**: Passare i parametri al `local_planner_`.
3.  **Dynamic Reconfigure**: Mappare i parametri nel callback `dynamicReconfigureCallback`.

### D. Punti Aperti e Note

1.  **Rotazione**: Attualmente la logica di leash considera solo la distanza euclidea (traslazione). È sufficiente per la maggior parte dei casi. Se il robot ruota molto sul posto, potremmo aver bisogno di un leash anche sull'orientamento, ma inizierei solo con la posizione.
2.  **Visualizzazione**: Sarebbe utile visualizzare in Rviz quando il leash è attivo. Possiamo aggiungere un campo `scaling_factor` a `LocalPlannerOutput` e pubblicarlo nei messaggi di debug o cambiare colore al marker del target virtuale (es. diventa giallo quando rallenta, rosso se resetta). **Risposta**: Pubblica un messaggio sul debug, scaling_factor va bene.
3.  **Overshoot Residuo**: Con il soft leash, un piccolissimo overshoot (pari a `leash_start_dist`) è ancora possibile se il robot è fermo esattamente sul waypoint e il target virtuale è `leash_start_dist` avanti. Tuttavia, poiché $V_{att} \to 0$ quando Robot $\to$ Waypoint, questo non dovrebbe accadere in pratica.

## 3. Checklist Operativa

1.  [ ] Modificare `LocalPlanner.hpp`: aggiungere membri per i parametri leash e metodi setter/getter.
2.  [ ] Modificare `LocalPlanner.cpp`: implementare la logica in `compute`.
3.  [ ] Modificare `ControllerTuning.cfg`: aggiungere i 4 parametri.
4.  [ ] Modificare `CartesianVelocityController.cpp`: wiring dei parametri.
5.  [ ] Test: Verificare che bloccando il robot in simulazione, il target virtuale si fermi a `leash_stop_dist`.
