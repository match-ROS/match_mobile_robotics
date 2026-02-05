# Piano di Correzione Visualizzazione Markers

Questo documento dettaglia le azioni necessarie per risolvere le anomalie rilevate nel sistema di visualizzazione del `cartesian_velocity_controller`.

## 1. Visualizzazione `v_link_linear` (Freccia Gialla)

**Problema:** La componente cartesiana della repulsione generata dai link (`v_link_linear`) è passata come `Vector3d::Zero()` al publisher, rendendo invisibile il contributo dei link sul TCP.

**Analisi Tecnica:**
La repulsione dei link viene calcolata direttamente nello spazio dei giunti (`q_dot_rep`) dal `LocalPlanner`. Tuttavia, per visualizzarla come "freccia gialla" al TCP (insieme a `v_goal` e `v_obs`), dobbiamo proiettare questa velocità dei giunti nello spazio cartesiano del TCP.

**Piano di Implementazione:**
Nel file `cartesian_velocity_controller.cpp`, all'interno del loop principale (prima della chiamata a `publishVelocityMarkers`):

1.  Recuperare `local_output.repulsive_links_joint` (che contiene $\dot{q}_{rep}$).
2.  Calcolare la velocità cartesiana risultante al TCP usando lo Jacobiano corrente del robot:
    $$ v_{link\_tcp} = J_{tcp} \cdot \dot{q}_{rep} $$
3.  Passare il vettore risultante (solo parte lineare) a `publishVelocityMarkers`.

```cpp
// Esempio pseudo-codice modifica
Eigen::VectorXd q_dot_rep = local_output.repulsive_links_joint;
Eigen::MatrixXd J_tcp = robot_state_->getJacobian(); // Jacobiano al TCP
Eigen::Vector6d v_rep_link_cartesian = J_tcp * q_dot_rep;

marker_publisher_->publishVelocityMarkers(..., v_rep_link_cartesian.head<3>());
```

---

## 2. Visualizzazione Vettori Direzione Distanza (`*_dir`)

**Problema:** I namespace `repulsion_tcp_dir` e `repulsion_link_dir` vengono puliti ma mai popolati. Manca l'indicazione visiva della direzione verso l'ostacolo.

**Analisi Tecnica:**
*   **TCP Obstacles (`ObstacleInfo`):** Contiene `position` (punto sull'ostacolo) e `distance_vector` (vettore da ostacolo a TCP).
*   **Link POIs (`LinkPOI`):** Contiene `position_world` (punto sul robot) e `distance_vector` (vettore da POI a ostacolo).

**Piano di Implementazione:**
Nel file `marker_publisher.cpp`, metodo `publishRepulsionMarkers`:

1.  Aggiungere creazione di marker tipo `LINE_LIST` o `ARROW` sottili.
2.  **Per TCP (`repulsion_tcp_dir`):**
    *   Start: `obs.position - obs.distance_vector` (Posizione Robot)
    *   End: `obs.position` (Posizione Ostacolo)
3.  **Per Link (`repulsion_link_dir`):**
    *   Start: `poi.position_world` (Posizione Robot)
    *   End: `poi.position_world + poi.distance_vector` (Posizione Ostacolo)
4.  Colore suggerito: Grigio o Bianco semi-trasparente per indicare la "linea di vista" del sensore di distanza.

---

## 3. Visualizzazione Punti Ostacolo

**Problema:** Si vede il punto sul robot che "soffre", ma non si vede *cosa* sta evitando (il punto corrispondente sull'ostacolo).

**Piano di Implementazione:**
Estendere la logica del punto 2 per aggiungere una sfera anche sul punto finale (lato ostacolo).

1.  Creare un marker sferico (piccolo, raggio es. 2-3 cm) nel punto "End" calcolato sopra.
2.  Colore: Rosso Scuro o Grigio Scuro.
3.  Questo aiuta a capire se il sistema sta rilevando l'ostacolo nella posizione corretta (es. debugging della percezione/filtro punti).

---

## Riepilogo Modifiche File

### `cartesian_velocity_controller.cpp`
*   Calcolo di `v_link_linear` tramite proiezione Jacobiana.
*   Aggiornamento chiamata a `publishVelocityMarkers`.

### `components/marker_publisher.cpp`
*   Implementazione logica per `repulsion_tcp_dir` (Line + Sphere Obstacle).
*   Implementazione logica per `repulsion_link_dir` (Line + Sphere Obstacle).

