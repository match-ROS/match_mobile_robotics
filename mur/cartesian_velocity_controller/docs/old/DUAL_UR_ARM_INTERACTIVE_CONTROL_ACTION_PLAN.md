# Piano d’azione — controllo interattivo dual arm (MUR620 + 2x UR)

## Obiettivo
Creare un modo **interattivo** per inviare comandi (pose) ai **due bracci UR** del robot lanciato da:
`mur_bringup/launch/single_mur620_gazebo.launch`

Requisiti dichiarati:
- Controllo **indipendente** dei due bracci (sinistro `l`, destro `r`).
- Accettabile anche avere **una UI da terminale per braccio** (2 istanze).
- Mantenere **immutati** i file YAML:
  - `cartesian_velocity_controller/config/controller_params_mur620_ur10_l.yaml`
  - `cartesian_velocity_controller/config/controller_params_mur620_ur10_r.yaml`

Contesto (dal launch):
- Un solo robot con `robot_name=mur620` → namespace `/mur620`.
- MoveIt avviato (`start_moveit=true`) e gruppi pianificazione: `UR_arm_l`, `UR_arm_r`.

---

## Prima di scegliere: cosa dobbiamo verificare (ground truth su ROS)
Queste verifiche servono a evitare assunzioni su nomi di topic/servizi.

- **Controller node name per braccio (velocity)**:
  - atteso: `/mur620/cartesian_velocity_controller_l` e `/mur620/cartesian_velocity_controller_r`
  - topic target pose atteso: `/<controller_node>/target_pose`
- **Controller manager**:
  - servizio atteso: `/mur620/controller_manager/switch_controller`
  - lista controller: `/mur620/controller_manager/list_controllers`
- **Nomi controller da switchare** (da doc già presente):
  - velocity: `joint_group_vel_controller_{l,r}/unsafe`
  - moveit: `UR10_{l,r}/arm_controller`

Comandi rapidi (solo per verifica, non “soluzione”):
- `rosservice call /mur620/controller_manager/list_controllers "{}"`
- `rostopic info /mur620/cartesian_velocity_controller_l/target_pose`
- `rostopic info /mur620/cartesian_velocity_controller_r/target_pose`

Output atteso:
- i controller velocity risultano **loaded** e possono essere portati a **running** tramite switch.
- esiste almeno 1 subscriber sul `target_pose` quando il relativo controller node è attivo.

---

## Opzione A (raccomandata): **2 terminali / 2 istanze** dello script (una per braccio)
### Idea
Eseguire **due istanze** dello stesso script interattivo, una configurata per il braccio sinistro (`l`) e una per il destro (`r`).

Questo approccio è “per bene” perché:
- mantiene la separazione mentale e operativa (ogni terminale = un braccio);
- riduce drasticamente la complessità della UI e degli stati interni;
- evita di dover progettare una “UI multiplex” (selezione braccio, viste doppie, ecc.).

### Come dovrebbe funzionare (UX)
- Terminale A: menù e comandi solo per `UR_arm_l` / `UR10_l`.
- Terminale B: menù e comandi solo per `UR_arm_r` / `UR10_r`.
- Lo switch controller agisce **solo** sul controller del braccio relativo (start/stop della coppia corretta), senza interferire con l’altro braccio.

### Comandi “target” (da ottenere come risultato finale)
Esempi indicativi:
- Braccio sinistro:
  - `rosrun cartesian_velocity_controller robot_interactive_control.py --mur-ns /mur620 --arm l`
- Braccio destro:
  - `rosrun cartesian_velocity_controller robot_interactive_control.py --mur-ns /mur620 --arm r`

### Pro
- **Semplicità**: minima complessità software.
- **Affidabilità**: meno stati condivisi, meno rischio di bug.
- **Scalabilità**: se domani vuoi aggiungere un terzo “controllore” (es. base mobile), non si intreccia.

### Contro / rischi
- Due finestre/terminali da gestire.
- Se le frame TF risultano prefissate (es. `mur620/...`), bisogna verificare e passare i frame corretti per ciascuna istanza.

---

## Opzione B: **UI unica** che controlla entrambi i bracci
### Idea
Un solo programma interattivo con:
- stato “attivo”: braccio `l` o `r`;
- viste di stato per entrambi;
- comandi “send pose” e “switch controller” che agiscono sul braccio selezionato.

### Pro
- Un solo terminale.
- Possibile aggiungere funzioni “sincronizzate” (es. manda pose a entrambi) in modo naturale.

### Contro
- **Più complessa**: più codice UI, più stato, più edge-case.
- Richiede design accurato per evitare confusione (soprattutto nello switch e nello stato controller attivo).

---

## Opzione C: Wrapper che avvia 2 UI in modo comodo (tmux / script)
### Idea
Manteniamo Opzione A (due istanze), ma aggiungiamo un launcher “convenience”:
- avvia automaticamente due processi;
- opzionalmente apre un layout `tmux` (split verticale) o due tab.

### Pro
- Quasi la stessa semplicità di A.
- UX migliore (un solo comando per “setup dual arm control”).

### Contro
- Dipendenza/assunzione su `tmux` o sul tool scelto.

---

## Raccomandazione
Partire con **Opzione A** (2 terminali / 2 istanze), perché massimizza robustezza e minimizza lavoro.

Se in futuro emerge un bisogno reale di:
- macro/comandi sincronizzati,
- un’unica dashboard,
allora si evolve verso **Opzione B**, avendo già “validato” bene i comportamenti per singolo braccio.

---

## Piano operativo (incrementale, con checkpoint)
### Fase 0 — Ricognizione rapida (no refactor)
- Verificare i nomi effettivi di:
  - controller node per braccio,
  - servizi `controller_manager`,
  - controller names loaded/running,
  - TF frame reali (prefissati o no).

Output della fase:
- Tabella con:
  - braccio `l`: `controller_node`, `velocity_controller`, `moveit_controller`, `ee_frame`, `global_frame`
  - braccio `r`: idem

### Fase 1 — “Single-arm correct” (una istanza)
- Assicurare che lo script, lanciato con `--arm l`, invii pose al solo braccio `l`:
  - via MoveIt quando attivo,
  - via `PoseStamped` su `target_pose` quando in velocity mode.
- Ripetere per `--arm r`.

Checkpoint:
- invio pose `l` non muove `r` e viceversa;
- lo switch `l` non spegne controller `r` e viceversa.

### Fase 2 — “Dual terminals” (due istanze contemporanee)
- Lanciare due istanze in parallelo (Opzione A).
- Testare:
  - invio pose alternato (l poi r),
  - invio pose ravvicinato,
  - switch controller su uno mentre l’altro resta invariato.

Checkpoint:
- nessun conflitto di risorse evidente (topic, service, TF).

### Fase 3 — (Opzionale) “Convenience launcher” (Opzione C)
- Aggiungere un comando/documentazione per avviare le due UI in modo rapido (eventuale `tmux`).

---

## Aspetti tecnici da decidere prima dell’implementazione UI unica (se mai)
Se scegliamo Opzione B, dobbiamo chiarire:
- come rappresentare lo stato “controller attivo” per ciascun braccio (potenzialmente indipendente);
- come gestire le impostazioni frame/topic per ciascun braccio;
- come evitare errori umani (es. inviare pose al braccio sbagliato).

---

## Criteri di accettazione (Definition of Done)
- Possibile lanciare 2 istanze:
  - una per `l`, una per `r`,
  - e inviare pose indipendenti con comportamento ripetibile.
- Switching controller per un braccio funziona e non impatta l’altro.
- Documentazione “how-to” con comandi esatti per:
  - avvio singolo braccio,
  - avvio dual (due terminali o launcher).

