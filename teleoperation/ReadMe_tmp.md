Il launch file da eseguire è: master_slave_mur620b_mur620d_dual_real.launch

Nei file yaml sono contenuti i parametri comuni hai due master e hai due slave. Nel launch file c'è l'overriding per i parametri specifici del destro e del sinistro

I nomi dai file yaml possono trarre in confusione

Al momento il codice è impostato in modo tale 
Siccome c'è una differenza di montaggio tra gli UR del mur620b e del mur620d viene applicata la rotazione all'interno del launch file. bisogna correggerla se si cambiano i robot.
Potevo farla dinamica? Si, ma visto che usavo solo quei robot ho preferito lasciarla così.

I seguenti launch file sono di utilità per posizionare i robot in posizioni predefinite:
- mur620b_mur620d_moveit_home_dual_arm.launch
- mur620b_mur620d_moveit_teleop_home_dual_arm.launch

Questo secondo launch file esegue moveit verso una posa che è definita solo in questo branch perché l'ho aggiunta al file srdf. Tale posa è uguale sia per master che per slave, ma siccome c'è una differenza di montaggio, all'avvio del launch di teleoperazione questi non coincideranno, ma si muoveranno entrambi verso una posa comune.

I robot sono sensibili solo alle celle di carico, non potete applicare forza lungo la catena cinematica del robot.

Per registrare ed eseguire il replay:
- Eseguire il nodo di teleoperazione
- lanciare start_rosbag_dual_real_run_snapshot.sh
- eseguire la movimentazione e killare lo script sh
- eseguire rosrun su analyze_dual_slave_replay_window.py, da questo si possono definire un po' di parametri per l'inizio e la fine della registrazione
- eseguire replay_dual_slave_twist_from_bag.launch per il replay della registrazione

Il  nodo di teleoperazione usa il twist controller. Se i limiti di quest'ultimo sono troppo bassi vi taglia il comando e il robot non si muove come avreste pensato