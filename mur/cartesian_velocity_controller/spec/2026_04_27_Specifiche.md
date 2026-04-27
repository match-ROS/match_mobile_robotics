contesto:
il pacchetto qui presente è stato sviluppato per un obstacle avoidance di un robot formato da una base mobile mir600 e da due bracci ur10e. Al momento vorrei integrare delle modifiche per simulare con il robot reale un processo di stampa 3D. Per tale processo è necessaria la base e un solo braccio ur10e.

Il processo di stampa che dovrei simulare:
L'idea è di montare un ugello sul polso del robot che sia rivolto sempre verticalmente e che deve seguire un percorso planare ad una certa quota, che tendenzialmente è 2D, ma potrebbe variare la sua quota z lungo il tracciato.

Cosa vorrei fare io:
- Montare un tool fittizio sul polso del robot
- Definire un percorso che il tcp deve seguire
- Il robot intero si deve muovere in sinergia affinché il tcp segua il percorso preimpostato a velocità costante, e nel mentre l'intera struttura sia in grado di riorientarsi ed eseguire i movimenti necessari per supportare il movimento del tcp
- L'idea alla base di tutto è avere un sistema di obstacle avoidance che permetta alla struttura di avitare gli ostacoli proseguendo comunque con il task primario, ovvero che il tcp deve seguire la propria traiettoria.

Struttura del robot:
- braccio ur10e con 6 gradi di libertà
- Lifter che fa da settimo asse (solleva la base dell'ur), che però può essere controllato solo in posizione e non in velocità, tale per cui quando si movimenta questo asse è necessario che il resto della catena vada un po' dietro a quella che è la sua dinamica e non viceversa.
- Base mobile mir600 con sterzo differenziale

Rilevamento ostacoli:
- Uso dei laserscanner del mir per ostacoli bassi che minacciano la base
- Ostacoli simulati che entreranno nella griglia per la parte alta del robot (non ho telecamere a disposizione)

Cosa mi serve:
- Integrare i laser scanner nel mio obstacle avoidance
- Avere un sistema wholebody control per muovere tutto il robot insieme
- Avere un sistema per definire la traiettoria che il tcp deve percorrere.
- Avere un sistema che mi permetta di far percorrere al tcp tale traiettoria
- Integrare l'obstacle avoidance nel movimento del robot
- Avere un sistema che sia in grado di compensare la dinamica del lifter

Robot:
- il robot che userò è il mur620d, trovi il launch file, urdf e altro nella cartella