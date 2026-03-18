### Master

sistema massa molla smorzatore senza feedback di forza diretto dalla cella di carico dello slave. Ma feedback tramite molla virtuale sulla differenza di posa tra i due.

### Slave

Lo slave viene mosso solamente dalla forza virtuale della molla e ha una compliance.

### Equazioni di Controllo per lo Slave (Cedevolezza/Ammettenza)

Forza virtuale generata dall'errore di posizione (Molla virtuale dello slave):
$$F_{virtuale} = K_s \cdot (X_{master} - X_{slave})$$

Comando di velocità da inviare allo slave (Legge di ammettenza):
$$V_{slave} = \frac{1}{D_s} \cdot (F_{virtuale} - F_{esterna})$$

*Dove:*
* $K_s$: Rigidezza virtuale dello slave.
* $D_s$: Smorzamento virtuale dello slave.
* $F_{esterna}$: Forza di contatto letta dal sensore F/T dello slave (compensata dalla gravità).

---

### Equazioni di Controllo per il Master (Feedback Aptico/Ammettenza)

Forza di feedback inviata dallo slave al master (Molla virtuale del master):
$$F_{feedback} = K_m \cdot (X_{slave} - X_{master})$$

Equazione differenziale della dinamica desiderata (Massa-Molla-Smorzatore):
$$M_m \ddot{X}_{master} + D_m \dot{X}_{master} = F_{umano} + K_m \cdot (X_{slave} - X_{master})$$

Calcolo dell'accelerazione istantanea (per l'integrazione nel nodo ROS):
$$\ddot{X}_{master} = \frac{1}{M_m} \Big( F_{umano} + K_m \cdot (X_{slave} - X_{master}) - D_m \dot{X}_{master} \Big)$$

Integrazione numerica di Eulero per ottenere il comando di velocità da inviare al master:
$$V_{master}(t) = V_{master}(t-\Delta t) + \ddot{X}_{master} \cdot \Delta t$$

Integrazione numerica di Eulero per aggiornare la posizione virtuale del master:
$$X_{master}(t) = X_{master}(t-\Delta t) + V_{master}(t) \cdot \Delta t$$

*Dove:*
* $M_m$: Inerzia (massa) virtuale del master.
* $D_m$: Smorzamento virtuale del master.
* $K_m$: Rigidezza virtuale del master (feedback di forza).
* $F_{umano}$: Forza applicata dall'operatore letta dal sensore F/T del master.
* $\Delta t$: Tempo di campionamento del ciclo di controllo (es. $0.008$ s per 125 Hz).

---

### Formula di Supporto per il Tuning

Calcolo dello smorzamento critico per il master (per evitare oscillazioni e garantire stabilità):
$$D_m \approx 2\sqrt{M_m K_m}$$


### Altre note

Nel codice già scritto del precedente pacchetto ci sono molti parametri che io vorrei un po' semplificare nel nuovo.
Quello che vorrei mantenere sono le seguenti cose:
* Il sistema di overrinding del tf quando viene inviata la posa. I robot master e slave sono identici, quindi non serve una trasformazione esplicità, è sufficiente sovrascrivere il frame.
* Il robot viene controllato tramite twist di velocità che viene espresso nel frame della base, mentre forze e momenti sono espressi nel frame dell'end effector, quindi bisogna fare i calcoli in maniera coerente. Non so se è conveniente fare i calcoli nel frame dell'end effector e poi trasformare tutto nel frame della base. In ogni caso la trasformazione della forza e del momento con solo la rotazione funziona e va bene.
* I filtri esponenziali che ci sono vanno bene. Se possibile lascerei il solo parametro in hz
* la deadband la terrei come è stata implementata
* I limiti di velocità accelerazione e jerk
* Il sistema a substep per il dt
* Altri sistemi di sicurezza utili e non eccessivi

Cosa non servirebbe più:
* Massa e smorzamento dinamici
* La parte di riflessione della forza, che ora non viene più fatta
* La possibilità di avere masse e smorzamenti differenti sui singoli assi
* Altri parametri non in linea con la richiesta fatta sopra
* Tutta la parte di coupling che in realtà non viene utilizzata

### Considerazioni

Le equazioni che ti ho fornito non devono essere prese alla lettera, ma sono solo uno spunto di partenza per creare il pacchetto di teleoperazione in versione semplificata, su cui discutere insieme per arrivare ad avere un piano d'azione dettagliato per poter implementare il tutto.