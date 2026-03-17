# Wrench: trasformazione completa vs sola rotazione

## Contesto

Nel nodo `teleop_master_haptic_controller_node.cpp` il wrench viene trasformato dal frame sorgente al frame target ruotando separatamente:

- la forza `F`
- la coppia `tau`

In pratica oggi fai questo:

```cpp
F_target = R * F_source
tau_target = R * tau_source
```

Questa operazione cambia solo il sistema di riferimento angolare del vettore, ma non cambia il punto rispetto a cui la coppia e calcolata.

Nel codice c'e anche un commento esplicito:

```cpp
// NOTE: ... wrenchMsgToWrench3 rotates vectors but does not apply p x f.
```

Quindi la tua implementazione attuale non esegue la trasformazione completa di un wrench tra due frame rigidi con origine diversa.

## Punto chiave

Un wrench non e solo "forza + coppia":

- `F` e una forza
- `tau` e il momento di quella forza rispetto a una certa origine

Se cambi solo orientazione del frame, basta ruotare i due vettori.

Se invece cambi anche origine del frame, la coppia deve cambiare anche per effetto del braccio:

```text
tau_new = R * tau_old + r x F_new
```

dove:

- `R` e la rotazione tra i frame
- `F_new = R * F_old`
- `r` e il vettore che va dalla nuova origine alla vecchia origine, espresso nel frame nuovo
- `r x F_new` e il momento addizionale generato dalla forza quando la osservi da un altro punto

Nota: se definisci `r` nel verso opposto, cambia il segno del termine di prodotto vettoriale. La fisica non cambia: cambia solo la convenzione.

## Cosa significa fisicamente

La forza e un vettore "libero" dal punto di vista dell'espressione del frame:

```text
F_new = R * F_old
```

La coppia invece non e un vettore indipendente dal punto. Se sposti il polo di riduzione del wrench, una stessa forza produce un momento diverso.

Esempio semplice:

- hai una forza di 10 N lungo `y`
- il sensore e a 0.1 m dall'origine del frame target lungo `x`

Allora compare una coppia aggiuntiva:

```text
tau = r x F = [0.1, 0, 0] x [0, 10, 0] = [0, 0, 1] Nm
```

Quella coppia non arriva da un sensore che "misura torsione vera": nasce solo dal fatto che stai osservando la stessa forza da un'origine diversa.

## Differenza tra i due casi

### 1. Sola rotazione

Formula:

```text
F_new = R * F_old
tau_new = R * tau_old
```

Assunzione implicita:

- la coppia e riferita allo stesso punto fisico
- i due frame hanno origine coincidente, oppure il termine dovuto al braccio e trascurabile

Questa trasformazione e corretta se:

- cambi solo orientazione del frame
- oppure il wrench e gia espresso nel punto rispetto a cui ti interessa lavorare

### 2. Trasformazione completa del wrench

Formula:

```text
F_new = R * F_old
tau_new = R * tau_old + r x F_new
```

Questa trasformazione e necessaria se:

- il frame sorgente e il frame target non hanno la stessa origine
- vuoi esprimere il wrench rispetto a un altro punto del robot, per esempio dalla tool frame alla base
- vuoi che la componente rotazionale includa anche il momento generato dalle forze lineari

## Conseguenza pratica sulla parte rotazionale

Questa e la differenza piu importante:

- con la sola rotazione, la parte angolare dipende solo dalla coppia misurata dal sensore
- con la trasformazione completa, la parte angolare dipende sia dalla coppia misurata sia dalla forza misurata tramite il termine `r x F`

Quindi, con la tua implementazione attuale, possono succedere questi casi:

### Caso A: forza grande, coppia misurata quasi zero

Con la tua trasformazione attuale:

```text
tau_target ~ R * tau_source ~ 0
```

Con trasformazione completa:

```text
tau_target ~ r x F_target
```

Quindi potresti vedere:

- raw torque piccolo o nullo
- torque filtrato piccolo o nullo
- ma fisicamente ci sarebbe una coppia equivalente non nulla nel frame target

### Caso B: il contatto e eccentricamente rispetto al frame target

Se il contatto genera soprattutto forza lineare ma con un braccio rispetto alla base o al frame di controllo, la trasformazione completa produce una coppia apparente nel frame target.

Con la sola rotazione, quella coppia non compare.

Risultato:

- la parte rotazionale dell'ammettenza viene sottoeccitata
- il controller "vede" meno torque di quello che meccanicamente sarebbe coerente rispetto al frame target

## Impatto sull'ammettenza

Nel tuo controller la dinamica angolare usa un termine del tipo:

```text
M_ang * domega + D_ang * omega = tau_hand - tau_feedback
```

Se `tau_feedback` e ottenuto con sola rotazione:

- entra solo la coppia misurata dal sensore, ruotata
- non entra il contributo dovuto alle forze applicate lontano dall'origine del frame target

Se `tau_feedback` e ottenuto con trasformazione completa:

- entra la coppia misurata ruotata
- entra anche il momento equivalente prodotto dalla forza per effetto del braccio

In generale questo rende la parte rotazionale:

- piu coerente con la meccanica reale
- piu accoppiata alla traslazione
- piu sensibile a contatti laterali o eccentrici

## Quando la tua implementazione va bene

La sola rotazione e ragionevole se vale almeno una di queste condizioni:

- il frame target ha praticamente la stessa origine del frame sensore
- il braccio tra i due frame e molto piccolo
- ti interessa filtrare solo la coppia "misurata direttamente", senza convertire le forze in momento equivalente
- vuoi volutamente separare traslazione e rotazione nel controller

In questi casi il termine `r x F` puo essere piccolo o indesiderato.

## Quando invece manca qualcosa

La tua implementazione e fisicamente incompleta se:

- prendi un wrench misurato in `tool0`
- lo vuoi esprimere in `base_link`, `base_link_inertia` o in un altro frame con origine diversa
- poi usi la componente `tau` per guidare un'ammettenza rotazionale

In quel caso stai facendo una trasformazione di orientazione, ma non una vera trasformazione di wrench.

## Effetto qualitativo sul comportamento che osservi

Se la parte rotazionale filtrata resta circa a zero ma il robot si muove comunque, ci sono due possibilita principali:

1. il moto arriva dal canale cinematico master -> slave (target pose e feedforward twist), non dal wrench
2. la parte torque che ti aspetteresti non compare perche il termine `r x F` non viene aggiunto

In pratica potresti avere rotazioni del sistema dovute a:

- traiettoria del master
- molla virtuale
- errori di orientazione

senza vedere un torque filtrato coerente nel frame target.

## Esempio numerico intuitivo

Supponi:

- forza al TCP: `F = [0, 20, 0] N`
- coppia al TCP: `tau = [0, 0, 0] Nm`
- il frame target e 0.15 m "dietro" lungo `x`

Allora:

### Con sola rotazione

```text
tau_target = 0
```

### Con trasformazione completa

```text
r = [0.15, 0, 0]
tau_target = r x F = [0, 0, 3] Nm
```

Quindi una forza puramente lineare diventa anche una sollecitazione rotazionale quando cambi punto di riferimento.

## In una frase

La "sola rotazione" cambia come esprimi i vettori.

La "trasformazione completa del wrench" cambia sia come esprimi i vettori sia il punto rispetto a cui il momento e definito.

Per la forza lineare la differenza non c'e quasi mai.
Per la coppia rotazionale la differenza puo essere molto grande.

## Pseudocodice della trasformazione completa

```cpp
Eigen::Vector3d F_target = R * F_source;
Eigen::Vector3d tau_target = R * tau_source + r.cross(F_target);
```

con:

- `R`: rotazione source -> target
- `r`: vettore dalla nuova origine alla vecchia origine, espresso nel target

## Collegamento diretto al tuo codice

Nel file `src/teleop_master_haptic_controller_node.cpp` la funzione `wrenchMsgToWrench3()`:

- ruota la forza
- ruota la coppia
- non aggiunge il termine di braccio

Lo stesso schema compare anche nel nodo slave `teleop_slave_twist_outer_loop_node.cpp`.

Quindi oggi il sistema implementa:

- una trasformazione vettoriale corretta
- ma non una trasformazione completa del wrench tra punti diversi

## Conclusione pratica

Se vuoi che la parte rotazionale dell'ammettenza sia "coerente" con la traslazione quando esprimi il wrench in un frame con origine diversa dal sensore, allora devi usare la trasformazione completa del wrench.

Se invece vuoi che la rotazione reagisca solo alla coppia direttamente misurata dal sensore, allora la tua implementazione attuale e una scelta semplificata ma intenzionale.

La vera differenza e questa:

- implementazione attuale: rotazione pura dei vettori
- implementazione completa: rotazione dei vettori + trasporto del momento tramite `r x F`
