# Analisi approfondita: problemi di stabilità e proposta di architettura bilaterale

## Problema 1: Instabilità del master quando lo slave è in contatto

### Cosa succede nel codice attuale

Il master calcola (riga 639, 666):

```cpp
F_feedback = (-kf_force_ * slave_filt_.f);          // riga 639
rhs_lin = (F_hand - F_feedback) - D * v;             // riga 666
// → rhs = F_hand + kf * Fs - D * v
```

**Scenario: slave tocca un muro, operatore fermo** ($F_{\text{hand}} = 0$):

1. Lo slave contatta il muro → $F_s$ appare con direzione opposta al moto
2. $F_{\text{feedback}} = -k_f F_s$ → **punta nella stessa direzione del moto**
3. Sul master: $\text{rhs} = 0 - (-k_f F_s) - Dv = k_f F_s - Dv$
4. Il master accelera **all'indietro** (via dalla parete) — corretto in principio
5. **Ma**: il segnale $F_s$ arriva con **ritardo** (TF, filtro EMA, rete ROS, 1÷20 ms tipicamente)
6. Il master reagisce al ritardato $F_s$, e quando il master si muove indietro, la forza sullo slave cambia → il master reagisce in ritardo → **ciclo di oscillazione**

> [!CAUTION]
> Il loop forza-forza con ritardo è intrinsecamente **non passivo**: l'energia iniettata dal ritardo non viene dissipata dallo smorzatore $D$ se il range di frequenza è sufficientemente alto. Questo è esattamente il motivo per cui vedi il master tremare quando lo slave sente una forza.

### Perché la molla da sola non risolve

Aggiungere $-K_s(p_m - p_s)$ al master **aiuta** perché introduce un accoppiamento posizionale dissipativo, ma se mantieni **anche** la riflessione diretta $k_f F_s$, i due canali possono interferire:

- Molla dice: "vai verso lo slave" (corretto)
- Riflessione FT dice: "allontanati dalla forza" (corretto ma ritardato)
- Se sono in fase opposta si cancellano; se il ritardo li mette in fase, amplificano

---

## Problema 2: PID vs. Compliance sullo slave — conflitto critico

### Bug nel codice attuale

Guarda la riga 489-497 dello slave:

```cpp
const double alpha = computeAlpha(wrench_filt_.f, v_ff_lin);  // 489

v_cmd_lin = alpha * k_ff_ * v_ff_lin + corr_p;   // 491  ← corr_p NON è scalato!
// ...
v_cmd_lin += v_comp_lin;                           // 497  ← compliance
```

**`alpha` scala solo il feedforward**, ma **il PID (`corr_p`) continua a spingere a piena potenza!**

**Scenario: slave in contatto con un muro**:

```
Tempo 0: master fermo, slave allineato, F_s = 0
Tempo 1: operatore spinge master verso muro
Tempo 2: slave raggiunge il muro, F_s cresce
Tempo 3: alpha → 0 (feedforward disabilitato, corretto!)
         MA: corr_p = Kp * (p_master - p_slave) → continua a crescere!
         E:  v_comp = -k_adm * F_s → spinge via dal muro
         RISULTATO: PID e compliance si combattono
Tempo 4: lo slave oscilla tra PID (avanti) e compliance (indietro)
```

> [!WARNING]
> Il PID non "sa" che lo slave è bloccato da un ostacolo. Continua a generare velocità verso il target (che è la posa del master, al di là del muro). Questo è in **conflitto diretto** con la compliance.

Il risultato è che:
- Lo slave mantiene una forza non nulla sull'ostacolo (equilibrio PID vs compliance)
- Questa forza si riflette sul master via $k_f F_s$, creando un feedback costante anche se l'operatore non spinge più
- Circolo vizioso

---

## Problema 3: La molla interagisce male col PID durante il contatto

Se aggiungiamo la molla virtuale $K_s(p_m - p_s)$ al master senza modificare lo slave:

1. Operatore spinge → master avanza → slave segue (PID) → va bene
2. Slave tocca muro → si ferma → $p_m - p_s$ cresce
3. Molla tira il master indietro — **bene**, operatore lo sente
4. **Ma** il PID sullo slave continua a generare $K_p(p_m - p_s)$ → spinge against il muro
5. La forza residua FT si riflette sul master tramite $k_f F_s$
6. Sul master hai molla (buona, stabile) + riflessione FT (cattiva, instabile) → risultato incerto

---

## 4. Proposta: architettura rivisitata

Servono **due correzioni** per avere un sistema stabile e trasparente.

### 4.1 Sul master: molla virtuale + eliminare o attenuare la riflessione FT diretta

**Opzione A** (consigliata): **Spring-only, senza FT diretta**

$$
M \dot{v} + D v = F_{\text{hand}} - K_s(p_m - p_s) - B_s(v_m - v_s)
$$

| Pro | Contro |
|---|---|
| Passivamente stabile (nessun ritardo di misura nel loop) | L'operatore sente la rigidità come "elastico", non come forza di contatto |
| Nessuna sensibilità al rumore FT | Meno "texture" haptica al contatto |
| Semplice da tarare | |

**Opzione B**: **Spring + FT attenuata**

$$
M \dot{v} + D v = F_{\text{hand}} - K_s(p_m - p_s) - B_s(v_m - v_s) - k_f F_s
$$

Con $k_f$ **molto basso** (0.01÷0.05 anziché 0.3). Qui la molla fa il lavoro principale e l'FT aggiunge una sfumatura di feedback di contatto.

**Opzione C**: **FT attenuata solo come termine di damping direzionale** (avanzata)

Usa $F_s$ non come forza riflessa, ma come segnale per **aumentare lo smorzamento** nella direzione del contatto:

$$
D_{\text{eff}} = D + k_d \frac{F_s F_s^T}{\|F_s\|^2 + \epsilon}
$$

Questo rende il master più "viscoso" quando lo slave è in contatto, senza iniettare energia.

### 4.2 Sul slave: rimuovere il conflitto PID vs compliance

**Opzione 1** (semplice, consigliata): **Scalare anche il PID con alpha**

```cpp
// PRIMA (attuale):
v_cmd_lin = alpha * k_ff_ * v_ff_lin + corr_p;

// DOPO (corretto):
v_cmd_lin = alpha * k_ff_ * v_ff_lin + alpha * corr_p;
```

Quando la forza è alta, alpha → 0, quindi sia feedforward sia PID si disattivano, e rimane solo la compliance. Lo slave si "arrende" all'ostacolo.

**Opzione 2** (più sofisticata): **PID con guadagno proporzionale dipendente dalla forza**

$$
K_p^{\text{eff}} = K_p \cdot \alpha(F) + K_p^{\text{min}}
$$

Dove $K_p^{\text{min}}$ è un piccolo guadagno residuo che mantiene lo slave vagamente orientato verso il target anche in contatto, ma senza combattere troppo.

**Opzione 3**: **Eliminare il PID, usare solo FF + compliance + spring**

$$
v_s = k_{\text{ff}} \cdot v_m + K_s(p_m - p_s) - k_{\text{adm}} F_s
$$

Il termine $K_s(p_m - p_s)$ sostituisce il PID. In questo caso la "molla virtuale" è simmetrica: presente sia sul master sia sullo slave. Questo è l'approccio più pulito dal punto di vista della passività.

---

## 5. Architettura raccomandata (completa)

```
                        ┌─────────────────────────┐
                        │     Molla virtuale       │
              ┌─────────┤  K_s·(pm-ps) + B_s·Δv   ├─────────┐
              │         └─────────────────────────┘         │
              ▼                                              ▼
   ┌─────────────────┐                          ┌─────────────────┐
   │     MASTER       │                          │      SLAVE      │
   │                  │    v_ff (feedforward)     │                 │
   │ M·dv + D·v      │─────────────────────────►│  v_s = α·kff·vm │
   │ = F_hand         │                          │       + α·Kp·ep │
   │ - F_spring       │                          │       - kadm·Fs │
   │                  │    p_target (pose)        │                 │
   │                  │─────────────────────────►│  ep = pm - ps    │   
   └────────┬─────────┘                          └────────┬────────┘
            │ v_cmd                                       │ v_cmd
            ▼                                             ▼
         Master HW                                     Slave HW
```

| Cambiamento | Dove | Effetto |
|---|---|---|
| Rimuovere/ridurre $k_f F_s$ | Master | Elimina l'instabilità da ritardo FT |
| Aggiungere molla $K_s(p_m - p_s)$ | Master | Feedback posizionale stabile |
| Scalare PID con alpha | Slave | Elimina conflitto PID vs compliance |

---

## 6. Parametri suggeriti per il primo test

```yaml
# MASTER — nuovi parametri molla virtuale
spring_stiffness_linear: 15.0     # N/m  (iniziare a 5-10, salire gradualmente)
spring_damping_linear: 3.0        # N·s/m
max_spring_force: 15.0            # N (saturazione sicurezza)
force_reflection_scale: 0.0       # DISABILITATA (oppure 0.02-0.05)

# SLAVE — alpha scala tutto (non solo feedforward)
# (modifica nel codice, non nel yaml — nessun parametro nuovo necessario)
```

---

## 7. Riepilogo: cosa stiamo risolvendo

| Problema | Causa root | Soluzione |
|---|---|---|
| Master instabile al contatto slave | FT diretta ritardata nel loop ammettenza | Molla virtuale (posizionale, senza ritardo) |
| PID vs compliance fight | PID non scalato da alpha(F) | Scalare PID con alpha come il feedforward |
| Deriva posizionale | Nessun termine K nel master | Molla virtuale $K_s$ |
| Forza residua sull'ostacolo | PID forza il contatto, compliance resiste | Alpha→0 disattiva PID al contatto |


## 8. Aggiunte

### Punto 1: Perché l'Opzione 3 (senza PID) risolve il problema del movimento in contatto
Hai ragione a preoccuparti delle Opzioni 1-2: se Alpha scala tutto a zero durante il contatto, perdi il controllo. Ma con l'Opzione 3 non c'è alpha: il bilanciamento avviene naturalmente tra molla e compliance.

L'equazione dello slave diventa:

$$v_s = k_{\text{ff}} \cdot v_m + K_s(p_m - p_s) - k_{\text{adm}} F_s$$

Scenario: slave in contatto, operatore vuole muovere

Operatore muove il master → $v_m \neq 0$, $p_m$ si sposta
$K_s(p_m - p_s)$ cresce → spinge lo slave a seguire
Se la direzione è lungo la superficie: $F_s$ non cambia molto → lo slave scorre
Se la direzione è contro l'ostacolo: $F_s$ cresce → compliance resiste → lo slave si muove meno
Non c'è nessun alpha che azzera tutto — l'operatore mantiene sempre il controllo
L'equilibrio si autoregola: la forza che lo slave applica all'ostacolo è $F_s = K_s \cdot e_p / k_{\text{adm}}$. L'operatore controlla la forza di contatto spostando il master più o meno avanti.

### Punto 2: Errore a regime senza PID — serve un integrale?
La domanda è: con la sola molla $K_s$, c'è un errore di posizione stazionario?

Analizziamo la dinamica. In free space ($F_s = 0$):

$$\dot{p}s = v_s = k{\text{ff}} \cdot v_m + K_s(p_m - p_s)$$

Definiamo $e_p = p_m - p_s$, quindi $\dot{e}_p = v_m - v_s$:

$$\dot{e}p = v_m - k{\text{ff}} v_m - K_s e_p = (1 - k_{\text{ff}}) v_m - K_s e_p$$

Con $k_{\text{ff}} = 1.0$ (il valore attuale nel config):

$$\dot{e}_p = -K_s \cdot e_p$$

Questo è un sistema del primo ordine esponenzialmente stabile che converge a $e_p = 0$ con costante di tempo $\tau = 1/K_s$.

Con il feedforward a 1.0, non serve un integrale: l'errore stazionario è già zero in free space.

Il feedforward fornisce la componente di velocità, la molla corregge la deviazione transitoria.

Ma cosa succede con disturbi persistenti?
Se c'è una perturbazione costante $d$ (attrito, gravità, offset del modello):

$$\dot{e}p = -K_s e_p + d \quad \Rightarrow \quad e{p,\text{ss}} = \frac{d}{K_s}$$

In questo caso c'è un errore stazionario $d/K_s$. Per eliminarlo, si può aggiungere un termine integrale leggero con clamping intelligente:

$$v_s = k_{\text{ff}} v_m + K_s e_p + K_i \int e_p , dt - k_{\text{adm}} F_s$$

Con le seguenti protezioni anti-windup:

Condizione	Azione sull'integrale
$|F_s| > F_{\text{freeze}}$	Freeze dell'integrale (non accumula)
$|e_p|$ cala sotto soglia	Integrale attivo normalmente
Integrale supera un max	Clamp del valore
In questo modo:

In free space: l'integrale corregge lentamente i disturbi costanti
In contatto: l'integrale si congela, non accumula windup, non combatte la compliance
Riepilogo: architettura slave finale proposta
$$v_s = \underbrace{k_{\text{ff}} \cdot v_m}{\text{feedforward}} + \underbrace{K_s \cdot e_p + K_i \int{\text{clamped}} e_p , dt}{\text{molla + integrale}} - \underbrace{k{\text{adm}} \cdot F_s}_{\text{compliance}}$$

Con l'integrale congelato quando $|F_s| > F_{\text{freeze}}$.

Vuoi che proceda con l'implementazione? In tal caso preparo un implementation plan dettagliato con le modifiche a entrambi i nodi (master: molla virtuale, slave: rimozione PID e sostituzione con spring + integrale clamped).