# Miglioramenti proposti per `teleop_master_haptic_controller_node.cpp` (C++)

Questo documento dettaglia interventi pratici per rendere il tuo controllo in ammettenza **più consistente**, **più fluido** e **più robusto**.

Target: `src/match_mobile_robotics/teleoperation/src/teleop_master_haptic_controller_node.cpp` e utility in `include/teleoperation/core/{wrench_utils.hpp,math_utils.hpp}`.

---

## 1) Usare un $dt$ consistente (eventualmente fisso)

### 1.1 Situazione attuale (pro/contro)

Attualmente il nodo usa un `ros::Timer` con periodo derivato da `control_rate`, ma integra usando:

- $dt = (now - last\_time)$

**Pro**
- segue il tempo reale (se il timer slitta, la dinamica resta “fisicamente” coerente).

**Contro**
- jitter del timer → jitter nel $dt$ → jitter sulla risposta (soprattutto se il wrench è rumoroso e/o se il filtro è leggero).
- se $dt$ ogni tanto raddoppia (latenza), l’integratore Euler forward può produrre “scatti”.

### 1.2 Opzione A (consigliata): $dt$ misurato ma “sanitizzato”

È la soluzione più robusta in pratica: usi $dt$ misurato, ma lo **clampi** e/o lo **spezzi** in più sub-step se troppo grande.

- **Parametri suggeriti**
  - `dt_nominal = 1.0 / control_rate`
  - `dt_min = 0.25 * dt_nominal`
  - `dt_max = 2.0 * dt_nominal` (o 3.0 se vuoi più tolleranza)

- **Regola**
  - se $dt < dt\_min$ → usa `dt_nominal` o salta aggiornamento (ma continua a pubblicare l’ultimo comando)
  - se $dt > dt\_max$ → o:
    - (1) clamp a `dt_max`, oppure
    - (2) **sub-stepping**: ripeti $N$ passi con $dt = dt/N$ (più stabile e più “morbido”)

**Perché aiuta la fluidità**
- impedisce che un singolo tick ritardato produca un salto di velocità $\Delta v = a\,dt$ troppo grande.

### 1.3 Opzione B: $dt$ fisso = $1/f$ (semplice ma va compreso bene)

Imposti $dt = 1.0/control\_rate$ e ignori il tempo reale.

**Vantaggio**
- risposta estremamente “regolare” (nessun jitter dovuto al timer).

**Rischio**
- se il loop gira più lento del rate nominale, tu **sottostimi** il tempo passato → dinamica più “rigida” e potenzialmente instabile rispetto alla realtà (stai integrando meno di quanto dovresti).

**Quando ha senso**
- quando sei sicuro che il loop sia molto vicino al rate nominale (jitter minimo) o se accetti che la dinamica sia “clock-driven” più che time-driven.

### 1.4 Dettaglio implementativo: usare `ros::TimerEvent`

Il callback riceve `ros::TimerEvent`. Tipicamente puoi usare:
- `ev.current_real - ev.last_real` come $dt$ (misurato dal timer)

È spesso più coerente di chiamare `ros::Time::now()` due volte, e riduce ambiguità se in futuro cambi time source.

---

## 2) Deadband “complessiva” (sulla norma) invece che per asse

### 2.1 Problema della deadband per asse

La deadband attuale azzera ogni asse se $|f_i| < db$. Questo causa:
- **“attrito virtuale anisotropo”**: anche quando sei già in movimento, se vuoi cambiare direzione devi superare la deadband su quell’asse.
- percezione di controllo “a scatti” quando combini componenti su assi diversi.

Tu chiedi una deadband **complessiva**: una volta “in moto”, piccoli contributi laterali non devono venire azzerati solo perché il singolo asse è sotto soglia.

### 2.2 Soluzione 1 (semplice): deadband dura sulla norma

Definisci:
- $n = \lVert \mathbf{f} \rVert$
- se $n < db$ → $\mathbf{f}_\text{out}=0$
- altrimenti $\mathbf{f}_\text{out}=\mathbf{f}$

**Pro**
- elimina il problema “per asse”.

**Contro**
- discontinuità netta al superamento della soglia: quando $n$ passa da $db-\epsilon$ a $db+\epsilon$ la forza “appare” di colpo.

### 2.3 Soluzione 2 (consigliata): deadzone “soft” sulla norma (continua)

Mantieni la direzione ma “rimuovi” una sfera di raggio $db$ in modo continuo:

- $n = \lVert \mathbf{f} \rVert$
- se $n \le db$ → $\mathbf{f}_\text{out}=0$
- se $n > db$ → $\mathbf{f}_\text{out} = \left(\frac{n-db}{n}\right)\mathbf{f}$

Questa formula:
- preserva direzione
- evita lo “step” netto
- garantisce che appena superi la soglia, la forza parte da 0 e cresce linearmente con $n-db$

### 2.4 Aggiunta utile: isteresi / “latch” di movimento

Per ridurre ulteriore chatter vicino alla soglia, puoi introdurre due soglie:
- `db_enter` (più alta) per “iniziare” a muoversi
- `db_exit` (più bassa) per “continuare” a muoversi

Logica tipica:
- se sei “fermo” e $n < db\_enter$ → output 0
- se sei “in moto” e $n < db\_exit$ → output 0 e torni “fermo”

**Impatto**
- una volta che hai superato la soglia e iniziato a muoverti, non “ricadi” subito nella deadband al minimo calo.

### 2.5 Dove applicarla nel tuo nodo

Oggi fai:
1) filtro EMA
2) deadband per asse
3) clamp norma

Con deadband sulla norma, una pipeline ragionevole è:

- **EMA** (per ridurre rumore)
- **deadzone soft sulla norma**
- **clamp sulla norma** (sicurezza)

Nota: se fai clamp prima della deadzone soft, la zona morta si comporta in modo strano a forze alte. Quindi meglio deadzone → clamp.

### 2.6 Torques: stesso ragionamento

Se abiliti `use_torques`, valuta deadband sulla norma anche per $\boldsymbol{\tau}$ (se vuoi un comportamento isotropo anche in rotazione).

---

## 3) Altri miglioramenti ad alto impatto (valutazione)

Qui sotto trovi interventi che spesso sono **più importanti della sola taratura $m,d$** per la fluidità percepita.

### 3.1 Anti-windup / gestione saturazione (molto consigliato)

Tu saturi la velocità con `clampNorm3(v, max_speed)`, ma l’integratore aggiorna comunque:
$$
v \leftarrow v + a\,dt
$$
e poi clamp.

In presenza di saturazione continua:
- lo stato “spinge” contro la saturazione e quando la forza cala può rilasciare energia in modo brusco (effetto simile a windup, anche se non c’è un integratore classico su errore).

Soluzioni:
- **clamp sull’accelerazione** (limitare $\lVert a \rVert$ o per asse)
- oppure aggiornare $v$ con una logica che rispetta la saturazione (es. proiettare l’incremento lungo la componente non saturata)
- oppure aggiungere una piccola **dissipazione extra** quando saturato (es. aumentare $d$ effective)

Risultato: transitori più morbidi quando vai in saturazione.

### 3.2 Limitatore di jerk / rate limiter sulla velocità commandata

Anche con filtro sul wrench, comandi di velocità possono cambiare “troppo in fretta” se il contatto è discontinuo.

Un “rate limiter” su $v$ (o meglio su $\dot{v}$ e/o jerk) produce un miglioramento netto di fluidità.

Tipico:
- limita $\Delta v / dt$ (acceleration limit)
- opzionale: limita $\Delta a / dt$ (jerk limit)

### 3.3 Migliorare il filtro: da EMA a low-pass “con frequenza” (opzionale)

EMA è ok e molto usato, ma spesso è più intuitivo specificare:
- una frequenza di taglio $f_c$ e derivare $\alpha$ da $dt$:
  $$
  \alpha = \frac{dt}{\tau + dt}, \quad \tau = \frac{1}{2\pi f_c}
  $$

Vantaggi:
- comportamento più consistente se $dt$ cambia (anche poco)
- tuning “fisico” in Hz

Se scegli anche di stabilizzare $dt$, questo punto è meno critico, ma resta utile.

### 3.4 Trasformazione del wrench: attenzione (possibile bug concettuale)

Nel tuo nodo attuale converti forza e torque con **solo rotazione** (due volte):
- $\mathbf{f}' = R\,\mathbf{f}$
- $\boldsymbol{\tau}' = R\,\boldsymbol{\tau}$

Questo è corretto **solo se il wrench è espresso rispetto allo stesso punto/origine** tra i frame (o se ti interessa solo la rotazione e l’origine coincide).

In generale, la trasformazione di un wrench tra frame con traslazione $p$ richiede:
- $\mathbf{f}' = R\,\mathbf{f}$
- $\boldsymbol{\tau}' = R\,\boldsymbol{\tau} + p \times (R\,\mathbf{f})$

Se `wrench_target_frame` e `wrench_source_frame` non hanno la stessa origine (tipico: tool vs base), trascurare il termine $p \times f$ può introdurre componenti spurie di torque (e talvolta influenzare anche interpretazione del comando se in futuro usi torques).

**Valutazione**
- se stai usando solo forze (`use_torques=false`) l’impatto è ridotto, ma resta importante capire *dove* è definita la forza e in quale frame la comandi.
- se abiliti torques, questa diventa una priorità.

### 3.5 Bias/offset e gravità del sensore (spesso la causa #1 di “non fluido”)

Se il tuo wrench include:
- bias del sensore
- peso utensile/non compensato
- drift termico

allora il controllore vede una forza “costante” che genera una velocità costante (o micro-movimenti) e tu finisci per combatterla con deadband/smorzamento.

Miglioramenti tipici:
- stima bias “a robot fermo” e sottrazione
- compensazione gravità/tool mass (se hai modello)
- filtro passa-basso dedicato sul bias (molto lento) + filtro più veloce sul segnale residuo

Risultato: puoi abbassare deadband e aumentare fluidità senza instabilità.

### 3.6 Parametri per asse (massa e damping anisotropi)

Oggi hai $m$ e $d$ scalari per la parte lineare (stesso su x,y,z).

In teleoperazione reale spesso serve:
- $M=\mathrm{diag}(m_x,m_y,m_z)$
- $D=\mathrm{diag}(d_x,d_y,d_z)$

Perché:
- assi verticali spesso richiedono più smorzamento/inerzia percepita
- contatti/attriti differiscono per direzione (es. su base mobile, o su manipolatore con compliance diversa)

### 3.7 Separare “forza mano” e “forza di feedback” con pesi/frequenze diverse

Hai già:
- $F_\text{hand}$
- $F_\text{feedback}$ (slave + coupling)

Miglioramenti:
- filtri differenti: più filtro su feedback (per evitare oscillazioni), meno su hand (per responsività)
- saturazioni separate
- eventuale blending dipendente dalla velocità o dal contesto (contatto/non contatto)

### 3.8 Diagnostica per tuning (ti fa risparmiare ore)

Hai già publisher di debug del wrench filtrato. Ulteriori segnali utili:
- $dt$ effettivo (min/max/mean)
- $a$ calcolata (norma)
- stato $v$ prima/dopo clamp + flag “saturato”
- norma di $F_\text{hand}$, $F_\text{feedback}$

Con questi puoi capire subito se la non-fluidità viene da:
- input rumoroso
- dt sporco
- saturazione continua
- deadband troppo alta/bassa

---

## 4) Proposta di roadmap minimale (priorità)

Se vuoi massimizzare l’impatto con poche modifiche:

- **P1**: $dt$ “sanitizzato” (misurato + clamp + optional sub-stepping).
- **P1**: deadband sulla **norma** (meglio soft + isteresi).
- **P1**: rate limit su velocità o accelerazione (per transitori morbidi).
- **P2**: bias/gravity compensation del wrench (se oggi stai usando deadband per “nascondere” offset).
- **P2**: anti-windup in saturazione (soprattutto se spesso raggiungi `max_linear_speed`).
- **P3**: wrench transform completo (se abiliti torques o se cambi frame/origini).

---

## 5) Note di tuning (intuizioni veloci)

Per la dinamica lineare $M\dot{v}+Dv=F$:
- la costante di tempo approssimata è $\tau \approx M/D$.
- se vuoi più “prontezza” riduci $\tau$ (aumenta $D$ o riduci $M$).
- se vuoi più “morbidezza” aumenta $M$ e regola $D$ per evitare oscillazioni.

Ma: se il problema è jitter/offset del wrench, **filtri + bias** contano più di $M,D$.

