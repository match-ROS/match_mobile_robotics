# Whole-Body Preferred TCP Tuning Notes

Questo file spiega in dettaglio come funzionano i parametri piu' importanti per il comportamento:

- la base deve tenere il TCP in una zona preferita rispetto al robot;
- il braccio deve compensare solo localmente;
- se il TCP si allontana troppo dalla zona ottimale, la base deve recuperare.

Il contenuto qui sotto descrive il comportamento del controller attuale implementato in:

- `src/whole_body_print_controller_node.cpp`
- `config/whole_body_print_demo.yaml`

## Obiettivo del controllo

L'idea non e' "il braccio segue il path e la base aiuta un po'".

L'idea corretta e':

1. il path definisce dove deve stare il TCP nel mondo;
2. la base deve cercare di mantenere il TCP in una zona di lavoro comoda rispetto a `base_link`;
3. il braccio rifinisce il tracking locale;
4. se il TCP si allontana troppo dalla zona comoda, il contributo del braccio viene limitato e la base deve fare di piu'.

## Coordinate di riferimento

I parametri seguenti sono espressi nel frame della base:

- `preferred_tcp_x`
- `preferred_tcp_y`
- `arm_full_x_error`
- `arm_start_x_error`
- `k_preferred_x`
- `k_lateral`

Interpretazione:

- `x`: avanti/dietro rispetto a `base_link`
- `y`: laterale sinistra/destra rispetto a `base_link`

## 1. `preferred_tcp_x`, `preferred_tcp_y`

Questi due parametri definiscono il punto ottimale del TCP rispetto alla base mobile.

In pratica rispondono alla domanda:

> "Se il robot stesse lavorando bene, dove vorrei vedere il TCP rispetto alla base?"

### Significato pratico

Se imposti:

```yaml
preferred_tcp_x: 0.50
preferred_tcp_y: -0.80
```

vuol dire che il TCP ideale e':

- `0.50 m` davanti alla base
- `0.80 m` sul lato destro della base

### Effetto sul comportamento

Questo punto non e' il path.
E' il centro della zona di lavoro preferita.

Il controller usa questo punto per:

- decidere se il TCP e' ancora in una zona buona per il braccio;
- generare una spinta della base per riportare il TCP in zona.

### Come sceglierli

Sceglili guardando:

- una posa del braccio in cui il manipolatore e' raccolto e non vicino ai limiti;
- una posa in cui il polso non e' "tirato";
- una posa in cui la base ha ancora margine per ruotare o avanzare.

### Errore tipico

Un errore comune e' usare `preferred_tcp_x/y` per correggere un path definito male.

Non farlo.

Regola:

- il path descrive il lavoro nel mondo;
- `preferred_tcp_x/y` descrivono la zona di lavoro comoda del robot.

## 2. `arm_full_x_error`, `arm_full_y_error`

Questi parametri definiscono la zona interna in cui:

- il target e' considerato comodo rispetto alla base;
- il TCP reale e' considerato ancora in posizione corretta;
- il braccio puo' seguire il target pienamente.

### Concetto

Pensa a una "zona verde" attorno a `preferred_tcp_x/y`.

Se sia il target sia il TCP reale sono dentro questa zona:

- `arm_tracking_scale = 1.0`
- il braccio puo' seguire completamente il target
- la base non e' ancora considerata "fuori tracking zone"

### Implementazione attuale

Il controller calcola:

- errore del target rispetto al punto preferito
- errore del TCP reale rispetto al punto preferito

e considera "in zona" solo se entrambi restano entro `full_*`.

In forma semplificata:

```text
target_dx = abs(target_in_base.x - preferred_tcp_x)
target_dy = abs(target_in_base.y - preferred_tcp_y)
tcp_dx    = abs(tcp_in_base.x - preferred_tcp_x)
tcp_dy    = abs(tcp_in_base.y - preferred_tcp_y)

in_full_zone =
  target_dx <= arm_full_x_error &&
  target_dy <= arm_full_y_error &&
  tcp_dx    <= arm_full_x_error &&
  tcp_dy    <= arm_full_y_error
```

### Effetto del tuning

Se li aumenti:

- il braccio resta pienamente attivo su una zona piu' ampia;
- la base interviene piu' tardi;
- aumenta il rischio che il braccio si distenda troppo.

Se li riduci:

- la base viene chiamata in causa prima;
- il braccio resta piu' raccolto;
- se li riduci troppo, la base diventa dominante e il sistema puo' sembrare lento o rigido.

### Regola pratica

Per il tuo caso, questi parametri devono rappresentare davvero l'intorno "buono" del TCP, non l'intorno massimo fisicamente raggiungibile.

## 3. `arm_start_x_error`, `arm_start_y_error`

Questi parametri definiscono la soglia esterna oltre la quale il braccio viene fortemente limitato.

Pensa a una seconda zona, piu' grande, una "zona gialla".

Fra `full_*` e `start_*` il controller fa un blending progressivo.
Fuori `start_*` applica il limite forte al contributo del braccio.

### Implementazione attuale

Il controller usa:

```text
dx = max(target_dx, tcp_dx)
dy = max(target_dy, tcp_dy)
```

Poi:

- se `dx <= full_x` e `dy <= full_y` -> scala `1.0`
- se `dx >= start_x` oppure `dy >= start_y` -> scala `arm_far_scale`
- in mezzo -> blending continuo

La parte in mezzo e':

```text
rx = max(0, (dx - full_x) / (start_x - full_x))
ry = max(0, (dy - full_y) / (start_y - full_y))
blend = clamp(max(rx, ry), 0, 1)
arm_scale = 1 - blend * (1 - arm_far_scale)
```

### Effetto del tuning

Se aumenti `arm_start_x_error` e `arm_start_y_error`:

- il braccio resta attivo anche quando il TCP e' piu' lontano dal punto ottimale;
- la base viene sollecitata piu' tardi;
- il braccio tende a distendersi di piu'.

Se li riduci:

- il braccio viene "strozzato" prima;
- la base e' costretta a recuperare prima;
- il TCP resta piu' vicino alla zona preferita.

### Regola importante

Devono essere piu' grandi di `arm_full_*`.

Una scelta tipica e':

- `arm_full_x_error`: zona veramente comoda
- `arm_start_x_error`: limite oltre il quale il braccio non deve piu' "salvare la situazione"

## 4. `k_preferred_x`

Questo parametro controlla quanto la base viene spinta avanti o indietro per riportare il TCP nella posizione preferita lungo l'asse `x`.

### Implementazione attuale

Nel solver della base si calcola un riferimento:

```text
x_error = tcp_in_base.x - preferred_tcp_x
u_ref_linear = k_preferred_x * x_error
```

Se `allow_reverse` e' `false`, la componente negativa viene tagliata.

### Interpretazione

Se il TCP e' troppo avanti rispetto al punto preferito:

- `x_error > 0`
- il controller chiede alla base di avanzare
- l'idea e' "inseguire il TCP" con la navetta per riportarlo in zona

Questo e' esattamente il caso che hai descritto:

- il braccio si distende in avanti;
- il TCP si trova troppo avanti;
- la base deve accelerare in avanti per riprendersi il TCP.

### Se e' troppo basso

Sintomi:

- la base resta indietro;
- il braccio continua a compensare troppo;
- il TCP lavora davanti alla zona ottimale per troppo tempo.

### Se e' troppo alto

Sintomi:

- la base accelera aggressivamente;
- puo' fare overshoot;
- il sistema puo' diventare nervoso quando il TCP entra ed esce dalla zona preferita.

### Nota importante

`k_preferred_x` non genera da solo tutta la velocita' della base.
E' un bias del solver.

Il comportamento finale dipende anche da:

- `weight_linear`
- `max_linear_velocity`
- `max_linear_acceleration`
- contributo del task principale di tracking del path

## 5. `k_lateral`

Questo parametro controlla quanto la base reagisce quando il TCP e' spostato lateralmente rispetto alla zona preferita.

### Implementazione attuale

Nel solver:

```text
y_error = tcp_in_base.y - preferred_tcp_y
u_ref_angular = k_lateral * y_error
```

Questa componente agisce sulla velocita' angolare della base.

### Interpretazione

La base differenziale non puo' muoversi lateralmente puro.
Quindi un errore laterale del TCP viene corretto soprattutto ruotando la base.

### Se e' troppo basso

Sintomi:

- il TCP resta disassato lateralmente;
- la navetta non "si mette sotto" al lavoro;
- il braccio compensa di lato piu' del dovuto.

### Se e' troppo alto

Sintomi:

- la base ruota troppo;
- possono comparire oscillazioni o zig-zag;
- il path puo' essere seguito con orientamento troppo aggressivo.

## 6. `max_linear_velocity`

Questo parametro e' il limite duro sulla velocita' lineare della base.

### Cosa fa davvero

Anche se il solver vorrebbe far andare la base piu' veloce, il comando viene saturato:

```text
cmd.linear.x = clamp(u_linear, -max_linear_velocity, max_linear_velocity)
```

Se `allow_reverse` e' `false`, resta solo la parte positiva.

### Perche' e' importante nel tuo caso

Se:

- il braccio si e' gia' allungato molto;
- `k_preferred_x` chiede piu' recupero;
- ma `max_linear_velocity` e' troppo basso;

allora la base continua a essere "corretta" nella direzione giusta, ma troppo lentamente.

Il sintomo tipico e':

- la base sembra capire cosa fare;
- pero' non ha abbastanza autorita' per recuperare il TCP.

### Come riconoscerlo

Guarda il debug:

- `base_linear_saturated = true` spesso
- `base_command.linear.x` spesso vicino al massimo

Se succede spesso, il limite e' un collo di bottiglia reale.

### Attenzione

Alzarlo troppo senza rivedere l'accelerazione puo' produrre un robot che:

- vuole andare veloce;
- ma arriva con partenze e correzioni brusche.

Quindi spesso `max_linear_velocity` va ragionato insieme a:

- `max_linear_acceleration`
- `k_preferred_x`

## Come interagiscono insieme

Questi parametri non sono indipendenti.

### Caso 1: il braccio si distende troppo in avanti

Cause probabili:

- `arm_full_x_error` troppo grande
- `arm_start_x_error` troppo grande
- `k_preferred_x` troppo basso
- `max_linear_velocity` troppo basso

Effetto:

- il braccio continua a lavorare troppo in `x`
- la base non recupera con abbastanza decisione

### Caso 2: la base prova a recuperare ma resta lenta

Cause probabili:

- `k_preferred_x` basso
- `weight_linear` alto
- `max_linear_velocity` basso
- `max_linear_acceleration` basso

### Caso 3: la base ruota molto ma non si posiziona bene

Cause probabili:

- `k_lateral` troppo alto
- `k_preferred_x` troppo basso
- `preferred_tcp_y` scelto in modo poco realistico

## Procedura di tuning consigliata

### Step 1: fissa bene la geometria

Prima scegli:

- `preferred_tcp_x`
- `preferred_tcp_y`

Devono descrivere la posizione comoda del TCP, non una correzione artificiale del path.

### Step 2: stringi la zona comoda del braccio

Lavora su:

- `arm_full_x_error`
- `arm_full_y_error`
- `arm_start_x_error`
- `arm_start_y_error`

Obiettivo:

- il braccio deve essere libero solo nella zona davvero utile;
- fuori zona la base deve diventare il principale attuatore di recupero.

### Step 3: aumenta l'autorita' della base in `x`

Lavora su:

- `k_preferred_x`
- `max_linear_velocity`
- eventualmente `max_linear_acceleration`

Obiettivo:

- quando il TCP e' troppo avanti, la base deve riuscire davvero a riportarlo indietro rispetto a se stessa.

### Step 4: rifinisci il laterale

Lavora su:

- `k_lateral`

Obiettivo:

- la base deve rimettersi sotto al TCP senza zig-zag.

## Cosa guardare nel debug

I campi piu' utili sono:

- `target_in_base`
- `current_tcp_position`
- `arm_tracking_scale`
- `base_in_tracking_zone`
- `base_command`
- `base_linear_saturated`
- `base_angular_saturated`
- `preferred_tcp_x`
- `preferred_tcp_y`

### Interpretazione rapida

Se:

- `arm_tracking_scale` resta vicino a `1.0`
- il TCP e' lontano dal punto preferito

allora la zona del braccio e' ancora troppo permissiva.

Se:

- `arm_tracking_scale` scende
- ma la base non recupera

allora il problema e' piu' lato:

- `k_preferred_x`
- `max_linear_velocity`
- `max_linear_acceleration`

Se:

- `base_linear_saturated` e' quasi sempre `true`

allora la base sta gia' chiedendo il massimo e non basta.

## Regola pratica finale

Per ottenere il comportamento che desideri:

- `preferred_tcp_x/y` devono descrivere una posa davvero comoda;
- `arm_full_*` devono essere abbastanza stretti;
- `arm_start_*` non devono lasciare troppo margine al braccio;
- `k_preferred_x` deve dare vera autorita' alla base;
- `max_linear_velocity` deve permettere fisicamente alla navetta di recuperare.

Se il comportamento desiderato e':

> "il TCP continua il suo moto, ma la base accelera per riportarlo nella zona ottimale"

allora il tuning va fatto soprattutto nell'ordine:

1. `arm_full_x_error`
2. `arm_start_x_error`
3. `k_preferred_x`
4. `max_linear_velocity`
5. `k_lateral`
