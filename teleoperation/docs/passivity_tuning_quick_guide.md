# Guida rapida al tuning del Passivity Layer

## Obiettivo

Questa guida serve per tarare i parametri del blocco:

- `passivity/enabled`
- `passivity/linear_only`
- `passivity/tank_energy_init`
- `passivity/tank_energy_min`
- `passivity/tank_energy_max`
- `passivity/recharge_gain`
- `passivity/discharge_gain`
- `passivity/power_deadband`
- `passivity/gamma_min`
- `passivity/gamma_lowpass_alpha`
- `passivity/gamma_rate_limit`

nel file `config/master_real_mur620b_ur10_l.yaml`.

Il layer agisce solo sulla riflessione diretta del wrench dello slave verso il master:

```text
F_reflection = -kf_force * F_slave
Tau_reflection = -kf_torque * Tau_slave
```

La molla virtuale resta fuori.

## Unita di misura

### Grandezze fisiche

- `F_reflection`, `F_slave`: `N`
- `Tau_reflection`, `Tau_slave`: `N*m`
- `v_lin`: `m/s`
- `omega`: `rad/s`
- `D_lin`: `N*s/m`
- `D_ang`: `N*m*s/rad`
- `power_out_requested`, `power_out_applied`, `power_diss`: `W = J/s`
- `energy_before`, `energy_after`, `tank_energy_*`: `J`
- `dt`: `s`

### Parametri adimensionali o quasi

- `gamma_raw`, `gamma_applied`: adimensionali, tra `0` e `1`
- `gamma_min`: adimensionale, tra `0` e `1`
- `recharge_gain`: adimensionale
- `discharge_gain`: adimensionale
- `gamma_lowpass_alpha`: adimensionale, tra `0` e `1`
- `linear_only`: booleano
- `enabled`: booleano

### Parametro dinamico

- `gamma_rate_limit`: `1/s`

Significa quanto rapidamente `gamma` puo risalire verso `1`.

## Come leggere gli ordini di grandezza

Il tank lavora sull'energia, quindi e utile ragionare cosi:

### Parte lineare

```text
P = F * v
```

Esempi:

- `5 N * 0.05 m/s = 0.25 W`
- `10 N * 0.10 m/s = 1.0 W`
- `20 N * 0.10 m/s = 2.0 W`

Se questa potenza resta attiva per `0.1 s`, l'energia coinvolta e:

```text
E = P * dt
```

Esempi:

- `0.25 W * 0.1 s = 0.025 J`
- `1.0 W * 0.1 s = 0.10 J`
- `2.0 W * 0.1 s = 0.20 J`

### Parte rotazionale

```text
P = Tau * omega
```

Esempi:

- `0.5 N*m * 0.5 rad/s = 0.25 W`
- `1.0 N*m * 1.0 rad/s = 1.0 W`

### Dissipazione dell'ammettenza

Il recharge del tank usa la dissipazione gia presente nel damping:

```text
P_diss = v^T D v
```

Esempio lineare:

- `D = 10 N*s/m`, `v = 0.10 m/s`
- `P_diss = 10 * 0.1^2 = 0.10 W`

Quindi, con velocita basse, il tank si ricarica lentamente. Questo e normale.

## Interpretazione pratica dei parametri

### `tank_energy_init` [J]

Energia iniziale del tank all'avvio o dopo reset.

- troppo bassa: il feedback viene tagliato quasi subito;
- troppo alta: il layer interviene tardi e protegge poco.

Ordine di grandezza iniziale sensato:

- `0.5 - 2.0 J`

### `tank_energy_min` [J]

Energia minima che vuoi sempre lasciare nel tank.

- piu alta: protezione piu conservativa;
- piu bassa: piu trasparenza, meno margine.

Ordine di grandezza iniziale:

- `0.05 - 0.3 J`

### `tank_energy_max` [J]

Capacita massima del tank.

- troppo bassa: il sistema diventa facilmente "strozzato";
- troppo alta: il tank accumula troppa energia e interviene poco.

Ordine di grandezza iniziale:

- `2 - 10 J`

### `power_deadband` [W]

Sotto questa soglia il layer non considera significativa la potenza in uscita.

Serve per non reagire a rumore, micro-movimenti e piccoli segnali.

Ordine di grandezza iniziale:

- `0.05 - 0.3 W`

Se il sistema taglia troppo presto, alza leggermente la deadband.
Se invece lascia passare troppa attivita piccola ma vibratoria, abbassala.

### `recharge_gain` [-]

Quanto velocemente la dissipazione del damping ricarica il tank.

- piu alto: il tank recupera energia piu in fretta;
- troppo alto: il layer diventa poco incisivo.

Valore iniziale consigliato:

- `1.0`

Range pratico:

- `0.5 - 2.0`

### `discharge_gain` [-]

Quanto "costa" al tank la potenza attiva riflessa.

- piu alto: il layer e piu severo;
- piu basso: il layer lascia passare piu energia.

Valore iniziale consigliato:

- `1.0`

Range pratico:

- `1.0 - 3.0`

### `gamma_lowpass_alpha` [-]

Smussa la risalita di `gamma`.

- `0`: nessun filtraggio;
- vicino a `1`: risalita molto rapida.

Valore iniziale sensato:

- `0.1 - 0.3`

### `gamma_rate_limit` [1/s]

Limita quanto velocemente `gamma` puo tornare a `1`.

- piu basso: recupero piu lento e stabile;
- piu alto: recupero piu rapido e piu trasparente.

Ordine di grandezza iniziale:

- `5 - 20 1/s`

### `gamma_min` [-]

Impone un valore minimo a `gamma_applied`.

- `0`: comportamento standard, il layer puo tagliare fino a zero;
- `> 0`: mantiene sempre una parte del feedback, anche quando il tank chiederebbe di tagliare di piu.

Valore iniziale prudente:

- `0.05 - 0.20`

Nota: utile per evitare la completa perdita di contatto percepito, ma riduce la garanzia di passivita stretta.

## Sequenza di tuning consigliata

### 1. Parti solo con la parte lineare

Nel tuo setup attuale:

- `use_torques: false`

Quindi per i primi test puoi anche ragionare come se il problema fosse solo traslazionale.

### 2. Tieni bassa la riflessione base

Prima di tarare il tank, conviene non avere un `force_reflection_scale` troppo aggressivo.

Per primi test reali io starei circa in:

- `0.05 - 0.20`

e non partirei da `0.65`.

### 3. Inizia con un tank conservativo

Proposta iniziale:

```yaml
passivity:
  enabled: true
  linear_only: true
  tank_energy_init: 1.0
  tank_energy_min: 0.1
  tank_energy_max: 4.0
  recharge_gain: 1.0
  discharge_gain: 1.5
  power_deadband: 0.10
  gamma_min: 0.0
  gamma_lowpass_alpha: 0.20
  gamma_rate_limit: 10.0
```

Questa configurazione tende a proteggere piu la stabilita che la trasparenza.

### 4. Guarda il topic debug

Il debug del passivity layer e pubblicato nel namespace privato del nodo:

- setup singolo: `/<nome_nodo>/debug/passivity_stats`
- setup duale: `/teleop_master_haptic_controller_left/debug/passivity_stats`
- setup duale: `/teleop_master_haptic_controller_right/debug/passivity_stats`

Il topic pubblica:

1. `energy_before` `[J]`
2. `energy_after` `[J]`
3. `gamma_raw` `[-]`
4. `gamma_applied` `[-]`
5. `power_out_requested` `[W]`
6. `power_out_applied` `[W]`
7. `power_diss` `[W]`
8. `|F_reflection_requested|` `[N]`
9. `|F_reflection_used|` `[N]`
10. `|Tau_reflection_requested|` `[N*m]`
11. `|Tau_reflection_used|` `[N*m]`

### 5. Come interpretare i risultati

#### Caso A: il master resta ancora oscillatorio

Vuol dire che il layer e troppo permissivo.

Prova in questo ordine:

1. abbassa `force_reflection_scale`
2. aumenta `discharge_gain`
3. riduci `tank_energy_init`
4. riduci `tank_energy_max`
5. riduci `power_deadband`

#### Caso B: il feedback diventa troppo "morto"

Vuol dire che il layer taglia troppo presto o recupera troppo lentamente.

Prova in questo ordine:

1. aumenta `tank_energy_init`
2. aumenta `tank_energy_max`
3. aumenta `power_deadband`
4. aumenta `recharge_gain`
5. aumenta `gamma_rate_limit`

#### Caso C: il contatto e stabile ma il rilascio e nervoso

Di solito il problema e nel recupero di `gamma`.

Prova:

1. abbassare `gamma_rate_limit`
2. ridurre `gamma_lowpass_alpha`

## Regole pratiche veloci

- Se `gamma_applied` sta quasi sempre vicino a `1`, il layer sta intervenendo poco.
- Se `gamma_applied` crolla spesso verso `0`, il layer e molto aggressivo.
- Se `energy_after` resta quasi sempre vicino a `tank_energy_min`, il tank e troppo piccolo o si scarica troppo.
- Se `energy_after` resta quasi sempre vicino a `tank_energy_max`, il tank e probabilmente troppo permissivo.

## Valori iniziali consigliati per te

Per il tuo caso io partirei cosi:

```yaml
force_reflection_scale: 0.10
torque_reflection_scale: 0.10

passivity:
  enabled: true
  linear_only: true
  tank_energy_init: 1.0
  tank_energy_min: 0.1
  tank_energy_max: 4.0
  recharge_gain: 1.0
  discharge_gain: 1.5
  power_deadband: 0.10
  gamma_lowpass_alpha: 0.20
  gamma_rate_limit: 10.0
```

Poi farei i test in questo ordine:

1. contatto singolo leggero;
2. contatto singolo rigido;
3. grasp bimanuale leggero;
4. grasp bimanuale piu rigido.

## Ultima nota importante

Il tank non sostituisce il tuning del resto del loop. Se il sistema resta instabile anche con `gamma` che taglia molto, la causa non e solo nella riflessione diretta, ma anche in:

- `force_reflection_scale` troppo alto;
- frame del wrench incoerenti;
- molla virtuale poco smorzata;
- slave troppo rigido al contatto.
