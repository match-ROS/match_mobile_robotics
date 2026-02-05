## PID controller params (config/controller_params.yaml)

Riferimento: sezione `pid_controller` in `config/controller_params.yaml`. I parametri sono divisi tra controllo di posizione (XYZ) e orientamento (errore asse-angolo). I limiti di uscita sono collegati ai massimi del local planner:
- `output_limit` posizione = `max_linear_velocity` (0.5 m/s)
- `output_limit` orientamento = `max_angular_velocity` (1.0 rad/s)

### Parametri comuni (come influenzano il comportamento)
- `kp` (proporzionale): risponde subito all'errore. Valore alto = correzione piu aggressiva ma piu rischio di oscillazioni.
- `ki` (integrale): accumula errore nel tempo. Elimina l'offset, ma troppo alto porta a overshoot se l'uscita va spesso in saturazione.
- `kd` (derivativo): smorza variazioni rapide. Con il valore attuale `0.0` la derivata è disattivata.
- `kff` (feed-forward): aggiunge una quota diretta dalle velocita desiderate del planner. Riduce lo sforzo del feedback.
- `output_limit`: saturazione sulla norma del comando (mantiene la direzione). Protegge da velocita eccessive.
- `derivative_filter_tau`: costante di tempo del filtro passa-basso sulla derivata. Aumentarla filtra di piu ma rende il D piu lento.
- `deadband`: annulla errori molto piccoli per evitare inseguimenti nervosi quando il target è quasi raggiunto.

### Sezione posizione (XYZ)
Valori: `kp=2.0`, `ki=0.5`, `kd=0.0`, `kff=0.8`, `output_limit=0.5 m/s`, `deadband=0.0002 m`, `tau=0.02 s`.

Effetti pratici:
- Correzione P: 1 cm di errore -> `2.0 × 0.01 = 0.02 m/s`.
- Integrale: errore costante 5 mm per 2 s -> integrale = `0.005 × 2 = 0.01`; contributo I = `0.5 × 0.01 = 0.005 m/s` (aiuta a chiudere l'offset residuo).
- Feed-forward: se il local planner chiede 0.20 m/s, FF = `0.8 × 0.20 = 0.16 m/s`; il PID aggiunge correzioni sopra questo valore.
- Deadband: errori <0.2 mm vengono ignorati, utile per evitare dither dovuto a rumore sensori.
- Saturazione: qualsiasi combinazione P+I+FF che superi 0.5 m/s viene scalata mantenendo la direzione (evita comandi troppo rapidi).

Esempio rapido di tuning:
- Se il tracking è lento ma stabile: aumenta gradualmente `kp` (es. 2.5) tenendo `ki` costante.
- Se resta un piccolo offset a target: aumenta `ki` (es. 0.6) ma controlla eventuale overshoot.
- Se compaiono oscillazioni dovute a errori rapidi: imposta un `kd` piccolo (es. 0.05) e conferma che `tau=0.02` filtri abbastanza.

### Sezione orientamento (asse-angolo)
Valori: `kp=1.5`, `ki=0.4`, `kd=0.0`, `kff=0.8`, `output_limit=1.0 rad/s`, `deadband=0.001 rad`, `tau=0.02 s`.

Effetti pratici:
- Correzione P: errore di 5 deg (~0.087 rad) -> `1.5 × 0.087 ≈ 0.13 rad/s`.
- Integrale: errore costante 1 deg (~0.017 rad) per 1 s -> integrale = `0.017 × 1 = 0.017`; contributo I ≈ `0.4 × 0.017 = 0.0068 rad/s`.
- Feed-forward: se la traiettoria richiede 0.5 rad/s, FF = `0.8 × 0.5 = 0.4 rad/s`; il feedback corregge sopra questo valore.
- Deadband: errori <0.001 rad (~0.057 deg) vengono ignorati per evitare inseguimenti quando il target è gia allineato.
- Saturazione: la norma del comando angolare non supera 1.0 rad/s, proteggendo il robot da rotazioni eccessive.

Suggerimenti:
- Per un orientamento piu rigido, aumenta `kp` fino a notare oscillazioni, poi valuta un `kd` piccolo (0.02-0.05) se le oscillazioni sono dovute a cambi rapidi.
- Se rimane un offset di pochi decimi di grado, alza leggermente `ki` (es. 0.45) controllando che il tempo di assestamento resti accettabile.

### Nota sul filtro derivativo
Con `tau=0.02 s` e un ciclo a 100 Hz (`dt ≈ 0.01 s`), il coefficiente di filtro è:
`alpha = dt / (tau + dt) ≈ 0.01 / 0.03 ≈ 0.33`
Quindi ogni nuova derivata pesa ~33% e il 67% viene dallo storico: utile per smussare rumore su sensori di posizione/orientamento se `kd` viene abilitato.

