# Come leggere il debug del Passivity Layer

Questo documento riassume come leggere il topic di debug del `PassivityLayer` durante i test.

## Topic da usare

Il debug e pubblicato nel namespace del nodo master:

- setup singolo: `/<nome_nodo>/debug/passivity_stats`
- setup duale sinistro: `/teleop_master_haptic_controller_left/debug/passivity_stats`
- setup duale destro: `/teleop_master_haptic_controller_right/debug/passivity_stats`

Esempi:

```bash
rostopic echo /teleop_master_haptic_controller_left/debug/passivity_stats
rostopic echo /teleop_master_haptic_controller_right/debug/passivity_stats
```

## Ordine dei campi

Il messaggio e un `std_msgs/Float64MultiArray` con questi valori:

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

## Come interpretarli

- `gamma_applied = 1.0`: il layer non sta limitando il feedback.
- `gamma_applied < 1.0`: il layer sta riducendo forza e, se attivo, anche coppia.
- Se imposti `passivity/gamma_min > 0`, `gamma_applied` non scendera mai sotto quel valore.
- Se `gamma_raw < gamma_applied`, il floor `gamma_min` sta impedendo al layer di tagliare ulteriormente.
- `energy_after` vicino a `tank_energy_min`: il tank e quasi scarico e il feedback puo essere tagliato.
- `power_out_requested` alta con `power_diss` bassa: il sistema sta spendendo energia piu velocemente di quanto la recuperi.
- `|F_reflection_used|` molto minore di `|F_reflection_requested|`: limitazione evidente sul contributo di forza riflessa.

## Lettura pratica veloce

Se vuoi capire in pochi secondi cosa sta succedendo:

1. guarda `gamma_applied`
2. guarda `energy_after`
3. confronta `power_out_requested` con `power_diss`
4. confronta `|F_reflection_requested|` con `|F_reflection_used|`

## Regole pratiche

- Se il master oscilla ancora, il layer e probabilmente troppo permissivo.
- Se il feedback sembra troppo debole o "morto", il layer sta probabilmente tagliando troppo presto o recuperando troppo lentamente.
- `gamma_min > 0` puo aiutare a non perdere completamente il contatto, ma riduce la garanzia di passivita stretta.
