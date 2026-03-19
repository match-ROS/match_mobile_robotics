# Come leggere il debug `admittance_dynamics`

Questo documento riassume come leggere il topic di debug `admittance_dynamics` del nodo `teleop_master_haptic_controller`.

## Topic da usare

Il topic e pubblicato nel namespace privato del nodo master:

- setup singolo: `/<nome_nodo>/debug/admittance_dynamics`
- setup duale sinistro: `/teleop_master_haptic_controller_left/debug/admittance_dynamics`
- setup duale destro: `/teleop_master_haptic_controller_right/debug/admittance_dynamics`

Esempi:

```bash
rostopic echo /teleop_master_haptic_controller_left/debug/admittance_dynamics
rostopic echo /teleop_master_haptic_controller_right/debug/admittance_dynamics
```

## Cosa rappresenta il messaggio

Il messaggio e un `std_msgs/Float64MultiArray` con 36 valori.

I valori sono organizzati in 12 gruppi da 3 elementi:

- asse `x`
- asse `y`
- asse `z`

Gli assi sono espressi nel frame usato dal controller per il wrench, cioe `wrench_target_frame`.

## Significato generale

- `base`: valore statico letto dai parametri `mass_*`, `damping_*` oppure dagli override `*_xyz`
- `extra`: contributo dinamico aggiunto dal scheduler di `dynamic_mass` o `dynamic_damping`
- `eff`: valore effettivo finale usato dal controller nel calcolo dell'ammettenza

In pratica:

- `M_eff = M_base + M_extra`
- `D_eff = D_base + D_extra`

Se la dinamica adattiva e disabilitata, oppure non sta aggiungendo nulla, i campi `extra` valgono circa zero e `eff` coincide con `base`.

## Ordine dei campi

1. `M_lin_base.x` `[kg]`: massa virtuale lineare statica asse x
2. `M_lin_base.y` `[kg]`: massa virtuale lineare statica asse y
3. `M_lin_base.z` `[kg]`: massa virtuale lineare statica asse z
4. `M_lin_extra.x` `[kg]`: massa virtuale lineare aggiunta dinamicamente asse x
5. `M_lin_extra.y` `[kg]`: massa virtuale lineare aggiunta dinamicamente asse y
6. `M_lin_extra.z` `[kg]`: massa virtuale lineare aggiunta dinamicamente asse z
7. `M_lin_eff.x` `[kg]`: massa virtuale lineare effettiva asse x
8. `M_lin_eff.y` `[kg]`: massa virtuale lineare effettiva asse y
9. `M_lin_eff.z` `[kg]`: massa virtuale lineare effettiva asse z
10. `D_lin_base.x` `[N*s/m]`: smorzamento lineare statico asse x
11. `D_lin_base.y` `[N*s/m]`: smorzamento lineare statico asse y
12. `D_lin_base.z` `[N*s/m]`: smorzamento lineare statico asse z
13. `D_lin_extra.x` `[N*s/m]`: smorzamento lineare aggiunto dinamicamente asse x
14. `D_lin_extra.y` `[N*s/m]`: smorzamento lineare aggiunto dinamicamente asse y
15. `D_lin_extra.z` `[N*s/m]`: smorzamento lineare aggiunto dinamicamente asse z
16. `D_lin_eff.x` `[N*s/m]`: smorzamento lineare effettivo asse x
17. `D_lin_eff.y` `[N*s/m]`: smorzamento lineare effettivo asse y
18. `D_lin_eff.z` `[N*s/m]`: smorzamento lineare effettivo asse z
19. `M_ang_base.x` `[kg*m^2]`: massa virtuale angolare statica asse x
20. `M_ang_base.y` `[kg*m^2]`: massa virtuale angolare statica asse y
21. `M_ang_base.z` `[kg*m^2]`: massa virtuale angolare statica asse z
22. `M_ang_extra.x` `[kg*m^2]`: massa virtuale angolare aggiunta dinamicamente asse x
23. `M_ang_extra.y` `[kg*m^2]`: massa virtuale angolare aggiunta dinamicamente asse y
24. `M_ang_extra.z` `[kg*m^2]`: massa virtuale angolare aggiunta dinamicamente asse z
25. `M_ang_eff.x` `[kg*m^2]`: massa virtuale angolare effettiva asse x
26. `M_ang_eff.y` `[kg*m^2]`: massa virtuale angolare effettiva asse y
27. `M_ang_eff.z` `[kg*m^2]`: massa virtuale angolare effettiva asse z
28. `D_ang_base.x` `[N*m*s/rad]`: smorzamento angolare statico asse x
29. `D_ang_base.y` `[N*m*s/rad]`: smorzamento angolare statico asse y
30. `D_ang_base.z` `[N*m*s/rad]`: smorzamento angolare statico asse z
31. `D_ang_extra.x` `[N*m*s/rad]`: smorzamento angolare aggiunto dinamicamente asse x
32. `D_ang_extra.y` `[N*m*s/rad]`: smorzamento angolare aggiunto dinamicamente asse y
33. `D_ang_extra.z` `[N*m*s/rad]`: smorzamento angolare aggiunto dinamicamente asse z
34. `D_ang_eff.x` `[N*m*s/rad]`: smorzamento angolare effettivo asse x
35. `D_ang_eff.y` `[N*m*s/rad]`: smorzamento angolare effettivo asse y
36. `D_ang_eff.z` `[N*m*s/rad]`: smorzamento angolare effettivo asse z

## Lettura pratica veloce

Se vuoi capire subito cosa sta succedendo:

1. confronta `*_extra` con zero
2. confronta `*_eff` con `*_base`
3. verifica se l'aumento e isotropo oppure diverso tra `x`, `y`, `z`

## Regole pratiche

- Se `M_lin_extra` cresce, il master diventa piu "pesante" in traslazione e tende a reagire meno bruscamente.
- Se `D_lin_extra` cresce, il master diventa piu smorzato e tende a dissipare di piu.
- Se `M_ang_extra` o `D_ang_extra` restano a zero, probabilmente la parte angolare dinamica e disabilitata oppure non e ancora entrata in azione.
- Se `eff` e sempre uguale a `base`, il scheduling dinamico non sta contribuendo in quel momento.
