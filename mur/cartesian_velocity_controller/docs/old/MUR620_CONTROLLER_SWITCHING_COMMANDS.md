# MUR620 — Comandi terminale per lo switching dei controller (ROS1)

Questo documento raccoglie i comandi utili per **listare** e **switchare** i controller dei due bracci sul namespace `mur620`.

> Nota: nel bringup i controller `joint_group_vel_controller_{l,r}/unsafe` risultano tipicamente **loaded ma stopped**.

## 1) Verificare che i servizi esistano

```bash
rosservice list | grep controller_manager
```

Se il robot gira sotto `/mur620`, i servizi attesi sono (tra gli altri):

- `/mur620/controller_manager/list_controllers`
- `/mur620/controller_manager/switch_controller`

## 2) Listare i controller e il loro stato

```bash
rosservice call /mur620/controller_manager/list_controllers "{}"
```

Cerca nello stdout i controller:

- `joint_group_vel_controller_l/unsafe`
- `joint_group_vel_controller_r/unsafe`
- `UR10_l/arm_controller`
- `UR10_r/arm_controller`

e controlla lo stato `running` / `stopped`.

## 3) Passare a velocity control (braccio sinistro)

Stop del controller MoveIt + start del controller velocity:

```bash
rosservice call /mur620/controller_manager/switch_controller "{
  start_controllers: ['joint_group_vel_controller_l/unsafe'],
  stop_controllers:  ['UR10_l/arm_controller'],
  strictness: 2,
  start_asap: false,
  timeout: 0.0
}"
```

## 4) Passare a velocity control (braccio destro)

```bash
rosservice call /mur620/controller_manager/switch_controller "{
  start_controllers: ['joint_group_vel_controller_r/unsafe'],
  stop_controllers:  ['UR10_r/arm_controller'],
  strictness: 2,
  start_asap: false,
  timeout: 0.0
}"
```

## 5) Tornare a MoveIt (braccio sinistro)

```bash
rosservice call /mur620/controller_manager/switch_controller "{
  start_controllers: ['UR10_l/arm_controller'],
  stop_controllers:  ['joint_group_vel_controller_l/unsafe'],
  strictness: 2,
  start_asap: false,
  timeout: 0.0
}"
```

## 6) Tornare a MoveIt (braccio destro)

```bash
rosservice call /mur620/controller_manager/switch_controller "{
  start_controllers: ['UR10_r/arm_controller'],
  stop_controllers:  ['joint_group_vel_controller_r/unsafe'],
  strictness: 2,
  start_asap: false,
  timeout: 0.0
}"
```

## 7) Verifica rapida dei topic di comando

Dopo lo switch a velocity control, i topic che dovrebbero ricevere il comando sono:

- `/mur620/joint_group_vel_controller_l/unsafe/command`
- `/mur620/joint_group_vel_controller_r/unsafe/command`

Esempi:

```bash
rostopic info /mur620/joint_group_vel_controller_l/unsafe/command
rostopic info /mur620/joint_group_vel_controller_r/unsafe/command
```

