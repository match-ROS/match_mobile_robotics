"""
Menu Manager - Handles all interactive terminal menus.
"""

import math
import threading
from typing import TYPE_CHECKING

from .colors import Colors
from .terminal_utils import (
    clear_screen,
    print_header,
    print_menu_item,
    print_info,
    print_success,
    print_error,
    print_warning,
    get_user_input,
    wait_for_key
)
from ..config import AVAILABLE_LINKS, AVAILABLE_OBSTACLES

if TYPE_CHECKING:
    from ..interactive_controller import InteractiveController


class MenuManager:
    """
    Manages all interactive terminal menus.
    
    Each menu method handles display and user interaction for
    a specific feature or submenu.
    """
    
    def __init__(self, controller: 'InteractiveController'):
        """
        Initialize the menu manager.
        
        Args:
            controller: Reference to the main InteractiveController
        """
        self.ctrl = controller
    
    # =========================================================================
    # Helper Methods
    # =========================================================================
    
    def _pose_frame_str(self, pose: dict) -> str:
        """
        Return a compact string describing the pose frames.

        Expected keys (new format): frame_id, ee_frame.
        Backward compatible: if frame_id missing, fallback to controller global_frame.
        """
        if not isinstance(pose, dict):
            return str(self.ctrl.ros.global_frame)
        frame_id = pose.get("frame_id") or self.ctrl.ros.global_frame
        ee_frame = pose.get("ee_frame")
        if ee_frame:
            return f"{frame_id} | {ee_frame}"
        return str(frame_id)

    def _pose_name_with_frame(self, name: str) -> str:
        """Format pose name including frame info (for UI lists)."""
        pose = self.ctrl.poses.get_pose(name)
        if not pose:
            return name
        return f"{name} [{self._pose_frame_str(pose)}]"

    def _print_ee_pose(self):
        """Print current end effector pose."""
        pose = self.ctrl.ros.get_current_ee_pose()
        if pose:
            pos = pose["position"]
            rot = pose["orientation"]
            ee_frame = pose.get("ee_frame") or self.ctrl.ros.ee_frame
            global_frame = pose.get("frame_id") or self.ctrl.ros.global_frame
            print(f"\n  {Colors.BOLD}📍 Posizione End Effector ({ee_frame} in {global_frame}):{Colors.END}")
            print(f"     {Colors.CYAN}Position:    [{pos[0]:>8.4f}, {pos[1]:>8.4f}, {pos[2]:>8.4f}]{Colors.END}")
            print(f"     {Colors.CYAN}Orientation: [{rot[0]:>8.4f}, {rot[1]:>8.4f}, {rot[2]:>8.4f}, {rot[3]:>8.4f}]{Colors.END}")
        else:
            print_error("Impossibile leggere la posizione dell'end effector")
    
    def _get_controller_status_str(self) -> str:
        """Get formatted controller status string."""
        active = self.ctrl.controller_mgr.get_active_controller()
        if active == "velocity":
            return f"{Colors.MAGENTA}VELOCITÀ{Colors.END}"
        elif active == "moveit":
            return f"{Colors.GREEN}MOVEIT{Colors.END}"
        else:
            return f"{Colors.RED}SCONOSCIUTO{Colors.END}"
    
    # =========================================================================
    # Main Menu
    # =========================================================================
    
    def show_main_menu(self) -> str:
        """
        Show the main menu.
        
        Returns:
            User's menu choice
        """
        # Fast refresh: avoid blocking service calls / disk I/O on every render
        self.ctrl.refresh(fast=True)
        
        clear_screen()
        print_header("🤖 CONTROLLO ROBOT INTERATTIVO")
        
        # EE position
        self._print_ee_pose()
        print()
        
        # Controller status
        print_info(f"Controller attivo: {self._get_controller_status_str()}")
        
        # Repulsive status
        rep_enabled = self.ctrl.repulsive.enabled
        status_color = Colors.GREEN if rep_enabled else Colors.RED
        status_text = 'ABILITATA' if rep_enabled else 'DISABILITATA'
        print_info(f"Velocità repulsiva: {status_color}{status_text}{Colors.END}")
        
        link = self.ctrl.repulsive.current_link or '<tutti>'
        obstacle = self.ctrl.repulsive.current_obstacle or '<tutti>'
        print_info(f"Link/Ostacolo: {link} / {obstacle}")
        print()
        
        # Loop status
        loop_enabled = self.ctrl.loop.enabled
        loop_color = Colors.GREEN if loop_enabled else Colors.RED
        loop_text = "IN ESECUZIONE" if loop_enabled else "FERMO"
        print_info(f"Loop movimento: {loop_color}{loop_text}{Colors.END}")
        
        if loop_enabled:
            pose_name = self.ctrl.loop.get_current_pose_name()
            valid_poses = self.ctrl.poses.get_valid_loop_poses()
            if pose_name and valid_poses:
                idx = self.ctrl.loop.current_pose_index
                print_info(f"Posa corrente: {self._pose_name_with_frame(pose_name)} ({idx + 1}/{len(valid_poses)})")
        print()
        
        # Menu items
        print_menu_item(1, "📍 Invia posa predefinita al robot")
        print_menu_item(2, "✏️  Invia posa personalizzata")
        print_menu_item(3, "💾 Salva posizione corrente come nuova posa")
        print_menu_item(4, "🔄 Cambia controller (MoveIt ↔ Velocità)")
        print_menu_item(5, "🛡️  Configura velocità repulsiva")
        print_menu_item(6, "🔁 Movimento in loop")
        print_menu_item(7, "📊 Mostra stato completo")
        print_menu_item(8, "🗑️  Elimina una posa salvata")
        print_menu_item(9, "⚡ Pubblica velocità joint (debug)")
        print_menu_item(10, "🔍 Verifica frame MoveIt (debug)")
        print_menu_item(0, "🚪 Esci")
        
        return get_user_input("Seleziona opzione")

    # =========================================================================
    # Direct Joint Velocity Menu
    # =========================================================================

    def show_direct_joint_velocity_menu(self):
        """Publish direct joint velocity commands to validate the downstream controller."""
        while True:
            clear_screen()
            print_header("⚡ VELOCITÀ JOINT (DEBUG)")

            # Show current wiring info
            controller_node = self.ctrl.ros.controller_node_name
            resolved_topic = self.ctrl.ros.resolve_velocity_command_topic()

            print_info(f"Nodo controller: {Colors.CYAN}{controller_node}{Colors.END}")
            print_info(f"Topic comando (da param): {Colors.CYAN}{resolved_topic or '(non risolto)'}{Colors.END}")
            print()

            # Minimal UI: choose joint INDEX + velocity.
            # We determine N from controller feedback / ros_control `joints` param.
            n_joints = int(self.ctrl.ros.get_command_joint_count(timeout_s=1.0) or 0)

            if n_joints > 0:
                print_info(f"Robot: {Colors.CYAN}{n_joints}{Colors.END} giunti (ordine controller).")
                print_info("Seleziona un giunto per NUMERO e inserisci una velocità (rad/s).")
                print_info("Durata=0 → pubblica finché non premi INVIO.")
                print()

                sel = get_user_input(f"Giunto [1..{n_joints}] (0=esci)")
                if sel == "0":
                    return
                if not sel:
                    print_error("Selezione vuota")
                    wait_for_key()
                    continue

                try:
                    n = int(sel)
                except ValueError:
                    print_error("Giunto non valido (usa solo un numero)")
                    wait_for_key()
                    continue
                if not (1 <= n <= n_joints):
                    print_error(f"Giunto non valido (usa 1..{n_joints})")
                    wait_for_key()
                    continue
                idx = n - 1

                vel_str = get_user_input("Velocità [rad/s] (es. 0.2, -0.2, 0=stop)")
                if vel_str is None or vel_str == "":
                    print_error("Velocità non valida")
                    wait_for_key()
                    continue
                try:
                    v = float(vel_str)
                except ValueError:
                    print_error("Velocità deve essere numerica")
                    wait_for_key()
                    continue

                dur_str = get_user_input("Durata [s] (default: 1, 0=finché premi INVIO)")
                try:
                    duration_s = float(dur_str) if dur_str else 1.0
                except ValueError:
                    print_error("Durata deve essere numerica")
                    wait_for_key()
                    continue

                joint_vel = [0.0 for _ in range(n_joints)]
                joint_vel[idx] = float(v)

                # keep defaults (minimal knobs)
                rate_hz = 50.0
                wait_timeout_s = 2.0
                topic_override = None
                stop_at_end = True
            else:
                # Fallback: keep old "vector entry" mode if we couldn't retrieve joint names
                print_warning("Impossibile ricavare i nomi/ordine dei giunti dal controller.")
                print_info("Fallback: inserisci le velocità come N numeri separati da spazio.")
                print_info("Esempio (UR10): 0 0 0 0.2 0 0")
                print_info("Durata=0 per pubblicare finché non premi INVIO.")
                print()

                vel_line = get_user_input("Velocità giunti (0=esci)")
                if vel_line == "0":
                    return
                if not vel_line:
                    print_error("Input vuoto")
                    wait_for_key()
                    continue

                try:
                    joint_vel = [float(x) for x in vel_line.replace(",", " ").split()]
                except ValueError:
                    print_error("Formato non valido. Usa solo numeri separati da spazio.")
                    wait_for_key()
                    continue

                if not joint_vel:
                    print_error("Nessuna velocità fornita")
                    wait_for_key()
                    continue

                dur_str = get_user_input("Durata [s] (default: 1, 0=finché premi INVIO)")
                try:
                    duration_s = float(dur_str) if dur_str else 1.0
                except ValueError:
                    print_error("Durata deve essere numerica")
                    wait_for_key()
                    continue

                rate_hz = 50.0
                wait_timeout_s = 2.0
                topic_override = None
                stop_at_end = True

            print()
            print_info("Invio comandi di velocità...")

            # Duration==0 -> run until user hits enter
            if duration_s <= 0.0:
                stop_event = threading.Event()
                result = {"ok": True}

                def _worker():
                    ok = self.ctrl.ros.publish_direct_joint_velocity(
                        joint_vel,
                        velocity_command_topic_override=topic_override,
                        rate_hz=rate_hz,
                        duration_s=0.0,
                        wait_for_subscriber_timeout_s=wait_timeout_s,
                        stop_at_end=stop_at_end,
                        stop_event=stop_event,
                    )
                    result["ok"] = bool(ok)

                th = threading.Thread(target=_worker, daemon=True)
                th.start()

                _ = get_user_input("Premi INVIO per fermare")
                stop_event.set()
                th.join(timeout=2.0)

                if result["ok"]:
                    print_success("Publish fermato.")
                else:
                    print_error("Publish fallito o interrotto.")
                wait_for_key()
                return

            ok = self.ctrl.ros.publish_direct_joint_velocity(
                joint_vel,
                velocity_command_topic_override=topic_override,
                rate_hz=rate_hz,
                duration_s=duration_s,
                wait_for_subscriber_timeout_s=wait_timeout_s,
                stop_at_end=stop_at_end,
            )

            if ok:
                print_success("Comando pubblicato.")
            else:
                print_error("Errore durante la pubblicazione.")

            wait_for_key()
            return
    
    # =========================================================================
    # Pose Selection Menu
    # =========================================================================
    
    def show_pose_menu(self):
        """Show saved poses and allow selection."""
        while True:
            clear_screen()
            print_header("📍 POSE SALVATE")
            
            self._print_ee_pose()
            print()
            
            pose_names = self.ctrl.poses.list_poses()
            
            if not pose_names:
                print_warning("Nessuna posa salvata!")
                wait_for_key()
                return
            
            for i, name in enumerate(pose_names, 1):
                pose = self.ctrl.poses.get_pose(name)
                desc = pose.get("description", "") if pose else ""
                pos = pose["position"] if pose else [0, 0, 0]
                frame_str = self._pose_frame_str(pose) if pose else self.ctrl.ros.global_frame
                print_menu_item(i, f"{name.upper()} [{frame_str}]: {desc}")
                print(f"      Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            
            print()
            print_menu_item(0, "↩️  Torna al menù principale")
            
            choice = get_user_input("Seleziona posa")
            
            try:
                idx = int(choice)
                if idx == 0:
                    return
                if 1 <= idx <= len(pose_names):
                    pose_name = pose_names[idx - 1]
                    if self.ctrl.send_pose(pose_name):
                        print_success(f"Posa '{pose_name}' inviata con successo!")
                    else:
                        print_error("Errore nell'invio della posa")
                    wait_for_key()
                else:
                    print_error("Opzione non valida")
                    wait_for_key()
            except ValueError:
                print_error("Input non valido")
                wait_for_key()
    
    # =========================================================================
    # Custom Pose Menu
    # =========================================================================
    
    def show_custom_pose_menu(self):
        """Allow entering a custom pose."""
        while True:
            clear_screen()
            print_header("✏️ POSA PERSONALIZZATA")
            
            self._print_ee_pose()
            print()
            
            print_info("Inserisci le coordinate della posa target.")
            print_info("Premi INVIO per usare il valore di default, '0' per uscire.")
            print()
            
            try:
                x_str = get_user_input("Posizione X [0.4] (0=esci)")
                if x_str == "0":
                    return
                x = float(x_str) if x_str else 0.4
                
                y_str = get_user_input("Posizione Y [0.0]")
                y = float(y_str) if y_str else 0.0
                
                z_str = get_user_input("Posizione Z [0.5]")
                z = float(z_str) if z_str else 0.5
                
                print()
                print_info("Orientazione (quaternione) - default: orientato verso il basso")
                
                use_default = get_user_input("Usare orientazione di default? [S/n]")
                
                if use_default.lower() in ['n', 'no']:
                    ox = float(get_user_input("Quaternione X [0.0]") or "0.0")
                    oy = float(get_user_input("Quaternione Y [0.707]") or "0.707")
                    oz = float(get_user_input("Quaternione Z [0.0]") or "0.0")
                    ow = float(get_user_input("Quaternione W [0.707]") or "0.707")
                else:
                    ox, oy, oz, ow = 0.0, 0.707, 0.0, 0.707
                
                print()
                print_info(f"Invio posa: position=[{x:.3f}, {y:.3f}, {z:.3f}]")
                print_info(f"            orientation=[{ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f}]")
                
                if self.ctrl.send_custom_pose(x, y, z, ox, oy, oz, ow):
                    print_success("Posa personalizzata inviata con successo!")
                else:
                    print_error("Errore nell'invio della posa")
                    
            except ValueError:
                print_error("Valore numerico non valido")
            
            wait_for_key()
    
    # =========================================================================
    # Save Pose Menu
    # =========================================================================
    
    def show_save_pose_menu(self):
        """Save current position as a new pose."""
        while True:
            clear_screen()
            print_header("💾 SALVA POSIZIONE CORRENTE")
            
            self._print_ee_pose()
            print()
            
            existing = self.ctrl.poses.list_poses()
            if existing:
                existing_fmt = [self._pose_name_with_frame(n) for n in existing]
                print_info(f"Pose già salvate: {', '.join(existing_fmt)}")
                print()
            
            name = get_user_input("Nome della nuova posa (0=esci)")
            if name == "0":
                return
            if not name:
                print_error("Nome non valido")
                wait_for_key()
                continue
            
            # Sanitize name
            clean_name = name.replace(" ", "_").lower()
            
            if self.ctrl.poses.pose_exists(clean_name):
                confirm = get_user_input(f"La posa '{clean_name}' esiste già. Sovrascrivere? [s/N]")
                if confirm.lower() not in ['s', 'si', 'yes', 'y']:
                    print_info("Operazione annullata")
                    wait_for_key()
                    continue
            
            description = get_user_input("Descrizione (opzionale)")
            
            if self.ctrl.save_current_pose(clean_name, description):
                print_success(f"Posa '{clean_name}' salvata!")
            else:
                print_error("Errore nel salvataggio della posa")
            
            wait_for_key()
    
    # =========================================================================
    # Controller Switch Menu
    # =========================================================================
    
    def show_controller_switch_menu(self):
        """Switch between velocity and MoveIt controllers."""
        while True:
            clear_screen()
            print_header("🔄 CAMBIO CONTROLLER")
            
            self.ctrl.controller_mgr.update_active_controller()
            names = self.ctrl.controller_mgr.get_controller_names()
            vel_name = names.get("velocity", "joint_group_vel_controller")
            moveit_name = names.get("moveit", "vel_joint_traj_controller")
            
            self._print_ee_pose()
            print()
            
            print_info(f"Controller attuale: {self._get_controller_status_str()}")
            print()
            
            print_menu_item(1, f"🟣 Passa a VELOCITÀ ({vel_name})")
            print_menu_item(2, f"🟢 Passa a MOVEIT ({moveit_name})")
            print()
            print_menu_item(0, "↩️  Torna al menù principale")
            
            choice = get_user_input("Seleziona opzione")
            
            if choice == "0":
                return
            elif choice == "1":
                if self.ctrl.controller_mgr.is_velocity_active():
                    print_info("Controller di velocità già attivo!")
                else:
                    if self.ctrl.controller_mgr.switch_to_velocity():
                        print_success("Passato a controller di velocità!")
                    else:
                        print_error("Errore nel cambio controller")
                wait_for_key()
            elif choice == "2":
                if self.ctrl.controller_mgr.is_moveit_active():
                    print_info("Controller MoveIt già attivo!")
                else:
                    if self.ctrl.controller_mgr.switch_to_moveit():
                        print_success("Passato a controller MoveIt!")
                    else:
                        print_error("Errore nel cambio controller")
                wait_for_key()
            else:
                print_error("Opzione non valida")
                wait_for_key()
    
    # =========================================================================
    # Repulsive Configuration Menu
    # =========================================================================
    
    def show_repulsive_menu(self):
        """Configure repulsive velocity settings."""
        while True:
            # Avoid blocking the UI with service calls on every refresh
            self.ctrl.repulsive.refresh_config(use_service=False)
            
            clear_screen()
            print_header("🛡️ CONFIGURAZIONE VELOCITÀ REPULSIVA")
            
            enabled = self.ctrl.repulsive.enabled
            status = "ABILITATA" if enabled else "DISABILITATA"
            status_color = Colors.GREEN if enabled else Colors.RED
            
            link = self.ctrl.repulsive.current_link or '<tutti>'
            obstacle = self.ctrl.repulsive.current_obstacle or '<tutti>'
            
            print(f"  Stato: {status_color}{Colors.BOLD}{status}{Colors.END}")
            print(f"  Link monitorato: {Colors.CYAN}{link}{Colors.END}")
            print(f"  Ostacolo target: {Colors.CYAN}{obstacle}{Colors.END}")
            print()
            
            if enabled:
                print_menu_item(1, "🔴 Disabilita velocità repulsiva")
            else:
                print_menu_item(1, "🟢 Abilita velocità repulsiva")
            
            print_menu_item(2, "🔗 Seleziona link da monitorare")
            print_menu_item(3, "🎯 Seleziona ostacolo target")
            print_menu_item(4, "📝 Inserisci link/ostacolo manualmente")
            print_menu_item(5, "🔄 Aggiorna stato")
            print()
            print_menu_item(0, "↩️  Torna al menù principale")
            
            choice = get_user_input("Seleziona opzione")
            
            if choice == "0":
                return
            elif choice == "1":
                if enabled:
                    if self.ctrl.repulsive.disable():
                        print_success("Velocità repulsiva disabilitata!")
                    else:
                        print_error("Impossibile disabilitare")
                else:
                    if self.ctrl.repulsive.enable():
                        print_success("Velocità repulsiva abilitata!")
                    else:
                        print_error("Impossibile abilitare")
                wait_for_key()
            elif choice == "2":
                self._select_link_menu()
            elif choice == "3":
                self._select_obstacle_menu()
            elif choice == "4":
                self._manual_link_obstacle_input()
            elif choice == "5":
                self.ctrl.repulsive.refresh_config(use_service=False)
                print_success("Stato aggiornato!")
                wait_for_key()
    
    def _select_link_menu(self):
        """Link selection submenu."""
        clear_screen()
        print_header("🔗 SELEZIONA LINK")
        
        current = self.ctrl.repulsive.current_link
        print_info(f"Link corrente: {current or '<tutti>'}")
        print()
        
        marker = " ← corrente" if not current else ""
        print_menu_item(1, f"<tutti i link>{Colors.GREEN}{marker}{Colors.END}")
        
        for i, link in enumerate(AVAILABLE_LINKS, 2):
            marker = " ← corrente" if link == current else ""
            print_menu_item(i, f"{link}{Colors.GREEN}{marker}{Colors.END}")
        
        print()
        print_menu_item(0, "↩️  Annulla")
        
        choice = get_user_input("Seleziona link")
        
        try:
            idx = int(choice)
            if idx == 0:
                return
            if idx == 1:
                if self.ctrl.repulsive.set_target_link(""):
                    print_success("Impostato: tutti i link")
                wait_for_key()
            elif 2 <= idx <= len(AVAILABLE_LINKS) + 1:
                new_link = AVAILABLE_LINKS[idx - 2]
                if self.ctrl.repulsive.set_target_link(new_link):
                    print_success(f"Link impostato a: {new_link}")
                wait_for_key()
        except ValueError:
            print_error("Input non valido")
            wait_for_key()
    
    def _select_obstacle_menu(self):
        """Obstacle selection submenu."""
        clear_screen()
        print_header("🎯 SELEZIONA OSTACOLO")
        
        current = self.ctrl.repulsive.current_obstacle
        print_info(f"Ostacolo corrente: {current or '<tutti>'}")
        print()
        
        marker = " ← corrente" if not current else ""
        print_menu_item(1, f"<tutti gli ostacoli>{Colors.GREEN}{marker}{Colors.END}")
        
        for i, obstacle in enumerate(AVAILABLE_OBSTACLES, 2):
            marker = " ← corrente" if obstacle == current else ""
            print_menu_item(i, f"{obstacle}{Colors.GREEN}{marker}{Colors.END}")
        
        print()
        print_menu_item(0, "↩️  Annulla")
        
        choice = get_user_input("Seleziona ostacolo")
        
        try:
            idx = int(choice)
            if idx == 0:
                return
            if idx == 1:
                if self.ctrl.repulsive.set_target_object(""):
                    print_success("Impostato: tutti gli ostacoli")
                wait_for_key()
            elif 2 <= idx <= len(AVAILABLE_OBSTACLES) + 1:
                new_obstacle = AVAILABLE_OBSTACLES[idx - 2]
                if self.ctrl.repulsive.set_target_object(new_obstacle):
                    print_success(f"Ostacolo impostato a: {new_obstacle}")
                wait_for_key()
        except ValueError:
            print_error("Input non valido")
            wait_for_key()
    
    def _manual_link_obstacle_input(self):
        """Manual link/obstacle input."""
        clear_screen()
        print_header("📝 INPUT MANUALE")
        
        current_link = self.ctrl.repulsive.current_link
        current_obstacle = self.ctrl.repulsive.current_obstacle
        
        print_info(f"Valori correnti:")
        print_info(f"  Link: {current_link or '<tutti>'}")
        print_info(f"  Ostacolo: {current_obstacle or '<tutti>'}")
        print()
        print_info("Premi INVIO per mantenere il valore corrente.")
        print_info("Inserisci '-' per selezionare tutti.")
        print()
        
        new_link = get_user_input(f"Nuovo link [{current_link or '<tutti>'}]")
        if new_link == "-":
            new_link = ""
        elif not new_link:
            new_link = current_link
        
        new_obstacle = get_user_input(f"Nuovo ostacolo [{current_obstacle or '<tutti>'}]")
        if new_obstacle == "-":
            new_obstacle = ""
        elif not new_obstacle:
            new_obstacle = current_obstacle
        
        link_ok = self.ctrl.repulsive.set_target_link(new_link)
        obj_ok = self.ctrl.repulsive.set_target_object(new_obstacle)
        
        if link_ok and obj_ok:
            print_success("Parametri aggiornati!")
        else:
            print_warning("Alcuni parametri potrebbero non essere stati aggiornati")
        
        wait_for_key()
    
    # =========================================================================
    # Loop Movement Menu
    # =========================================================================
    
    def show_loop_menu(self):
        """Configure and control loop movement."""
        while True:
            self.ctrl.poses.load()  # Refresh
            
            clear_screen()
            print_header("🔁 MOVIMENTO IN LOOP")
            
            # Status
            enabled = self.ctrl.loop.enabled
            status = "IN ESECUZIONE" if enabled else "FERMO"
            status_color = Colors.GREEN if enabled else Colors.RED
            print(f"  Stato: {status_color}{Colors.BOLD}{status}{Colors.END}")
            print()
            
            # Configured poses
            valid_poses = self.ctrl.poses.get_valid_loop_poses()
            
            print(f"  {Colors.BOLD}Pose nel loop:{Colors.END}")
            if valid_poses:
                current_idx = self.ctrl.loop.current_pose_index
                for i, pose_name in enumerate(valid_poses):
                    marker = " ← corrente" if enabled and i == current_idx % len(valid_poses) else ""
                    print(f"    {i+1}. {Colors.CYAN}{pose_name}{Colors.GREEN}{marker}{Colors.END}")
            else:
                print(f"    {Colors.YELLOW}(nessuna posa configurata){Colors.END}")
            print()
            
            # Parameters
            pos_tol = self.ctrl.poses.get_position_tolerance()
            orient_tol = self.ctrl.poses.get_orientation_tolerance()
            dwell = self.ctrl.poses.get_dwell_time()
            
            print(f"  {Colors.BOLD}Parametri:{Colors.END}")
            print(f"    Tolleranza posizione: {Colors.CYAN}{pos_tol:.3f} m{Colors.END}")
            print(f"    Tolleranza orientazione: {Colors.CYAN}{orient_tol:.3f} rad{Colors.END}")
            print(f"    ({math.degrees(orient_tol):.1f}°)")
            print(f"    Tempo di attesa: {Colors.CYAN}{dwell:.2f} s{Colors.END}")
            print()
            
            # Show current error if in loop
            if enabled and valid_poses:
                pose_name = self.ctrl.loop.get_current_pose_name()
                if pose_name:
                    target_pose = self.ctrl.poses.get_pose(pose_name)
                    if target_pose:
                        errors = self.ctrl.loop.get_current_errors(target_pose)
                        if errors:
                            pos_err, orient_err = errors
                            print(f"  {Colors.BOLD}Errore corrente:{Colors.END}")
                            pos_color = Colors.GREEN if pos_err < pos_tol else Colors.YELLOW
                            orient_color = Colors.GREEN if orient_err < orient_tol else Colors.YELLOW
                            print(f"    Posizione: {pos_color}{pos_err:.4f} m{Colors.END}")
                            print(f"    Orientazione: {orient_color}{orient_err:.4f} rad ({math.degrees(orient_err):.1f}°){Colors.END}")
                            print()
            
            # Menu options
            if enabled:
                print_menu_item(1, "🛑 Ferma loop")
            else:
                print_menu_item(1, "▶️  Avvia loop")
            
            print_menu_item(2, "📋 Configura pose nel loop")
            print_menu_item(3, "⚙️  Imposta tolleranze")
            print_menu_item(4, "⏱️  Imposta tempo di attesa")
            print()
            print_menu_item(0, "↩️  Torna al menù principale")
            
            choice = get_user_input("Seleziona opzione")
            
            if choice == "0":
                return
            elif choice == "1":
                if enabled:
                    if self.ctrl.loop.stop():
                        print_success("Loop fermato!")
                    else:
                        print_error("Errore nel fermare il loop")
                else:
                    if self.ctrl.start_loop():
                        print_success("Loop avviato!")
                    else:
                        print_error("Errore nell'avviare il loop")
                wait_for_key()
            elif choice == "2":
                self._configure_loop_poses_menu()
            elif choice == "3":
                self._configure_loop_tolerances_menu()
            elif choice == "4":
                self._configure_loop_dwell_menu()
    
    def _configure_loop_poses_menu(self):
        """Configure poses for loop movement."""
        clear_screen()
        print_header("📋 CONFIGURA POSE NEL LOOP")
        
        print_info("Pose disponibili:")
        pose_names = self.ctrl.poses.list_poses()
        current_loop = self.ctrl.poses.get_loop_poses()
        
        for i, name in enumerate(pose_names, 1):
            in_loop = " ✓" if name in current_loop else ""
            print(f"    {i}. {self._pose_name_with_frame(name)}{Colors.GREEN}{in_loop}{Colors.END}")
        print()
        
        print_info("Inserisci i numeri delle pose separate da virgola")
        print_info("Esempio: 1,3,2,4 per eseguire nell'ordine indicato")
        print_info("Premi INVIO per mantenere la configurazione attuale")
        print()
        
        selection = get_user_input("Selezione (0=annulla)")
        
        if selection == "0" or not selection:
            return
        
        try:
            indices = [int(x.strip()) for x in selection.split(',')]
            new_poses = []
            for idx in indices:
                if 1 <= idx <= len(pose_names):
                    new_poses.append(pose_names[idx - 1])
                else:
                    print_warning(f"Indice {idx} non valido, ignorato")
            
            if new_poses:
                if self.ctrl.poses.set_loop_poses(new_poses):
                    print_success(f"Configurate {len(new_poses)} pose nel loop: {', '.join(new_poses)}")
                else:
                    print_error("Errore nel salvataggio")
            else:
                print_error("Nessuna posa valida selezionata")
                
        except ValueError:
            print_error("Input non valido. Usa numeri separati da virgola.")
        
        wait_for_key()
    
    def _configure_loop_tolerances_menu(self):
        """Configure loop tolerances."""
        clear_screen()
        print_header("⚙️ CONFIGURA TOLLERANZE")
        
        pos_tol = self.ctrl.poses.get_position_tolerance()
        orient_tol = self.ctrl.poses.get_orientation_tolerance()
        
        print_info(f"Valori correnti:")
        print_info(f"  Tolleranza posizione: {pos_tol:.3f} m")
        print_info(f"  Tolleranza orientazione: {orient_tol:.3f} rad")
        print()
        
        try:
            pos_str = get_user_input(f"Tolleranza posizione in metri [{pos_tol:.3f}]")
            if pos_str:
                pos_tol = float(pos_str)
            
            orient_str = get_user_input(f"Tolleranza orientazione in radianti [{orient_tol:.3f}]")
            if orient_str:
                orient_tol = float(orient_str)
            
            if self.ctrl.poses.set_loop_tolerances(pos_tol, orient_tol):
                print_success(f"Tolleranze impostate: pos={pos_tol:.3f}m, orient={orient_tol:.3f}rad")
            else:
                print_error("Errore nel salvataggio")
                
        except ValueError:
            print_error("Valore numerico non valido")
        
        wait_for_key()
    
    def _configure_loop_dwell_menu(self):
        """Configure dwell time."""
        clear_screen()
        print_header("⏱️ CONFIGURA TEMPO DI ATTESA")
        
        dwell = self.ctrl.poses.get_dwell_time()
        
        print_info(f"Valore corrente: {dwell:.2f} secondi")
        print_info("Il robot attenderà questo tempo su ogni posa prima di passare alla successiva")
        print()
        
        try:
            dwell_str = get_user_input(f"Tempo di attesa in secondi [{dwell:.2f}]")
            if dwell_str:
                new_dwell = float(dwell_str)
                if new_dwell < 0:
                    print_error("Il tempo deve essere >= 0")
                elif self.ctrl.poses.set_loop_dwell_time(new_dwell):
                    print_success(f"Tempo di attesa impostato a {new_dwell:.2f} secondi")
                else:
                    print_error("Errore nel salvataggio")
        except ValueError:
            print_error("Valore numerico non valido")
        
        wait_for_key()
    
    # =========================================================================
    # Delete Pose Menu
    # =========================================================================
    
    def show_delete_pose_menu(self):
        """Delete a saved pose."""
        while True:
            self.ctrl.poses.load()
            
            clear_screen()
            print_header("🗑️ ELIMINA POSA")
            
            pose_names = self.ctrl.poses.list_poses()
            
            if not pose_names:
                print_warning("Nessuna posa salvata!")
                wait_for_key()
                return
            
            for i, name in enumerate(pose_names, 1):
                pose = self.ctrl.poses.get_pose(name)
                desc = pose.get("description", "") if pose else ""
                label = self._pose_name_with_frame(name)
                print_menu_item(i, f"{label}: {desc}")
            
            print()
            print_menu_item(0, "↩️  Torna al menù principale")
            
            choice = get_user_input("Seleziona posa da eliminare")
            
            try:
                idx = int(choice)
                if idx == 0:
                    return
                if 1 <= idx <= len(pose_names):
                    pose_name = pose_names[idx - 1]
                    confirm = get_user_input(f"Eliminare '{pose_name}'? [s/N]")
                    if confirm.lower() in ['s', 'si', 'yes', 'y']:
                        if self.ctrl.poses.delete_pose(pose_name):
                            print_success(f"Posa '{pose_name}' eliminata!")
                        else:
                            print_error("Errore nell'eliminazione")
                    else:
                        print_info("Operazione annullata")
                    wait_for_key()
                else:
                    print_error("Opzione non valida")
                    wait_for_key()
            except ValueError:
                print_error("Input non valido")
                wait_for_key()
    
    # =========================================================================
    # Status Menu
    # =========================================================================
    
    def show_status(self):
        """Show complete system status."""
        clear_screen()
        print_header("📊 STATO DEL SISTEMA")
        
        # Full refresh: include controller_manager/services and poses reload
        self.ctrl.refresh(fast=False)
        
        # EE pose
        self._print_ee_pose()
        print()
        
        # Controller
        print(f"  {Colors.BOLD}Controller:{Colors.END}")
        print(f"    Attivo: {self._get_controller_status_str()}")
        print()
        
        # Repulsive
        print(f"  {Colors.BOLD}Velocità Repulsiva:{Colors.END}")
        enabled = self.ctrl.repulsive.enabled
        status = "ABILITATA" if enabled else "DISABILITATA"
        status_color = Colors.GREEN if enabled else Colors.RED
        print(f"    Stato: {status_color}{status}{Colors.END}")
        print(f"    Link monitorato: {Colors.CYAN}{self.ctrl.repulsive.current_link or '<tutti>'}{Colors.END}")
        print(f"    Ostacolo target: {Colors.CYAN}{self.ctrl.repulsive.current_obstacle or '<tutti>'}{Colors.END}")
        print()
        
        # ROS config
        print(f"  {Colors.BOLD}Configurazione ROS:{Colors.END}")
        print(f"    Frame globale: {Colors.CYAN}{self.ctrl.ros.global_frame}{Colors.END}")
        print(f"    Frame EE: {Colors.CYAN}{self.ctrl.ros.ee_frame}{Colors.END}")
        print(f"    Nodo controller: {Colors.CYAN}{self.ctrl.ros.controller_node_name}{Colors.END}")
        print(f"    File pose: {Colors.CYAN}{self.ctrl.poses.poses_file_path}{Colors.END}")
        print()
        
        # Saved poses
        pose_count = self.ctrl.poses.get_pose_count()
        print(f"  {Colors.BOLD}Pose Salvate ({pose_count}):{Colors.END}")
        for name in self.ctrl.poses.list_poses():
            print(f"    • {self._pose_name_with_frame(name)}")
        print()
        
        # Loop movement
        print(f"  {Colors.BOLD}Movimento in Loop:{Colors.END}")
        loop_enabled = self.ctrl.loop.enabled
        loop_status = "IN ESECUZIONE" if loop_enabled else "FERMO"
        loop_color = Colors.GREEN if loop_enabled else Colors.RED
        print(f"    Stato: {loop_color}{loop_status}{Colors.END}")
        
        valid_poses = self.ctrl.poses.get_valid_loop_poses()
        print(f"    Pose configurate: {Colors.CYAN}{', '.join(valid_poses) if valid_poses else '(nessuna)'}{Colors.END}")
        
        pos_tol = self.ctrl.poses.get_position_tolerance()
        orient_tol = self.ctrl.poses.get_orientation_tolerance()
        dwell = self.ctrl.poses.get_dwell_time()
        
        print(f"    Tolleranza posizione: {Colors.CYAN}{pos_tol:.3f} m{Colors.END}")
        print(f"    Tolleranza orientazione: {Colors.CYAN}{orient_tol:.3f} rad ({math.degrees(orient_tol):.1f}°){Colors.END}")
        print(f"    Tempo di attesa: {Colors.CYAN}{dwell:.2f} s{Colors.END}")
        
        wait_for_key()

    # =========================================================================
    # Frame Info Debug Menu
    # =========================================================================

    def show_frame_info_menu(self):
        """Show frame information from the GetFrameInfo service for debugging."""
        import rospy
        clear_screen()
        print_header("🔍 VERIFICA FRAME MOVEIT")
        
        print_info("Chiamata al servizio get_frame_info...")
        print()
        
        try:
            from cartesian_velocity_controller.srv import GetFrameInfo
            
            # Build service name from controller node
            controller_node = self.ctrl.ros.controller_node_name
            service_name = f"{controller_node}/get_frame_info"
            
            rospy.wait_for_service(service_name, timeout=5.0)
            get_frame_info = rospy.ServiceProxy(service_name, GetFrameInfo)
            resp = get_frame_info()
            
            # Display results
            print(f"  {Colors.BOLD}Configurazione Frame:{Colors.END}")
            print(f"    Model root frame (URDF): {Colors.CYAN}{resp.model_root_frame}{Colors.END}")
            print(f"    Config global_frame:     {Colors.CYAN}{resp.config_global_frame}{Colors.END}")
            print(f"    TCP link:                {Colors.CYAN}{resp.tcp_link}{Colors.END}")
            print(f"    Group name:              {Colors.CYAN}{resp.group_name}{Colors.END}")
            print()
            
            # Frame match check
            if resp.frames_match:
                print(f"  {Colors.GREEN}{Colors.BOLD}✓ I FRAME COINCIDONO{Colors.END}")
                print(f"    {Colors.GREEN}Il global_frame e il model root frame sono allineati.{Colors.END}")
            else:
                print(f"  {Colors.RED}{Colors.BOLD}✗ FRAME NON COINCIDENTI!{Colors.END}")
                print(f"    {Colors.RED}Questa potrebbe essere la causa del problema di rotazione.{Colors.END}")
            print()
            
            # TF transform info
            print(f"  {Colors.BOLD}Trasformazione TF (model_root → global_frame):{Colors.END}")
            if resp.tf_available:
                t = resp.tf_model_to_global.translation
                r = resp.tf_model_to_global.rotation
                print(f"    Translation: [{t.x:.4f}, {t.y:.4f}, {t.z:.4f}]")
                print(f"    Rotation:    [w={r.w:.4f}, x={r.x:.4f}, y={r.y:.4f}, z={r.z:.4f}]")
            else:
                print(f"    {Colors.YELLOW}TF non disponibile tra i due frame{Colors.END}")
            print()
            
            # TCP pose
            print(f"  {Colors.BOLD}TCP Pose (in model frame via FK):{Colors.END}")
            p = resp.tcp_pose_in_model_frame.position
            o = resp.tcp_pose_in_model_frame.orientation
            print(f"    Position:    [{p.x:.4f}, {p.y:.4f}, {p.z:.4f}]")
            print(f"    Orientation: [w={o.w:.4f}, x={o.x:.4f}, y={o.y:.4f}, z={o.z:.4f}]")
            print()
            
            # Joint info
            if resp.joint_names:
                print(f"  {Colors.BOLD}Joint State ({len(resp.joint_names)} joints):{Colors.END}")
                for i, (name, pos) in enumerate(zip(resp.joint_names, resp.joint_positions)):
                    print(f"    {i+1}. {name}: {pos:.4f} rad ({math.degrees(pos):.1f}°)")
            
        except rospy.ROSException as e:
            print_error(f"Servizio non disponibile: {e}")
        except Exception as e:
            print_error(f"Errore: {e}")
        
        print()
        wait_for_key()
