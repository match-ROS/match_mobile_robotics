#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive menu handlers for scene control.
"""

from typing import List, Optional, Callable

from .utils import (
    Colors, clear_screen, print_header, print_subheader,
    print_menu_item, print_info, print_success, print_error,
    print_warning, get_user_input, wait_for_key
)
from .object_manager import ObjectInfo, ObjectStateManager, PRIMITIVE_TYPES


class MenuUI:
    """Handles all interactive menu displays and user interactions."""
    
    def __init__(self, state_manager: ObjectStateManager):
        """Initialize menu UI.
        
        Args:
            state_manager: Object state manager reference
        """
        self.state_manager = state_manager
        
        # Callbacks to be set by controller
        self.on_add_object: Optional[Callable] = None
        self.on_remove_object: Optional[Callable] = None
        self.on_disable_object: Optional[Callable] = None
        self.on_enable_object: Optional[Callable] = None
        self.on_move_object: Optional[Callable] = None
        self.on_start_loop: Optional[Callable] = None
        self.on_stop_loop: Optional[Callable] = None
        self.on_is_loop_running: Optional[Callable] = None
        self.on_add_waypoint: Optional[Callable] = None
        self.on_remove_waypoint: Optional[Callable] = None
        self.on_clear_waypoints: Optional[Callable] = None
        self.on_refresh: Optional[Callable] = None
        self.on_get_distance_info: Optional[Callable] = None
    
    def show_main_menu(self) -> str:
        """Display main menu and return user choice.
        
        Returns:
            User's menu selection as string
        """
        if self.on_refresh:
            self.on_refresh()
        
        clear_screen()
        print_header("🎭 CONTROLLO SCENA INTERATTIVO")
        
        # Show scene summary
        active_count = sum(1 for o in self.state_manager.objects.values() if o.enabled)
        disabled_count = len(self.state_manager.disabled_objects)
        sequences_count = sum(1 for o in self.state_manager.objects.values() if o.loop_waypoints)
        
        loop_count = 0
        if self.on_is_loop_running:
            loop_count = sum(1 for o in self.state_manager.objects.values() 
                           if self.on_is_loop_running(o.id))
        
        print(f"  {Colors.BOLD}Stato Scena:{Colors.END}")
        print(f"    Oggetti attivi: {Colors.GREEN}{active_count}{Colors.END}")
        print(f"    Oggetti disabilitati: {Colors.YELLOW}{disabled_count}{Colors.END}")
        print(f"    Sequenze configurate: {Colors.CYAN}{sequences_count}{Colors.END}")
        print(f"    Loop in esecuzione: {Colors.MAGENTA}{loop_count}{Colors.END}")
        print(f"    Frame globale: {Colors.CYAN}{self.state_manager.global_frame}{Colors.END}")
        print()
        
        print_menu_item(1, "📋 Visualizza stato oggetti")
        print_menu_item(2, "➕ Aggiungi nuovo oggetto")
        print_menu_item(3, "➖ Rimuovi oggetto")
        print_menu_item(4, "🔛 Abilita/Disabilita oggetto")
        print_menu_item(5, "🚀 Muovi oggetto")
        print_menu_item(6, "🔁 Configura movimento in loop")
        print_menu_item(7, "📊 Visualizza distanze")
        print_menu_item(8, "🔄 Aggiorna dalla scena")
        print_menu_item(0, "🚪 Esci")
        
        return get_user_input("Seleziona opzione")
    
    def show_objects_status(self):
        """Display detailed object status."""
        while True:
            if self.on_refresh:
                self.on_refresh()
            
            clear_screen()
            print_header("📋 STATO OGGETTI")
            
            if not self.state_manager.objects and not self.state_manager.disabled_objects:
                print_warning("Nessun oggetto presente nella scena")
                wait_for_key()
                return
            
            # Active objects
            print_subheader("Oggetti Attivi")
            active_objects = [o for o in self.state_manager.objects.values() if o.enabled]
            
            if not active_objects:
                print_info("Nessun oggetto attivo")
            else:
                for i, obj in enumerate(active_objects, 1):
                    loop_status = ""
                    if self.on_is_loop_running and self.on_is_loop_running(obj.id):
                        loop_status = f" {Colors.GREEN}[LOOP ATTIVO]{Colors.END}"
                    elif obj.loop_waypoints:
                        loop_status = f" {Colors.YELLOW}[{len(obj.loop_waypoints)} waypoints configurati]{Colors.END}"
                    
                    print(f"\n  {Colors.GREEN}[{i}]{Colors.END} {Colors.BOLD}{obj.id}{Colors.END}{loop_status}")
                    print(f"      Tipo: {Colors.CYAN}{obj.primitive_type}{Colors.END}")
                    print(f"      Dimensioni: {Colors.CYAN}{obj.dimensions}{Colors.END}")
                    print(f"      Posizione: {Colors.CYAN}[{obj.position[0]:.3f}, {obj.position[1]:.3f}, {obj.position[2]:.3f}]{Colors.END}")
                    print(f"      Frame: {Colors.CYAN}{obj.frame_id}{Colors.END}")
                    
                    if obj.loop_waypoints:
                        print(f"      Sequenza movimento: {Colors.MAGENTA}{len(obj.loop_waypoints)} waypoints{Colors.END} (loop={'SI' if obj.loop_enabled else 'NO'})")
            
            # Disabled objects
            if self.state_manager.disabled_objects:
                print_subheader("Oggetti Disabilitati")
                for obj_id in self.state_manager.disabled_objects.keys():
                    print(f"  {Colors.DIM}• {obj_id}{Colors.END}")
            
            print()
            print_menu_item('r', "🔄 Aggiorna")
            print_menu_item(0, "↩️  Torna al menù principale")
            
            choice = get_user_input("Seleziona opzione")
            
            if choice == "0":
                return
            elif choice.lower() == "r":
                continue
    
    def show_add_object_menu(self):
        """Menu for adding a new object."""
        clear_screen()
        print_header("➕ AGGIUNGI NUOVO OGGETTO")
        
        # Ask for ID
        obj_id = get_user_input("ID dell'oggetto (es. 'box_1')")
        if not obj_id:
            print_error("ID non valido")
            wait_for_key()
            return
        
        if obj_id in self.state_manager.objects:
            confirm = get_user_input(f"L'oggetto '{obj_id}' esiste già. Sovrascrivere? [s/N]")
            if confirm.lower() not in ['s', 'si', 'yes', 'y']:
                return
        
        # Ask for type
        print()
        print_info("Tipi disponibili:")
        print_menu_item(1, "box (parallelepipedo)")
        print_menu_item(2, "sphere (sfera)")
        print_menu_item(3, "cylinder (cilindro)")
        
        type_choice = get_user_input("Seleziona tipo")
        
        primitive_type = ""
        dimensions = []
        
        try:
            if type_choice == "1":
                primitive_type = "box"
                print()
                print_info("Inserisci le dimensioni del box [x, y, z] in metri")
                x = float(get_user_input("Dimensione X [0.1]") or "0.1")
                y = float(get_user_input("Dimensione Y [0.1]") or "0.1")
                z = float(get_user_input("Dimensione Z [0.1]") or "0.1")
                dimensions = [x, y, z]
                
            elif type_choice == "2":
                primitive_type = "sphere"
                print()
                r = float(get_user_input("Raggio [0.1]") or "0.1")
                dimensions = [r]
                
            elif type_choice == "3":
                primitive_type = "cylinder"
                print()
                h = float(get_user_input("Altezza [0.2]") or "0.2")
                r = float(get_user_input("Raggio [0.05]") or "0.05")
                dimensions = [h, r]
            else:
                print_error("Scelta non valida")
                wait_for_key()
                return
            
            # Ask for position
            print()
            print_info("Inserisci la posizione [x, y, z] in metri")
            px = float(get_user_input("Posizione X [0.5]") or "0.5")
            py = float(get_user_input("Posizione Y [0.0]") or "0.0")
            pz = float(get_user_input("Posizione Z [0.5]") or "0.5")
            position = [px, py, pz]
            
            # Add object
            if self.on_add_object and self.on_add_object(obj_id, primitive_type, dimensions, position):
                print_success(f"Oggetto '{obj_id}' aggiunto con successo!")
            else:
                print_error("Errore nell'aggiunta dell'oggetto")
                
        except ValueError:
            print_error("Valore numerico non valido")
        
        wait_for_key()
    
    def show_remove_object_menu(self):
        """Menu for removing an object."""
        while True:
            if self.on_refresh:
                self.on_refresh()
            
            clear_screen()
            print_header("➖ RIMUOVI OGGETTO")
            
            all_objects = list(self.state_manager.objects.keys()) + list(self.state_manager.disabled_objects.keys())
            
            if not all_objects:
                print_warning("Nessun oggetto da rimuovere")
                wait_for_key()
                return
            
            for i, obj_id in enumerate(all_objects, 1):
                disabled = " (disabilitato)" if obj_id in self.state_manager.disabled_objects else ""
                print_menu_item(i, f"{obj_id}{disabled}")
            
            print()
            print_menu_item(0, "↩️  Annulla")
            
            choice = get_user_input("Seleziona oggetto da rimuovere")
            
            try:
                idx = int(choice)
                if idx == 0:
                    return
                if 1 <= idx <= len(all_objects):
                    obj_id = all_objects[idx - 1]
                    confirm = get_user_input(f"Rimuovere '{obj_id}'? [s/N]")
                    if confirm.lower() in ['s', 'si', 'yes', 'y']:
                        if self.on_remove_object and self.on_remove_object(obj_id):
                            print_success(f"Oggetto '{obj_id}' rimosso!")
                        else:
                            print_error("Errore nella rimozione")
                        wait_for_key()
                        return
            except ValueError:
                print_error("Input non valido")
                wait_for_key()
    
    def show_enable_disable_menu(self):
        """Menu for enabling/disabling objects."""
        while True:
            if self.on_refresh:
                self.on_refresh()
            
            clear_screen()
            print_header("🔛 ABILITA/DISABILITA OGGETTI")
            
            active_objects = [o for o in self.state_manager.objects.values() if o.enabled]
            disabled_objects = list(self.state_manager.disabled_objects.keys())
            
            if not active_objects and not disabled_objects:
                print_warning("Nessun oggetto presente")
                wait_for_key()
                return
            
            print_subheader("Oggetti Attivi (seleziona per disabilitare)")
            for i, obj in enumerate(active_objects, 1):
                print_menu_item(i, f"🟢 {obj.id}")
            
            offset = len(active_objects)
            
            print_subheader("Oggetti Disabilitati (seleziona per abilitare)")
            for i, obj_id in enumerate(disabled_objects, offset + 1):
                print_menu_item(i, f"🔴 {obj_id}")
            
            print()
            print_menu_item(0, "↩️  Torna al menù principale")
            
            choice = get_user_input("Seleziona oggetto")
            
            try:
                idx = int(choice)
                if idx == 0:
                    return
                
                if 1 <= idx <= len(active_objects):
                    obj_id = active_objects[idx - 1].id
                    if self.on_disable_object and self.on_disable_object(obj_id):
                        print_success(f"Oggetto '{obj_id}' disabilitato!")
                    else:
                        print_error("Errore nella disabilitazione")
                    wait_for_key()
                    
                elif offset < idx <= offset + len(disabled_objects):
                    obj_id = disabled_objects[idx - offset - 1]
                    if self.on_enable_object and self.on_enable_object(obj_id):
                        print_success(f"Oggetto '{obj_id}' abilitato!")
                    else:
                        print_error("Errore nell'abilitazione")
                    wait_for_key()
                    
            except ValueError:
                print_error("Input non valido")
                wait_for_key()
    
    def show_move_object_menu(self):
        """Menu for moving an object."""
        while True:
            if self.on_refresh:
                self.on_refresh()
            
            clear_screen()
            print_header("🚀 MUOVI OGGETTO")
            
            active_objects = [o for o in self.state_manager.objects.values() if o.enabled]
            
            if not active_objects:
                print_warning("Nessun oggetto attivo da muovere")
                wait_for_key()
                return
            
            for i, obj in enumerate(active_objects, 1):
                print_menu_item(i, f"{obj.id} - pos: [{obj.position[0]:.2f}, {obj.position[1]:.2f}, {obj.position[2]:.2f}]")
            
            print()
            print_menu_item(0, "↩️  Annulla")
            
            choice = get_user_input("Seleziona oggetto")
            
            try:
                idx = int(choice)
                if idx == 0:
                    return
                
                if 1 <= idx <= len(active_objects):
                    obj = active_objects[idx - 1]
                    
                    print()
                    print_info(f"Posizione attuale: [{obj.position[0]:.3f}, {obj.position[1]:.3f}, {obj.position[2]:.3f}]")
                    print_info("Inserisci la nuova posizione:")
                    
                    px = float(get_user_input(f"X [{obj.position[0]:.2f}]") or str(obj.position[0]))
                    py = float(get_user_input(f"Y [{obj.position[1]:.2f}]") or str(obj.position[1]))
                    pz = float(get_user_input(f"Z [{obj.position[2]:.2f}]") or str(obj.position[2]))
                    
                    duration = float(get_user_input("Durata movimento (s) [1.0]") or "1.0")
                    
                    if self.on_move_object and self.on_move_object(obj.id, [px, py, pz], None, duration):
                        print_success(f"Comando di movimento inviato per '{obj.id}'!")
                    else:
                        print_error("Errore nell'invio del comando")
                    
                    wait_for_key()
                    return
                    
            except ValueError:
                print_error("Valore numerico non valido")
                wait_for_key()
    
    def show_loop_menu(self):
        """Menu for loop configuration."""
        while True:
            if self.on_refresh:
                self.on_refresh()
            
            clear_screen()
            print_header("🔁 CONFIGURAZIONE LOOP")
            
            objects_with_sequences = [o for o in self.state_manager.objects.values() 
                                     if o.enabled or o.loop_waypoints]
            
            if not objects_with_sequences:
                print_warning("Nessun oggetto attivo o con sequenze configurate")
                wait_for_key()
                return
            
            for i, obj in enumerate(objects_with_sequences, 1):
                loop_status = ""
                active_marker = ""
                if not obj.enabled:
                    active_marker = f" {Colors.DIM}(non in scena){Colors.END}"
                
                if self.on_is_loop_running and self.on_is_loop_running(obj.id):
                    loop_status = f" {Colors.GREEN}▶ IN ESECUZIONE{Colors.END}"
                elif obj.loop_waypoints:
                    loop_status = f" {Colors.YELLOW}[{len(obj.loop_waypoints)} waypoints]{Colors.END}"
                else:
                    loop_status = f" {Colors.DIM}[non configurato]{Colors.END}"
                
                print_menu_item(i, f"{obj.id}{loop_status}{active_marker}")
            
            print()
            print_menu_item(0, "↩️  Torna al menù principale")
            
            choice = get_user_input("Seleziona oggetto")
            
            try:
                idx = int(choice)
                if idx == 0:
                    return
                
                if 1 <= idx <= len(objects_with_sequences):
                    obj = objects_with_sequences[idx - 1]
                    self.show_object_loop_menu(obj.id)
                    
            except ValueError:
                print_error("Input non valido")
                wait_for_key()
    
    def show_object_loop_menu(self, obj_id: str):
        """Menu for configuring loop of a single object.
        
        Args:
            obj_id: Object ID to configure
        """
        while True:
            obj = self.state_manager.objects.get(obj_id)
            if not obj:
                return
            
            clear_screen()
            print_header(f"🔁 LOOP: {obj_id}")
            
            is_running = self.on_is_loop_running(obj_id) if self.on_is_loop_running else False
            status = f"{Colors.GREEN}IN ESECUZIONE{Colors.END}" if is_running else f"{Colors.RED}FERMO{Colors.END}"
            print(f"  Stato: {status}")
            print()
            
            # Show waypoints
            if obj.loop_waypoints:
                print_subheader(f"Waypoints ({len(obj.loop_waypoints)})")
                for i, wp in enumerate(obj.loop_waypoints):
                    pos = wp.get('position', [0, 0, 0])
                    dur = wp.get('duration', 1.0)
                    current = " ← corrente" if is_running and i == obj.loop_current_idx else ""
                    print(f"    {i+1}. pos=[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] dur={dur}s{Colors.GREEN}{current}{Colors.END}")
            else:
                print_info("Nessun waypoint configurato")
            
            print()
            
            if is_running:
                print_menu_item(1, "🛑 Ferma loop")
            else:
                print_menu_item(1, "▶️  Avvia loop")
            
            print_menu_item(2, "➕ Aggiungi waypoint")
            print_menu_item(3, "➖ Rimuovi waypoint")
            print_menu_item(4, "🗑️  Cancella tutti i waypoints")
            print_menu_item(5, "📍 Aggiungi posizione corrente come waypoint")
            print()
            print_menu_item(0, "↩️  Indietro")
            
            choice = get_user_input("Seleziona opzione")
            
            if choice == "0":
                return
            elif choice == "1":
                if is_running:
                    if self.on_stop_loop and self.on_stop_loop(obj_id):
                        print_success("Loop fermato!")
                    else:
                        print_error("Errore nel fermare il loop")
                else:
                    if self.on_start_loop and self.on_start_loop(obj_id):
                        print_success("Loop avviato!")
                    else:
                        print_error("Errore nell'avviare il loop")
                wait_for_key()
                
            elif choice == "2":
                self._show_add_waypoint_submenu(obj_id)
                
            elif choice == "3":
                self._show_remove_waypoint_submenu(obj_id)
                
            elif choice == "4":
                confirm = get_user_input("Cancellare tutti i waypoints? [s/N]")
                if confirm.lower() in ['s', 'si', 'yes', 'y']:
                    if self.on_clear_waypoints and self.on_clear_waypoints(obj_id):
                        print_success("Waypoints cancellati!")
                wait_for_key()
                
            elif choice == "5":
                pos = obj.position
                dur = float(get_user_input("Durata (s) [1.0]") or "1.0")
                if self.on_add_waypoint and self.on_add_waypoint(obj_id, pos, None, dur):
                    print_success(f"Waypoint aggiunto: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
                wait_for_key()
    
    def _show_add_waypoint_submenu(self, obj_id: str):
        """Submenu for adding a waypoint."""
        obj = self.state_manager.objects.get(obj_id)
        if not obj:
            return
        
        print()
        print_info("Inserisci la posizione del waypoint:")
        
        try:
            px = float(get_user_input(f"X [{obj.position[0]:.2f}]") or str(obj.position[0]))
            py = float(get_user_input(f"Y [{obj.position[1]:.2f}]") or str(obj.position[1]))
            pz = float(get_user_input(f"Z [{obj.position[2]:.2f}]") or str(obj.position[2]))
            dur = float(get_user_input("Durata (s) [1.0]") or "1.0")
            
            if self.on_add_waypoint and self.on_add_waypoint(obj_id, [px, py, pz], None, dur):
                print_success("Waypoint aggiunto!")
            
        except ValueError:
            print_error("Valore numerico non valido")
        
        wait_for_key()
    
    def _show_remove_waypoint_submenu(self, obj_id: str):
        """Submenu for removing a waypoint."""
        obj = self.state_manager.objects.get(obj_id)
        if not obj or not obj.loop_waypoints:
            print_warning("Nessun waypoint da rimuovere")
            wait_for_key()
            return
        
        print()
        idx_str = get_user_input(f"Indice waypoint da rimuovere (1-{len(obj.loop_waypoints)})")
        
        try:
            idx = int(idx_str) - 1
            if self.on_remove_waypoint and self.on_remove_waypoint(obj_id, idx):
                print_success("Waypoint rimosso!")
            else:
                print_error("Indice non valido")
        except ValueError:
            print_error("Input non valido")
        
        wait_for_key()
    
    def show_distances_menu(self):
        """Display distance information."""
        clear_screen()
        print_header("📊 DISTANZE ROBOT-OSTACOLI")
        
        distance_info = self.on_get_distance_info() if self.on_get_distance_info else None
        
        if distance_info is None:
            print_warning("Nessuna informazione sulle distanze disponibile")
            print_info("Assicurati che il distance_monitor_node sia in esecuzione")
            wait_for_key()
            return
        
        contacts = distance_info.contacts
        
        if not contacts:
            print_info("Nessun contatto rilevato")
        else:
            print(f"  {Colors.BOLD}Contatti rilevati: {len(contacts)}{Colors.END}")
            print()
            
            sorted_contacts = sorted(contacts, key=lambda c: c.distance)
            
            for i, contact in enumerate(sorted_contacts[:10]):
                if contact.distance < 0.05:
                    color = Colors.RED
                elif contact.distance < 0.15:
                    color = Colors.YELLOW
                else:
                    color = Colors.GREEN
                
                print(f"  {i+1}. {Colors.BOLD}{contact.link_name}{Colors.END} ↔ {Colors.CYAN}{contact.object_id}{Colors.END}")
                print(f"     Distanza: {color}{contact.distance:.4f} m{Colors.END}")
                print(f"     Punto robot: [{contact.robot_point.x:.3f}, {contact.robot_point.y:.3f}, {contact.robot_point.z:.3f}]")
                print(f"     Punto oggetto: [{contact.object_point.x:.3f}, {contact.object_point.y:.3f}, {contact.object_point.z:.3f}]")
                print()
            
            if len(contacts) > 10:
                print_info(f"... e altri {len(contacts) - 10} contatti")
        
        wait_for_key()

