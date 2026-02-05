#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Scene Control Module
====================
Modular Python package for interactive scene management in MoveIt.

This package provides:
- utils: Terminal colors, printing utilities, input helpers
- ros_interface: ROS publishers, subscribers, and service clients
- object_manager: Object state management and data classes
- menu_ui: Interactive menu handlers
- controller: Main controller orchestrating all components
"""

from .utils import (
    Colors,
    clear_screen,
    print_header,
    print_subheader,
    print_menu_item,
    print_info,
    print_success,
    print_error,
    print_warning,
    get_user_input,
    wait_for_key,
)

from .object_manager import (
    ObjectInfo,
    LoopWaypoint,
    PRIMITIVE_TYPES,
    DEFAULT_DIMENSIONS,
)

from .ros_interface import ROSInterface
from .menu_ui import MenuUI
from .controller import SceneInteractiveController

__all__ = [
    # Utils
    'Colors',
    'clear_screen',
    'print_header',
    'print_subheader',
    'print_menu_item',
    'print_info',
    'print_success',
    'print_error',
    'print_warning',
    'get_user_input',
    'wait_for_key',
    # Object Manager
    'ObjectInfo',
    'LoopWaypoint',
    'PRIMITIVE_TYPES',
    'DEFAULT_DIMENSIONS',
    # ROS Interface
    'ROSInterface',
    # Menu UI
    'MenuUI',
    # Controller
    'SceneInteractiveController',
]

__version__ = '1.0.0'

