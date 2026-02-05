#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive Scene Control - Entry Point
========================================
Interactive script for managing collision objects in MoveIt scene.

Features:
- View current collision objects status
- Add new objects (box, sphere, cylinder)
- Remove existing objects
- Enable/disable objects
- Move objects from one point to another
- Configure and manage loop movement for objects

This script is the entry point that uses the modular scene_control package.

Author: Generated for scene_builder package
"""

from scene_control import SceneInteractiveController


def main():
    """Entry point of the script."""
    controller = SceneInteractiveController()
    controller.run()


if __name__ == '__main__':
    main()
