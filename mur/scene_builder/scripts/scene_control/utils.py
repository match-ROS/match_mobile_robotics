#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Utility functions for terminal output and user input.
"""

import os


class Colors:
    """ANSI color codes for terminal output."""
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    MAGENTA = '\033[35m'
    ORANGE = '\033[38;5;208m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    DIM = '\033[2m'
    END = '\033[0m'


def clear_screen():
    """Clear the terminal screen."""
    os.system('clear' if os.name == 'posix' else 'cls')


def print_header(title: str):
    """Print a styled header."""
    width = 70
    print(f"\n{Colors.CYAN}{Colors.BOLD}{'═' * width}{Colors.END}")
    print(f"{Colors.CYAN}{Colors.BOLD}  {title.center(width - 4)}{Colors.END}")
    print(f"{Colors.CYAN}{Colors.BOLD}{'═' * width}{Colors.END}\n")


def print_subheader(title: str):
    """Print a styled subheader."""
    width = 60
    print(f"\n  {Colors.MAGENTA}{'─' * width}{Colors.END}")
    print(f"  {Colors.MAGENTA}{Colors.BOLD}{title}{Colors.END}")
    print(f"  {Colors.MAGENTA}{'─' * width}{Colors.END}")


def print_menu_item(number, text: str, highlight: bool = False):
    """Print a menu item with number."""
    color = Colors.GREEN if highlight else Colors.YELLOW
    print(f"  {color}[{number}]{Colors.END} {text}")


def print_info(text: str):
    """Print an informational message."""
    print(f"  {Colors.BLUE}ℹ {text}{Colors.END}")


def print_success(text: str):
    """Print a success message."""
    print(f"  {Colors.GREEN}✓ {text}{Colors.END}")


def print_error(text: str):
    """Print an error message."""
    print(f"  {Colors.RED}✗ {text}{Colors.END}")


def print_warning(text: str):
    """Print a warning message."""
    print(f"  {Colors.YELLOW}⚠ {text}{Colors.END}")


def get_user_input(prompt: str) -> str:
    """Get input from user with styled prompt."""
    return input(f"\n  {Colors.CYAN}→ {prompt}: {Colors.END}").strip()


def wait_for_key():
    """Wait for user to press a key."""
    input(f"\n  {Colors.CYAN}Premi INVIO per continuare...{Colors.END}")

