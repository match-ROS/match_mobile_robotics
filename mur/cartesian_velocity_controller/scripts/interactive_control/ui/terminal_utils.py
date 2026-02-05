"""
Terminal utility functions for interactive menu display.
"""

import os
from .colors import Colors


def clear_screen():
    """Clear the terminal screen."""
    os.system('clear' if os.name == 'posix' else 'cls')


def print_header(title: str, width: int = 60):
    """
    Print a styled header.
    
    Args:
        title: Header title text
        width: Width of the header box
    """
    print(f"\n{Colors.CYAN}{Colors.BOLD}{'═' * width}{Colors.END}")
    print(f"{Colors.CYAN}{Colors.BOLD}  {title.center(width - 4)}{Colors.END}")
    print(f"{Colors.CYAN}{Colors.BOLD}{'═' * width}{Colors.END}\n")


def print_menu_item(number: int, text: str, highlight: bool = False):
    """
    Print a menu item.
    
    Args:
        number: Menu item number
        text: Menu item description
        highlight: Whether to highlight this item
    """
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
    """
    Get input from user with styled prompt.
    
    Args:
        prompt: Prompt text to display
        
    Returns:
        User input string, stripped of whitespace
    """
    return input(f"\n  {Colors.CYAN}→ {prompt}: {Colors.END}").strip()


def wait_for_key():
    """Wait for user to press Enter."""
    input(f"\n  {Colors.CYAN}Premi INVIO per continuare...{Colors.END}")


def print_pose_info(name: str, position: list, orientation: list = None, 
                    description: str = "", indent: int = 4):
    """
    Print formatted pose information.
    
    Args:
        name: Pose name
        position: [x, y, z] position
        orientation: [qx, qy, qz, qw] quaternion (optional)
        description: Pose description
        indent: Number of spaces to indent
    """
    spaces = " " * indent
    print(f"{spaces}{Colors.BOLD}{name}{Colors.END}")
    if description:
        print(f"{spaces}  {Colors.DIM}{description}{Colors.END}")
    print(f"{spaces}  Position: [{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}]")
    if orientation:
        print(f"{spaces}  Orientation: [{orientation[0]:.4f}, {orientation[1]:.4f}, "
              f"{orientation[2]:.4f}, {orientation[3]:.4f}]")


def print_status_line(label: str, value: str, status_ok: bool = True):
    """
    Print a status line with colored value.
    
    Args:
        label: Status label
        value: Status value
        status_ok: Whether status is OK (green) or not (red)
    """
    color = Colors.GREEN if status_ok else Colors.RED
    print(f"  {label}: {color}{value}{Colors.END}")

