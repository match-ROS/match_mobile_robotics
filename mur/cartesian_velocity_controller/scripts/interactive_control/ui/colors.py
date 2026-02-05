"""
ANSI color codes for terminal output.
"""


class Colors:
    """ANSI escape codes for terminal colors and styling."""
    
    # Text colors
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    MAGENTA = '\033[35m'
    WHITE = '\033[97m'
    
    # Text styles
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    DIM = '\033[2m'
    
    # Reset
    END = '\033[0m'
    
    @classmethod
    def colorize(cls, text: str, color: str) -> str:
        """
        Wrap text with color codes.
        
        Args:
            text: Text to colorize
            color: Color code (use class attributes)
            
        Returns:
            Colorized string
        """
        return f"{color}{text}{cls.END}"
    
    @classmethod
    def bold(cls, text: str) -> str:
        """Make text bold."""
        return f"{cls.BOLD}{text}{cls.END}"
    
    @classmethod
    def success(cls, text: str) -> str:
        """Format as success (green)."""
        return f"{cls.GREEN}{text}{cls.END}"
    
    @classmethod
    def error(cls, text: str) -> str:
        """Format as error (red)."""
        return f"{cls.RED}{text}{cls.END}"
    
    @classmethod
    def warning(cls, text: str) -> str:
        """Format as warning (yellow)."""
        return f"{cls.YELLOW}{text}{cls.END}"
    
    @classmethod
    def info(cls, text: str) -> str:
        """Format as info (blue)."""
        return f"{cls.BLUE}{text}{cls.END}"

