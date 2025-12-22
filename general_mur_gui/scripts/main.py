#! /usr/bin/env python3
import signal
import sys
from PyQt5.QtWidgets import QApplication
from gui_layout import ROSGui

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ROSGui()
    window.show()

    def _handle_sigint(_sig, _frame):
        """Gracefully close the GUI when Ctrl+C is pressed in the terminal."""
        window.close()

    signal.signal(signal.SIGINT, _handle_sigint)

    sys.exit(app.exec_())
