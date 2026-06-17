#!/usr/bin/env python3
"""Cooperative handling GUI built on the shared MuR base GUI."""

import signal
import sys

from PyQt5 import QtWidgets

from match_cooperative_handling.cooperative_gui_module import CooperativeHandlingModule
from match_mur_gui.base_gui import MurBaseGui


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    window = MurBaseGui(
        modules=[CooperativeHandlingModule()],
        window_title="MuR Cooperative Handling",
    )
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
