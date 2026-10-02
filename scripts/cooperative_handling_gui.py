#!/usr/bin/env python3
"""Cooperative handling GUI built on the shared MuR base GUI."""

import signal
import sys

from PyQt5 import QtWidgets

from match_cooperative_handling.cooperative_gui_module import CooperativeHandlingModule
from match_mur_gui.base_gui import MurBaseGui
from match_mur_gui.app_icon import configure_gui_icon


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    icon = configure_gui_icon(app, "mur-cooperative-gui")
    window = MurBaseGui(
        modules=[CooperativeHandlingModule()],
        window_title="MuR Cooperative Handling",
    )
    window.resize(1280, 1000)
    window.setWindowIcon(icon)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
