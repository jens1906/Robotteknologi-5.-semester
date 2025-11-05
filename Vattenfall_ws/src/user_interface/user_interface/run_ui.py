import os
import sys

# Ensure this directory is on sys.path so 'joystick' (custom widget) can be imported
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

try:
    # Import PyQt6
    from PyQt6 import QtWidgets, uic
    # Import custom widget so uic can resolve <widget class="Joystick"> from the .ui file
    from joystick import Joystick  # noqa: F401 (imported for side effect/availability)
except Exception as e:
    print("Failed to import PyQt6 or custom widget 'joystick':", e)
    print("Tip: Install PyQt6 ->")
    print("  python -m pip install PyQt6")
    sys.exit(1)


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = QtWidgets.QMainWindow()
    ui_path = os.path.join(THIS_DIR, "P5_GUI.ui")
    if not os.path.exists(ui_path):
        print(f"UI file not found: {ui_path}")
        sys.exit(1)
    uic.loadUi(ui_path, win)

    # Ensure tabs stretch across available width when running this loader
    try:
        from PyQt6.QtCore import Qt  # for elide mode enum
        tab = win.findChild(QtWidgets.QTabWidget, "tabWidget")
        if tab is not None:
            # Allow the tab widget to grow horizontally
            tab.setSizePolicy(
                QtWidgets.QSizePolicy.Policy.Expanding,
                QtWidgets.QSizePolicy.Policy.Preferred,
            )
            # Make individual tabs expand to fill the tab bar width
            tab.tabBar().setExpanding(True)
            # Optional: elide long labels nicely instead of overflowing
            tab.tabBar().setElideMode(Qt.TextElideMode.ElideRight)
    except Exception as e:
        print("Warning: could not apply tab stretching:", e)

    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
