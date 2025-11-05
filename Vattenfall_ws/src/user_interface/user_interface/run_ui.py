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
    def apply_tab_stretch():
        try:
            from PyQt6.QtCore import Qt  # for elide mode enum
            tab = win.findChild(QtWidgets.QTabWidget, "tabWidget")
            if tab is not None:
                # Allow the tab widget to grow horizontally
                tab.setSizePolicy(
                    QtWidgets.QSizePolicy.Policy.Expanding,
                    QtWidgets.QSizePolicy.Policy.Preferred,
                )
                # Expanding tabs to fill available tab bar width
                tb = tab.tabBar()
                try:
                    tb.setExpanding(True)
                    tb.setElideMode(Qt.TextElideMode.ElideRight)
                    # Allow shrinking so expansion can distribute width evenly
                    tb.setStyleSheet(
                        "QTabBar { min-height: 28px; } QTabBar::tab { min-width: 0px; padding: 6px 10px; }"
                    )
                except Exception:
                    pass

            # Favor the tab widget over the emergency button in the parent HBox layout
            hbox = win.findChild(QtWidgets.QHBoxLayout, "horizontalLayout_3")
            if hbox is not None:
                # Layout order is [Emergency_Stop, tabWidget, stackedWidget]; favor tabWidget
                try:
                    hbox.setStretch(0, 0)
                    hbox.setStretch(1, 2)
                    hbox.setStretch(2, 1)
                except Exception as e:
                    print("Warning: could not set layout stretch:", e)

            # Prevent the Emergency button from grabbing extra horizontal space
            btn = win.findChild(QtWidgets.QPushButton, "Emergency_Stop")
            if btn is not None:
                sp = btn.sizePolicy()
                sp.setHorizontalPolicy(QtWidgets.QSizePolicy.Policy.Preferred)
                btn.setSizePolicy(sp)
        except Exception as e:
            print("Warning: could not apply tab stretching:", e)

    apply_tab_stretch()
    win.show()
    # Some layout calculations finalize after show; apply once more on the next cycle
    try:
        from PyQt6.QtCore import QTimer
        QTimer.singleShot(0, apply_tab_stretch)
    except Exception:
        pass
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
