import sys
import os
from PyQt6 import uic
from PyQt6.QtWidgets import QApplication, QMainWindow, QFileDialog


class TLXWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        ui_path = os.path.join(os.path.dirname(__file__), "P5_TLX_RATING.ui")
        uic.loadUi(ui_path, self)
        self._collect_widgets()
        self._configure_sliders()
        self._connect_signals()

    def _collect_widgets(self):
        self._slider_names = [
            "horizontalSlider",
            "horizontalSlider_2",
            "horizontalSlider_3",
            "horizontalSlider_4",
            "horizontalSlider_5",
            "horizontalSlider_6",
        ]
        self._lcd_names = [
            "lcdNumber",
            "lcdNumber_2",
            "lcdNumber_3",
            "lcdNumber_4",
            "lcdNumber_5",
            "lcdNumber_6",
        ]
        self.sliders = [getattr(self, n) for n in self._slider_names]
        self.lcds = [getattr(self, n) for n in self._lcd_names]
        # Export button (objectName pushButton in .ui)
        self.export_button = getattr(self, "pushButton", None)

    def _configure_sliders(self):
        for s in self.sliders:
            s.setRange(0, 100)
            if s.value() < 0 or s.value() > 100:
                s.setValue(50)

    def _connect_signals(self):
        for slider, lcd in zip(self.sliders, self.lcds):
            def handler(val, s=slider, l=lcd):
                # Snap to nearest multiple of 5
                snapped = round(val / 5) * 5
                if snapped != val:
                    # Prevent recursive signal emission
                    s.blockSignals(True)
                    s.setValue(snapped)
                    s.blockSignals(False)
                    val = snapped
                l.display(val)
            slider.valueChanged.connect(handler)
            handler(slider.value())
        if self.export_button:
            self.export_button.clicked.connect(self.export_results)

    def get_results(self):
        names = [
            "mental_demand",
            "physical_demand",
            "temporal_demand",
            "performance",
            "effort",
            "frustration",
        ]
        return {n: s.value() for n, s in zip(names, self.sliders)}

    def export_results(self):
        data = self.get_results()
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save TLX Ratings",
            "tlx_ratings.txt",
            "Text Files (*.txt);;All Files (*)",
        )
        if not filename:
            return
        order = [
            "mental_demand",
            "physical_demand",
            "temporal_demand",
            "performance",
            "effort",
            "frustration",
        ]
        lines = [f"{key.replace('_', ' ').title()}: {data[key]}" for key in order]
        content = "\n".join(lines) + "\n"
        with open(filename, "w", encoding="utf-8") as f:
            f.write(content)


def main():
    app = QApplication(sys.argv)
    win = TLXWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
