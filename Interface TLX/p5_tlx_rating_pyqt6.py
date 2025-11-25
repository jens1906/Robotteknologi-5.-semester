from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QSlider,
    QLCDNumber,
)
import sys


class TLXWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NASA TLX Rating")
        self.resize(610, 417)

        central = QWidget(self)
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)

        # Specification for each TLX dimension: (title, left_label, right_label)
        dimensions = [
            ("Mental Demand", "Low", "High"),
            ("Physical Demand", "Low", "High"),
            ("Temporal Demand", "Low", "High"),
            ("Performance", "Good", "Poor"),
            ("Effort", "Low", "High"),
            ("Frustration", "Low", "High"),
        ]

        self.sliders = []
        self.lcds = []

        for title, left, right in dimensions:
            title_label = QLabel(title)
            main_layout.addWidget(title_label)

            row = QHBoxLayout()
            left_label = QLabel(left)
            left_label.setMinimumWidth(30)
            row.addWidget(left_label)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, 100)
            slider.setValue(50)
            row.addWidget(slider)

            right_label = QLabel(right)
            row.addWidget(right_label)

            lcd = QLCDNumber()
            lcd.display(slider.value())
            row.addWidget(lcd)

            # Connect slider updates
            slider.valueChanged.connect(lcd.display)

            self.sliders.append(slider)
            self.lcds.append(lcd)

            main_layout.addLayout(row)

    def get_results(self):
        """Return a dict mapping dimension names to numeric values."""
        names = [
            "mental_demand",
            "physical_demand",
            "temporal_demand",
            "performance",
            "effort",
            "frustration",
        ]
        return {n: s.value() for n, s in zip(names, self.sliders)}


def main():
    app = QApplication(sys.argv)
    win = TLXWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
