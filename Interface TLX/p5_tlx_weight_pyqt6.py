import sys
import os
from typing import Dict, List, Tuple
from PyQt6 import uic
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QFileDialog,
    QMessageBox,
)
from PyQt6.QtCore import QObject

DIMENSION_NAMES = [
    "Mental Demand",
    "Physical Demand",
    "Temporal Demand",
    "Performance",
    "Effort",
    "Frustration",
]


def parse_button_text(text: str) -> Tuple[str, str]:
    """Parse a button label of the form 'A\nor\nB' -> (A, B).
    Falls back to returning (text, "") if pattern not found.
    """
    parts = [p.strip() for p in text.split("\n") if p.strip()]
    if "or" in parts:
        # Remove lone 'or' tokens
        parts = [p for p in parts if p.lower() != "or"]
    # Heuristic: after removing 'or', we expect exactly 2 parts
    if len(parts) == 2:
        return parts[0], parts[1]
    # Sometimes text may be 'A\nor\nB' with additional blank lines; attempt second pass
    lowered = text.lower()
    if " or " in lowered:
        before, after = text.split(" or ", 1)
        return before.strip(), after.strip()
    return text.strip(), ""


class TLXWeightWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        ui_path = os.path.join(os.path.dirname(__file__), "P5_TLX_WEIGHt.ui")
        if not os.path.exists(ui_path):
            QMessageBox.critical(self, "Error", f"UI file not found: {ui_path}")
            raise FileNotFoundError(ui_path)
        uic.loadUi(ui_path, self)
        self.weights: Dict[str, int] = {name: 0 for name in DIMENSION_NAMES}
        self._current_index = 0
        self._pages: List[QObject] = []
        self._page_buttons: List[List[QPushButton]] = []
        self._collect_pages()
        self._wire_buttons()
        self._update_window_title()

    def _collect_pages(self):
        self.stacked = getattr(self, "stackedWidget", None)
        if self.stacked is None:
            QMessageBox.critical(self, "Error", "stackedWidget not found in UI")
            raise RuntimeError("stackedWidget missing")
        count = self.stacked.count()
        for i in range(count):
            page = self.stacked.widget(i)
            self._pages.append(page)
            buttons = page.findChildren(QPushButton)
            # Filter out navigation/export style buttons if any (heuristic: text contains all 6 dims or is empty)
            filtered = [b for b in buttons if b.text().strip()]
            self._page_buttons.append(filtered)

    def _wire_buttons(self):
        for page_idx, buttons in enumerate(self._page_buttons):
            # Expect 2 buttons per pairwise comparison page.
            if len(buttons) < 2:
                continue
            for btn in buttons:
                btn.clicked.connect(lambda checked=False, b=btn, i=page_idx: self._handle_choice(b, i))

    def _handle_choice(self, button: QPushButton, page_index: int):
        # Parse the pair from the button label ("A\nor\nB")
        a, b = parse_button_text(button.text())
        # If we failed to get a proper pair, just advance (nothing counted)
        if not b or a == b:
            QMessageBox.warning(self, "Pair Parse", f"Could not parse a pair from: '{button.text()}'")
            self._advance()
            return

        # Ask user which dimension contributed more workload.
        box = QMessageBox(self)
        box.setWindowTitle("Select Dimension")
        box.setText(f"Which contributed more to workload?\n\n{a} OR {b}")
        btn_a = box.addButton(a, QMessageBox.ButtonRole.AcceptRole)
        btn_b = box.addButton(b, QMessageBox.ButtonRole.DestructiveRole)
        box.exec()
        clicked = box.clickedButton()
        chosen = a if clicked == btn_a else b
        # Sanity check / fallback
        if chosen not in self.weights:
            QMessageBox.warning(self, "Unrecognized", f"Chosen dimension '{chosen}' not in known list; ignoring.")
            self._advance()
            return
        self.weights[chosen] += 1
        self._advance()

    def _advance(self):
        self._current_index += 1
        if self._current_index >= len(self._pages):
            self._finish()
        else:
            self.stacked.setCurrentIndex(self._current_index)
            self._update_window_title()

    def _update_window_title(self):
        remaining = len(self._pages) - self._current_index
        self.setWindowTitle(f"NASA TLX Weighting - Remaining pairs: {remaining}")

    def _finish(self):
        # Show summary and offer export.
        summary_lines = [f"{k}: {v}" for k, v in self.weights.items()]
        msg = "Weighting complete:\n" + "\n" + "\n".join(summary_lines) + "\n\nExport results?"
        reply = QMessageBox.question(self, "Complete", msg)
        if reply == QMessageBox.StandardButton.Yes:
            self._export()

    def _export(self):
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save TLX Weights",
            "tlx_weights.txt",
            "Text Files (*.txt);;All Files (*)",
        )
        if not filename:
            return
        total_pairs = sum(self.weights.values()) or 1
        # Compute normalized weights (percentage)
        lines = []
        for dim in DIMENSION_NAMES:
            raw = self.weights[dim]
            pct = raw / total_pairs * 100.0
            lines.append(f"{dim}: {raw} ({pct:.1f}%)")
        content = "\n".join(lines) + "\n"
        with open(filename, "w", encoding="utf-8") as f:
            f.write(content)
        QMessageBox.information(self, "Saved", f"Weights exported to:\n{filename}")


def main():
    app = QApplication(sys.argv)
    win = TLXWeightWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
