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
        self._home_buttons: List[QPushButton] = []
        self._home_to_page: Dict[QPushButton, int] = {}
        self._completed_pairs: Dict[int, bool] = {}
        self._pair_selection: Dict[int, str] = {}
        self._collect_pages()
        self._wire_home_buttons()
        self._wire_pair_pages()
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

        # Assume page 0 is the home page containing 15 pair selector buttons
        if len(self._page_buttons) == 0:
            QMessageBox.critical(self, "Error", "No pages found in stackedWidget")
            raise RuntimeError("no pages")
        self._home_buttons = self._page_buttons[0]
        # Try to detect an explicit Export button on the home page
        self._export_button = None
        for b in self._home_buttons:
            if "export" in b.text().lower():
                self._export_button = b
                break
        # Map home buttons to subsequent pages (1..N-1). If counts mismatch, map as far as possible.
        for idx, btn in enumerate(self._home_buttons, start=1):
            if idx < count:
                self._home_to_page[btn] = idx
                self._completed_pairs[idx] = False

    def _wire_home_buttons(self):
        # Clicking a home button should navigate to its pair page
        for btn, page_idx in self._home_to_page.items():
            btn.clicked.connect(lambda checked=False, b=btn, i=page_idx: self._open_pair_page(b, i))
        if self._export_button is not None:
            self._export_button.clicked.connect(self._export)

    def _wire_pair_pages(self):
        # On pair pages, expect exactly two buttons representing the two dimensions
        for page_idx in range(1, len(self._page_buttons)):
            buttons = self._page_buttons[page_idx]
            if len(buttons) < 2:
                continue
            a_btn, b_btn = buttons[0], buttons[1]
            a_btn.clicked.connect(lambda checked=False, b=a_btn, i=page_idx: self._choose_on_pair_page(b, i))
            b_btn.clicked.connect(lambda checked=False, b=b_btn, i=page_idx: self._choose_on_pair_page(b, i))

    def _open_pair_page(self, home_button: QPushButton, page_index: int):
        self.stacked.setCurrentIndex(page_index)
        self._update_window_title()

    def _choose_on_pair_page(self, button: QPushButton, page_index: int):
        # Determine chosen dimension from the clicked button's text
        a, b = parse_button_text(button.text())
        # If parsing fails, treat the entire text as one dimension name
        chosen = a if a else button.text().strip()
        if chosen not in self.weights:
            # try second part
            if b in self.weights:
                chosen = b
            else:
                # fuzzy containment
                lowered = chosen.lower()
                matched = None
                for dim in self.weights.keys():
                    if dim.lower() in lowered:
                        matched = dim
                        break
                if matched:
                    chosen = matched
                else:
                    QMessageBox.warning(self, "Unrecognized", f"Could not map '{button.text()}' to a known dimension; ignoring.")
                    # Return to home
                    self.stacked.setCurrentIndex(0)
                    self._update_window_title()
                    return

        # If this pair had a previous selection, remove its count first
        prev = self._pair_selection.get(page_index)
        if prev and prev in self.weights:
            self.weights[prev] = max(0, self.weights[prev] - 1)
        # Apply new selection
        self.weights[chosen] += 1
        self._pair_selection[page_index] = chosen
        self._completed_pairs[page_index] = True
        # Find corresponding home button and mark it green
        home_btn = None
        for hb, idx in self._home_to_page.items():
            if idx == page_index:
                home_btn = hb
                break
        if home_btn is not None:
            home_btn.setStyleSheet("background-color: #5cb85c; color: white;")
        # Navigate back to home
        self.stacked.setCurrentIndex(0)
        self._update_window_title()

    def _advance(self):
        self._current_index += 1
        if self._current_index >= len(self._pages):
            self._finish()
        else:
            self.stacked.setCurrentIndex(self._current_index)
            self._update_window_title()

    def _update_window_title(self):
        # Set a static, clean window title (no remaining counter)
        self.setWindowTitle("NASA TLX Weighting")

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
        # Export raw counts only, one per line as "Name: count"
        lines = []
        for dim in DIMENSION_NAMES:
            raw = self.weights.get(dim, 0)
            lines.append(f"{dim}: {raw}")
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
