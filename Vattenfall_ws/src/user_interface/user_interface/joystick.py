from PyQt6.QtWidgets import QWidget, QApplication, QMainWindow, QGridLayout, QStyleFactory
from PyQt6.QtGui import QPainter
from PyQt6.QtCore import Qt, QPointF, QRectF, QLineF, pyqtSignal
from enum import Enum
import sys

class Direction(Enum):
    Left = 0
    Right = 1
    Up = 2
    Down = 3

class Joystick(QWidget):
    # Signals
    touched = pyqtSignal()  # Emitted when joystick is touched
    released = pyqtSignal()  # Emitted when joystick is released
    moved = pyqtSignal(tuple)  # Emitted with (Direction, distance) when moved
    
    def __init__(self, parent=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(100, 100)
        self.movingOffset = self._center()
        self.grabCenter = False
        self.__maxDistance = 50

    def paintEvent(self, event):
        painter = QPainter(self)
        # Draw outer circle (background)
        bounds = QRectF(-self.__maxDistance, -self.__maxDistance,
                        self.__maxDistance * 2, self.__maxDistance * 2).translated(self._center())
        painter.drawEllipse(bounds)
        
        # Draw center knob (always centered unless grabbed)
        painter.setBrush(Qt.GlobalColor.black)
        painter.drawEllipse(self._centerEllipse())

    def _centerEllipse(self):
        if self.grabCenter:
            return QRectF(-20, -20, 40, 40).translated(self.movingOffset)
        return QRectF(-20, -20, 40, 40).translated(self._center())

    def _center(self):
        w = self.width() if self.width() > 0 else 100
        h = self.height() if self.height() > 0 else 100
        return QPointF(w / 2, h / 2)

    def _boundJoystick(self, point):
        limitLine = QLineF(self._center(), point)
        if limitLine.length() > self.__maxDistance:
            limitLine.setLength(self.__maxDistance)
        return limitLine.p2()

    def joystickDirection(self):
        if not self.grabCenter:
            return 0
        normVector = QLineF(self._center(), self.movingOffset)
        currentDistance = normVector.length()
        angle = normVector.angle()

        distance = min(currentDistance / self.__maxDistance, 1.0)
        if 45 <= angle < 135:
            return (Direction.Up, distance)
        elif 135 <= angle < 225:
            return (Direction.Left, distance)
        elif 225 <= angle < 315:
            return (Direction.Down, distance)
        return (Direction.Right, distance)

    def mousePressEvent(self, ev):
        # In PyQt6 use position() to get QPointF
        self.grabCenter = self._centerEllipse().contains(ev.position())
        if self.grabCenter:
            print("Joystick touched!")
            self.touched.emit()  # Emit signal
        return super().mousePressEvent(ev)

    def mouseReleaseEvent(self, event):
        self.grabCenter = False
        self.movingOffset = self._center()  # Reset to center
        self.released.emit()  # Emit signal
        self.update()  # Redraw immediately

    def mouseMoveEvent(self, event):
        if self.grabCenter:
            # Use position() for a QPointF to match QRectF/QLineF expectations
            self.movingOffset = self._boundJoystick(event.position())
            self.moved.emit(self.joystickDirection())  # Emit signal with direction
            self.update()
        print(self.joystickDirection())


if __name__ == "__main__":
    app = QApplication(sys.argv)
    mw = QMainWindow()
    mw.setWindowTitle("Joystick example")
    cw = QWidget()
    layout = QGridLayout(cw)
    mw.setCentralWidget(cw)
    joystick = Joystick()
    layout.addWidget(joystick, 0, 0)
    mw.show()
    sys.exit(app.exec())
