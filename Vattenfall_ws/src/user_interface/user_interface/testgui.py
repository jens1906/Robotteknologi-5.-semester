#!/usr/bin/env python3
"""
Test GUI for image click detection.
Auto-generated style UI class for the test application.
"""

import sys
import cv2
from pathlib import Path
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QStatusBar
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import Qt
from PyQt6 import QtCore
from image_click_detector import ClickableImageLabel


class Ui_TestWindow(object):
    """UI class for test image viewer"""
    
    def setupUi(self, MainWindow):
        """Setup the UI"""
        MainWindow.setObjectName("TestWindow")
        MainWindow.resize(1200, 800)
        
        # Central widget
        self.centralwidget = QWidget(parent=MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        # Main layout
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        
        # Image label
        self.image_label = ClickableImageLabel()
        self.image_label.setMinimumSize(400, 400)
        self.image_label.setStyleSheet("border: 2px solid gray; background-color: #f0f0f0;")
        self.image_label.setObjectName("image_label")
        self.verticalLayout.addWidget(self.image_label)
        
        # Set central widget
        MainWindow.setCentralWidget(self.centralwidget)
        
        # Status bar
        self.statusbar = QStatusBar(parent=MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
    
    def retranslateUi(self, MainWindow):
        """Translate UI strings"""
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("TestWindow", "Image Click Detector - Test GUI"))
        self.image_label.setText(_translate("TestWindow", "image_label"))


class TestImageViewer(QMainWindow):
    """Main application window for testing image click detection"""
    
    def __init__(self, image_path=None):
        super().__init__()
        self.ui = Ui_TestWindow()
        self.ui.setupUi(self)
        
        # Connect signals
        self.ui.image_label.clicked.connect(self.on_image_clicked)
        self.ui.image_label.dragged.connect(self.on_image_dragged)
        
        # Load initial image if provided
        if image_path:
            self.load_image(image_path)
    
    def load_image(self, file_path):
        """Load image from given file path"""
        try:
            # Use OpenCV to load image
            image = cv2.imread(str(file_path))
            
            if image is None:
                self.ui.statusbar.showMessage(f"Failed to load image: {file_path}")
                return
            
            # Convert BGR to RGB for display
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            h, w = image_rgb.shape[:2]
            
            # Convert to QImage and display
            bytes_per_line = 3 * w
            qimage = QImage(image_rgb.data.tobytes(), w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
            pixmap = QPixmap.fromImage(qimage)
            
            # Scale to fit in label while maintaining aspect ratio
            scaled_pixmap = pixmap.scaled(
                self.ui.image_label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            
            self.ui.image_label.setPixmap(scaled_pixmap)
            self.ui.image_label.set_image_dimensions(w, h)
            
            # Update status bar
            self.ui.statusbar.showMessage(f"Image loaded: {Path(file_path).name} ({w}x{h})")
            
            print(f"Image loaded: {file_path} ({w}x{h})")
            
        except Exception as e:
            self.ui.statusbar.showMessage(f"Error: {str(e)}")
            print(f"Error loading image: {e}")
    
    def on_image_clicked(self, x, y, button):
        """Handle image click"""
        print(f"[CLICK] {button.upper():6} | Image Coordinates: ({x:4d}, {y:4d})")
        self.ui.statusbar.showMessage(f"Clicked at ({x}, {y}) - {button} button")
    
    def on_image_dragged(self, x, y, button):
        """Handle image drag"""
        print(f"[DRAG]  {button.upper():6} | Image Coordinates: ({x:4d}, {y:4d})")
        self.ui.statusbar.showMessage(f"Dragging at ({x}, {y}) - {button} button")


def main():
    app = QApplication(sys.argv)
    
    # Hardcoded image path
    image_path = "src/realsense_publisher/image/realsense_capture_20251027_142056_color.png"
    
    window = TestImageViewer(image_path)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
