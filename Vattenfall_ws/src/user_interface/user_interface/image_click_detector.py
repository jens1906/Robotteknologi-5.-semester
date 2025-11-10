#!/usr/bin/env python3
"""
Pure Python PyQt6 application for detecting mouse clicks and drags on images.
Usage: python3 image_click_detector.py <image_path>
Or run without arguments to open a file dialog.
"""

import sys
import cv2
import numpy as np
from pathlib import Path
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QFileDialog, QPushButton, QHBoxLayout, QStatusBar
from PyQt6.QtGui import QImage, QPixmap, QFont
from PyQt6.QtCore import pyqtSignal, Qt


class ClickableImageLabel(QLabel):
    """QLabel that detects mouse clicks and drags with precise image coordinates"""
    clicked = pyqtSignal(int, int, str)  # x, y, button type
    dragged = pyqtSignal(int, int, str)  # x, y while dragging
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.image_width = 0
        self.image_height = 0
        self.is_dragging = False
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
    
    def set_image_dimensions(self, width, height):
        """Store actual image dimensions for coordinate mapping"""
        self.image_width = width
        self.image_height = height
    
    def _map_coordinates(self, label_x, label_y):
        """Convert label screen coordinates to actual image coordinates"""
        if self.pixmap() is None or self.pixmap().isNull():
            return None, None
        
        label_width = self.width()
        label_height = self.height()
        pixmap = self.pixmap()
        pixmap_width = pixmap.width()
        pixmap_height = pixmap.height()
        
        # Calculate pixmap position (centered in label due to AlignCenter)
        offset_x = (label_width - pixmap_width) // 2
        offset_y = (label_height - pixmap_height) // 2
        
        # Convert click position to pixmap coordinates
        pixmap_x = label_x - offset_x
        pixmap_y = label_y - offset_y
        
        # Check if within pixmap bounds
        if pixmap_x < 0 or pixmap_y < 0 or pixmap_x >= pixmap_width or pixmap_y >= pixmap_height:
            return None, None
        
        # Map to actual image coordinates using scale ratio
        image_x = int(pixmap_x * self.image_width / pixmap_width)
        image_y = int(pixmap_y * self.image_height / pixmap_height)
        
        # Clamp to image bounds
        image_x = max(0, min(image_x, self.image_width - 1))
        image_y = max(0, min(image_y, self.image_height - 1))
        
        return image_x, image_y
    
    def mousePressEvent(self, event):
        """Handle mouse button press"""
        self.is_dragging = True
        label_x = int(event.position().x())
        label_y = int(event.position().y())
        image_x, image_y = self._map_coordinates(label_x, label_y)
        
        if image_x is not None and image_y is not None:
            button_name = "left"
            if event.button() == Qt.MouseButton.RightButton:
                button_name = "right"
            elif event.button() == Qt.MouseButton.MiddleButton:
                button_name = "middle"
            
            self.clicked.emit(image_x, image_y, button_name)
    
    def mouseMoveEvent(self, event):
        """Handle mouse movement while dragging"""
        if self.is_dragging:
            label_x = int(event.position().x())
            label_y = int(event.position().y())
            image_x, image_y = self._map_coordinates(label_x, label_y)
            
            if image_x is not None and image_y is not None:
                # Determine which button is being held
                if event.buttons() == Qt.MouseButton.LeftButton:
                    button_name = "left"
                elif event.buttons() == Qt.MouseButton.RightButton:
                    button_name = "right"
                elif event.buttons() == Qt.MouseButton.MiddleButton:
                    button_name = "middle"
                else:
                    button_name = "unknown"
                
                self.dragged.emit(image_x, image_y, button_name)
    
    def mouseReleaseEvent(self, event):
        """Handle mouse button release"""
        self.is_dragging = False
        print(f"Mouse released ({event.position().x():.0f}, {event.position().y():.0f})")


class ImageClickDetector(QMainWindow):
    """Main application window"""
    
    def __init__(self, image_path=None):
        super().__init__()
        self.setWindowTitle("Image Click Detector - PyQt6")
        self.setGeometry(100, 100, 1200, 800)
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Create clickable image label
        self.image_label = ClickableImageLabel()
        self.image_label.setMinimumSize(400, 400)
        self.image_label.setStyleSheet("border: 2px solid gray; background-color: #f0f0f0;")
        
        # Connect signals
        self.image_label.clicked.connect(self.on_image_clicked)
        self.image_label.dragged.connect(self.on_image_dragged)
        
        main_layout.addWidget(self.image_label)
        
        # Create status bar
        self.status_label = QStatusBar()
        self.setStatusBar(self.status_label)
        self.status_label.showMessage("Ready")
        
        # Track current image info
        self.current_image_path = image_path
        self.current_image = None
        
        # Load initial image if provided
        if image_path:
            self.load_image_from_path(image_path)
    
    def load_image_from_path(self, file_path):
        """Load image from given file path"""
        try:
            # Use OpenCV to load image
            image = cv2.imread(str(file_path))
            
            if image is None:
                self.status_label.showMessage(f"Failed to load image: {file_path}")
                return
            
            # Convert BGR to RGB for display
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            h, w = image_rgb.shape[:2]
            
            # Store image info
            self.current_image_path = file_path
            self.current_image = image_rgb
            
            # Convert to QImage and display
            bytes_per_line = 3 * w
            qimage = QImage(image_rgb.data.tobytes(), w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
            pixmap = QPixmap.fromImage(qimage)
            
            # Scale to fit in label while maintaining aspect ratio
            scaled_pixmap = pixmap.scaled(
                self.image_label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            
            self.image_label.setPixmap(scaled_pixmap)
            self.image_label.set_image_dimensions(w, h)
            
            # Update UI
            file_name = Path(file_path).name
            self.info_label.setText(f"Loaded: {file_name} ({w}x{h})")
            self.status_label.showMessage(f"Image loaded: {file_path}")
            
            print(f"\n{'='*60}")
            print(f"Image loaded: {file_path}")
            print(f"Dimensions: {w}x{h} pixels")
            print(f"{'='*60}")
            print("Ready to click and drag on the image...")
            print("Left-click: Record single click")
            print("Right-click: Record right-click")
            print("Left-drag: Drag with left mouse button")
            print(f"{'='*60}\n")
            
        except Exception as e:
            self.status_label.showMessage(f"Error loading image: {str(e)}")
            print(f"Error: {e}")
    
    def on_image_clicked(self, x, y, button):
        """Handle image click"""
        print(f"[CLICK] {button.upper():6} | Image Coordinates: ({x:4d}, {y:4d})")
        self.status_label.showMessage(f"Clicked at ({x}, {y}) - {button} button")
    
    def on_image_dragged(self, x, y, button):
        """Handle image drag"""
        print(f"[DRAG]  {button.upper():6} | Image Coordinates: ({x:4d}, {y:4d})")
        self.status_label.showMessage(f"Dragging at ({x}, {y}) - {button} button")


def main():
    app = QApplication(sys.argv)
    
    # Hardcoded image path
    image_path = "src/realsense_publisher/image/realsense_capture_20251027_142056_color.png"
    
    window = ImageClickDetector(image_path)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
