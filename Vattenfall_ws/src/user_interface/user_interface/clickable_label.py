"""Clickable QLabel for detecting mouse clicks and drags with image coordinates"""

from PyQt6.QtWidgets import QLabel
from PyQt6.QtCore import pyqtSignal, Qt


class ClickableImageLabel(QLabel):
    """QLabel that detects mouse clicks and drags with precise image coordinates"""
    clicked = pyqtSignal(int, int, str)  # x, y, button type
    dragged = pyqtSignal(int, int, str)  # x, y while dragging
    released = pyqtSignal(int, int, str)  # x, y when mouse button released
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.image_width = 0
        self.image_height = 0
        self.is_dragging = False
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMouseTracking(True)
    
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
        label_x = int(event.position().x())
        label_y = int(event.position().y())
        image_x, image_y = self._map_coordinates(label_x, label_y)
        
        if image_x is not None and image_y is not None:
            button_name = "left"
            if event.button() == Qt.MouseButton.RightButton:
                button_name = "right"
            elif event.button() == Qt.MouseButton.MiddleButton:
                button_name = "middle"
            
            self.released.emit(image_x, image_y, button_name)
        
        self.is_dragging = False
