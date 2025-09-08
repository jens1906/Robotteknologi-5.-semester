import sys
import math
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QLabel, QPushButton
from PyQt5.QtOpenGL import QGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import glutInit

glutInit()

class CylinderWidget(QGLWidget):
    def __init__(self):
        super().__init__()
        self.height = 1.0
        self.angle = 0.0
        self.cylinder_diameter = 6.5  # meters (default)
        self.cylinder_radius = self.cylinder_diameter / 2.0
        self.cylinder_height = 5.0  # meters (fixed)
        self.box_size = 0.25  # meters, rover size
        self.ur3e_joints = [0, 0, 0, 0, 0, 0]
        self.point = None
        self.last_mouse_pos = None
        self.camera_azimuth = 0.0
        self.camera_elevation = 20.0
        self.camera_distance = 8  # Initial camera distance

    def set_point(self, height, angle, ur3e_joints=None, diameter=None):
        self.height = height
        self.angle = angle
        if diameter is not None:
            self.cylinder_diameter = diameter
            self.cylinder_radius = diameter / 2.0
        if ur3e_joints is not None:
            self.ur3e_joints = ur3e_joints
        self.update()

    def initializeGL(self):
        glClearColor(0.9, 0.9, 0.9, 1)
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        gluPerspective(45, 1.0, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h if h != 0 else 1, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def mousePressEvent(self, event):
        self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self.last_mouse_pos is not None:
            dx = event.x() - self.last_mouse_pos.x()
            dy = event.y() - self.last_mouse_pos.y()
            self.camera_azimuth -= dx * 0.5  # Invert left/right
            self.camera_elevation += dy * 0.5
            self.camera_elevation = max(-89, min(89, self.camera_elevation))
            self.last_mouse_pos = event.pos()
            self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y() / 120  # One notch = 120
        self.camera_distance -= delta
        self.camera_distance = max(2, min(30, self.camera_distance))
        self.update()

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        # Calculate camera position based on azimuth and elevation
        r = self.camera_distance
        az = math.radians(self.camera_azimuth)
        el = math.radians(self.camera_elevation)
        cam_x = r * math.cos(el) * math.sin(az)
        cam_y = -r * math.cos(el) * math.cos(az)
        cam_z = r * math.sin(el) + 2
        gluLookAt(cam_x, cam_y, cam_z, 0, 0, 2, 0, 0, 1)
        # Draw ground plane
        glColor3f(0.7, 0.7, 0.7)
        glBegin(GL_QUADS)
        glVertex3f(-5, -5, 0)
        glVertex3f(5, -5, 0)
        glVertex3f(5, 5, 0)
        glVertex3f(-5, 5, 0)
        glEnd()
        # Draw axes
        glLineWidth(2)
        glBegin(GL_LINES)
        # X axis (red)
        glColor3f(1, 0, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(2, 0, 0)
        # Y axis (green)
        glColor3f(0, 1, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 2, 0)
        # Z axis (blue)
        glColor3f(0, 0, 1)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 2)
        glEnd()
        glLineWidth(1)
        # Draw cylinder
        glColor3f(0.2, 0.5, 0.8)
        self.draw_cylinder(self.cylinder_radius, self.cylinder_height)
        # Draw coordinate point
        glColor3f(1, 0, 0)
        self.draw_point(self.height, self.angle)

    def draw_cylinder(self, radius, height):
        slices = 64
        glPushMatrix()
        glTranslatef(0, 0, 0)
        quad = gluNewQuadric()
        gluCylinder(quad, radius, radius, height, slices, 1)
        glPopMatrix()

    def draw_cube(self, size):
        hs = size / 2.0
        glBegin(GL_QUADS)
        # Bottom face
        glVertex3f(-hs, -hs, -hs)
        glVertex3f(hs, -hs, -hs)
        glVertex3f(hs, hs, -hs)
        glVertex3f(-hs, hs, -hs)
        # Top face
        glVertex3f(-hs, -hs, hs)
        glVertex3f(hs, -hs, hs)
        glVertex3f(hs, hs, hs)
        glVertex3f(-hs, hs, hs)
        # Front face
        glVertex3f(-hs, -hs, -hs)
        glVertex3f(hs, -hs, -hs)
        glVertex3f(hs, -hs, hs)
        glVertex3f(-hs, -hs, hs)
        # Back face
        glVertex3f(-hs, hs, -hs)
        glVertex3f(hs, hs, -hs)
        glVertex3f(hs, hs, hs)
        glVertex3f(-hs, hs, hs)
        # Left face
        glVertex3f(-hs, -hs, -hs)
        glVertex3f(-hs, hs, -hs)
        glVertex3f(-hs, hs, hs)
        glVertex3f(-hs, -hs, hs)
        # Right face
        glVertex3f(hs, -hs, -hs)
        glVertex3f(hs, hs, -hs)
        glVertex3f(hs, hs, hs)
        glVertex3f(hs, -hs, hs)
        glEnd()

    def draw_point(self, height, angle):
        # Convert user input to cylinder coordinates
        if 0 <= height <= self.cylinder_height:
            theta = math.radians(angle)
            x = self.cylinder_radius * math.cos(theta)
            y = self.cylinder_radius * math.sin(theta)
            z = height
            glPushMatrix()
            glTranslatef(x, y, z)
            glRotatef(angle, 0, 0, 1)
            # Draw box (rover) with correct size
            glPushMatrix()
            glColor3f(0.6, 0.4, 0.2)
            self.draw_cube(self.box_size)
            glPopMatrix()
            # Move to radial face of box (outward from cylinder)
            glTranslatef(self.box_size/2, 0, 0)
            glRotatef(90, 0, 1, 0)
            # Draw UR3e arm
            self.draw_ur3e(self.ur3e_joints)
            glPopMatrix()

    def draw_ur3e(self, joints):
        # UR3e DH parameters (all in meters)
        dh_params = [
            {'a': 0,        'd': 0.15185, 'alpha': math.pi/2},
            {'a': -0.24355, 'd': 0,       'alpha': 0},
            {'a': -0.2132,  'd': 0,       'alpha': 0},
            {'a': 0,        'd': 0.13105, 'alpha': math.pi/2},
            {'a': 0,        'd': 0.08535, 'alpha': -math.pi/2},
            {'a': 0,        'd': 0.0921,  'alpha': 0},
        ]
        """
        # UR5e DH parameters (all in meters)
        dh_params = [
            {'a': 0,        'd': 0.1625,  'alpha': math.pi/2},
            {'a': -0.425,   'd': 0,       'alpha': 0},
            {'a': -0.3922,  'd': 0,       'alpha': 0},
            {'a': 0,        'd': 0.1333,  'alpha': math.pi/2},
            {'a': 0,        'd': 0.0997,  'alpha': -math.pi/2},
            {'a': 0,        'd': 0.0996,  'alpha': 0},
        ]
        """
        glPushMatrix()
        glColor3f(0.3, 0.3, 0.3)
        self.draw_cylinder_segment(0.04, 0.05)  # base (diameter 8cm, height 5cm)
        glTranslatef(0, 0, 0.05)
        for i, params in enumerate(dh_params):
            glRotatef(joints[i], 0, 0, 1)
            glColor3f(0.8, 0.2, 0.2)
            self.draw_sphere(0.03)  # joint (diameter 6cm)
            # Draw 'a' link (horizontal offset)
            if abs(params['a']) > 1e-4:
                glColor3f(0.2, 0.2, 0.8)
                self.draw_cylinder_segment(0.02, abs(params['a']), axis='x', direction=-1 if params['a'] < 0 else 1)
            glTranslatef(params['a'], 0, 0)
            glRotatef(params['alpha'] * 180 / math.pi, 1, 0, 0)
            # Draw 'd' link (vertical offset)
            if abs(params['d']) > 1e-4:
                glColor3f(0.2, 0.8, 0.2)
                self.draw_cylinder_segment(0.02, params['d'])
            glTranslatef(0, 0, params['d'])
        glPopMatrix()

    def draw_cylinder_segment(self, radius, height, axis='z', direction=1):
        quad = gluNewQuadric()
        glPushMatrix()
        if axis == 'x':
            glRotatef(90 * direction, 0, 1, 0)
        gluCylinder(quad, radius, radius, height, 20, 1)
        glPopMatrix()

    def draw_sphere(self, radius):
        quad = gluNewQuadric()
        gluSphere(quad, radius, 20, 20)

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Cylinder Coordinate Visualizer')
        self.setGeometry(100, 100, 800, 600)
        layout = QVBoxLayout()
        self.cylinder_widget = CylinderWidget()
        layout.addWidget(self.cylinder_widget)
        # First row: diameter, height, angle
        input_row1 = QHBoxLayout()
        self.diameter_input = QLineEdit()
        self.height_input = QLineEdit()
        self.angle_input = QLineEdit()
        input_row1.addWidget(QLabel('Diameter (m):'))
        input_row1.addWidget(self.diameter_input)
        input_row1.addWidget(QLabel('Height (m):'))
        input_row1.addWidget(self.height_input)
        input_row1.addWidget(QLabel('Angle (deg):'))
        input_row1.addWidget(self.angle_input)
        layout.addLayout(input_row1)
        # Second row: arm joints
        input_row2 = QHBoxLayout()
        self.ur3e_joint_inputs = [QLineEdit() for _ in range(6)]
        for i, joint_input in enumerate(self.ur3e_joint_inputs):
            input_row2.addWidget(QLabel(f'Joint {i+1} (deg):'))
            input_row2.addWidget(joint_input)
        self.set_btn = QPushButton('Set Point')
        self.set_btn.clicked.connect(self.set_point)
        input_row2.addWidget(self.set_btn)
        layout.addLayout(input_row2)
        self.setLayout(layout)

    def set_point(self):
        try:
            # Use current values if input is empty
            diameter_val = float(self.diameter_input.text()) if self.diameter_input.text().strip() else self.cylinder_widget.cylinder_diameter
            height_val = float(self.height_input.text()) if self.height_input.text().strip() else self.cylinder_widget.height
            angle_val = float(self.angle_input.text()) if self.angle_input.text().strip() else self.cylinder_widget.angle
            joints = []
            for i, j in enumerate(self.ur3e_joint_inputs):
                text = j.text()
                if text.strip() == '':
                    joints.append(self.cylinder_widget.ur3e_joints[i])
                else:
                    joints.append(float(text))
            self.cylinder_widget.set_point(height_val, angle_val, joints, diameter_val)
        except ValueError:
            pass

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())