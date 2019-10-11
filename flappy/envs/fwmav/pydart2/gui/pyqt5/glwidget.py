from __future__ import division
from __future__ import print_function
from __future__ import absolute_import
from past.utils import old_div
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import OpenGL.GL as GL
import OpenGL.GLU as GLU
# import OpenGL.GLUT as GLUT
from PyQt5 import QtGui
from PyQt5 import QtCore
from PyQt5 import QtWidgets
from PyQt5.QtOpenGL import QGLWidget, QGLFormat
from pydart2.gui import trackball
# import time
from pydart2.gui.opengl.scene import OpenGLScene
import numpy as np
# from numpy.linalg import norm


class GLWidget(QtWidgets.QOpenGLWidget):

    def __init__(self, parent=None):
        super(GLWidget, self).__init__(parent)

        fmt = QtGui.QSurfaceFormat()
        # fmt.setSampleBuffers(True)
        fmt.setSamples(4)
        self.setFormat(fmt)

        self.width = 1280
        self.height = 720

        self.lastPos = None
        self.zoom = -1.2

        self.sim = None
        self.viewer = None
        self.captureIndex = 0

        self.scene = OpenGLScene(self.width, self.height, parent)
        self.renderer = self.scene.renderer

        self.lock_camera = False

    def sizeHint(self):
        return QtCore.QSize(self.width, self.height)

    def paintGL(self):
        self.scene.render(self.sim)

    def resizeGL(self, w, h):
        self.scene.resize(w, h)

    def initializeGL(self):
        self.scene.init()

    def set_lock_camera(self, lock=True):
        self.lock_camera = lock

    def mousePressEvent(self, event):
        self.lastPos = event.pos()

        pos = np.array([event.pos().x(), event.pos().y()],
                       dtype=np.float64)
        self.viewer.safe_call_callback("on_mouse_press", pos)

    def mouseReleaseEvent(self, event):
        self.lastPos = None
        self.viewer.safe_call_callback("on_mouse_release")

    def mouseMoveEvent(self, event):
        # (w, h) = (self.width, self.height)
        x = event.x()
        y = event.y()
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        if not self.lock_camera:
            modifiers = QtWidgets.QApplication.keyboardModifiers()
            if modifiers == QtCore.Qt.ShiftModifier:
                self.scene.tb.zoom_to(dx, -dy)
            elif modifiers == QtCore.Qt.ControlModifier:
                self.scene.tb.trans_to(dx, -dy)
            else:
                self.scene.tb.drag_to(x, y, dx, -dy)

        p0 = np.array([self.lastPos.x(), self.lastPos.y()],
                      dtype=np.float64)
        p1 = np.array([event.pos().x(), event.pos().y()],
                      dtype=np.float64)
        self.viewer.safe_call_callback("on_mouse_move", p0, p1)

        self.lastPos = event.pos()
        # self.paintGL()
        self.update()

    def capture(self, name=None):
        data = GL.glReadPixels(0, 0,
                               self.width, self.height,
                               GL.GL_RGBA,
                               GL.GL_UNSIGNED_BYTE)
        img = QtGui.QImage(data, self.width, self.height,
                           QtGui.QImage.Format_RGBA8888_Premultiplied)
        T = QtGui.QTransform()
        T.scale(1.0, -1.0)
        img = img.transformed(T)

        if name is None:
            name = 'frame'
        dir = self.viewer.capture_dir()
        filename = '%s/%s.%04d.png' % (dir, name, self.captureIndex)
        img.save(filename)
        print('Capture to %s' % filename)
        self.captureIndex += 1
