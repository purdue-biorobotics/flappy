# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group

from __future__ import print_function
from __future__ import absolute_import
from builtins import chr
from builtins import str
import sys
import os
import signal

# import numpy as np
# import math
# import time
# import inspect


import logging

import OpenGL.GLUT
from PyQt4 import QtGui
from PyQt4 import QtCore
from .glwidget import GLWidget
from pydart2.gui.trackball import Trackball


def signal_handler(signal, frame):
    print('You pressed Ctrl+C! Bye.')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class PyQt4Window(QtGui.QMainWindow):
    GLUT_INITED = False

    def __init__(self, sim=None, title=None):
        if not PyQt4Window.GLUT_INITED:
            OpenGL.GLUT.glutInit(sys.argv)

        if title is None:
            title = "Simulation Viewer"
        self.app = QtGui.QApplication([title])

        super(PyQt4Window, self).__init__()

        # setup_logger()
        self.logger = logging.getLogger(__name__)
        self.init_callbacks()
        self.set_simulation(sim)
        self.logger.info('sim = [%s]' % str(self.sim))

        # Check and create captures directory
        self.set_capture_rate(100)
        self.set_capture_dir('data/captures')

        self.initActions()
        self.initToolbar()
        self.initMenu()
        self.initUI()
        # self.init_cameras()

        self.idleTimer = QtCore.QTimer()
        self.idleTimer.timeout.connect(self.idleTimerEvent)
        self.idleTimer.start(0)

        self.renderTimer = QtCore.QTimer()
        self.renderTimer.timeout.connect(self.renderTimerEvent)
        self.renderTimer.start(80)

        # self.camera_event(0)

        self.after_reset = True
        self.set_capture_rate(100)

        self.input_keys = list()
        self.topLeft()

    def run_application(self,):
        self.show()
        self.app.exec_()

    def closeEvent(self, event):
        self.safe_call_callback("on_close")
        super(PyQt4Window, self).closeEvent(event)

    def topLeft(self):
        frameGm = self.frameGeometry()
        desktop = QtGui.QApplication.desktop()
        screen = desktop.screenNumber(desktop.cursor().pos())
        # centerPoint = desktop.screenGeometry(screen).center()
        # frameGm.moveCenter(centerPoint)
        topLeftPoint = desktop.screenGeometry(screen).topLeft()
        frameGm.moveTopLeft(topLeftPoint)
        self.move(frameGm.topLeft())

    def get_simulation(self):
        return self.sim

    def set_simulation(self, sim):
        self.sim = sim
        if hasattr(self, "glwidget"):
            self.glwidget.sim = sim
        self.read_default_callbacks_from_sim()

    def capture_rate(self,):
        return self._capture_rate

    def set_capture_rate(self, _capture_rate):
        self._capture_rate = _capture_rate

    def capture_dir(self,):
        return self._capture_dir

    def set_capture_dir(self, _capture_dir):
        self._capture_dir = _capture_dir

    def initUI(self):
        TOOLBOX_HEIGHT = 30
        STATUS_HEIGHT = 30
        PANEL_WIDTH = 0
        self.setGeometry(0, 0, 1280 + PANEL_WIDTH,
                         720 + TOOLBOX_HEIGHT + STATUS_HEIGHT)
        # self.setWindowTitle('Toolbar')

        self.ui = QtGui.QWidget(self)
        self.ui.setGeometry(0, TOOLBOX_HEIGHT, 1280 + PANEL_WIDTH, 720)

        self.hbox = QtGui.QHBoxLayout()

        self.glwidget = GLWidget(self)
        self.glwidget.sim = self.sim
        self.glwidget.viewer = self
        self.glwidget.setGeometry(0, 0, 1280, 720)

        self.hbox.addWidget(self.glwidget)
        self.ui.setLayout(self.hbox)

    def add_right_panel(self, panel):
        TOOLBOX_HEIGHT = 30
        STATUS_HEIGHT = 30
        PANEL_WIDTH = panel.width

        self.setGeometry(0, 0, 1280 + PANEL_WIDTH,
                         720 + TOOLBOX_HEIGHT + STATUS_HEIGHT)
        self.ui.setGeometry(0, TOOLBOX_HEIGHT, 1280 + PANEL_WIDTH, 720)
        self.glwidget.setGeometry(0, 0, 1280, 720)
        self.glwidget.resizeGL(1280, 720)

        self.right_panel = panel
        layout = panel.build_layout()
        self.hbox.addLayout(layout)

        self.ui.setLayout(self.hbox)

    def init_callbacks(self,):
        self.callback_events = ["reset", "step", "render_with_ri",
                                "name",
                                "draw_with_ri",
                                "num_frames", "set_frame",
                                "status", "on_close", "on_key_press",
                                "on_mouse_press", "on_mouse_move",
                                "on_mouse_release"]
        self.callbacks = dict()
        for evt in self.callback_events:
            self.callbacks[evt] = None

    def read_default_callbacks_from_sim(self,):
        self.logger.info('read_default_callbacks_from_sim')
        if self.sim is None:
            return

        for evt in self.callback_events:
            if hasattr(self.sim, evt):
                self.callbacks[evt] = getattr(self.sim, evt)
            else:
                self.callbacks[evt] = None

    def callback(self, evt):
        return self.callbacks[evt]

    def has_callback(self, evt):
        return self.callbacks[evt] is not None

    def set_callback(self, evt, func):
        self.callbacks[evt] = func

    def safe_call_callback(self, evt, *args):
        func = self.callbacks[evt]
        if func is None:
            return None
        return func(*args)

    def callbacks_as_string(self,):
        return "\n".join(["\t[%s: %s]" % (str(evt), str(func))
                          for evt, func in self.callbacks.items()])

    def num_cameras(self,):
        return self.glwidget.scene.num_cameras()

    def replace_camera(self, idx, trackball):
        return self.glwidget.scene.replace_camera(idx, trackball)

    def add_camera(self, trackball, name):
        return self.glwidget.scene.add_camera(trackball, name)

    def add_camera_event(self, scene, trackball, name):
        cam_id = scene.num_cameras() - 1
        if name is None:
            name = 'Camera %d' % cam_id
        action = QtGui.QAction(name, self)
        action.triggered.connect(lambda: self.camera_event(cam_id))
        self.cameraMenu.addAction(action)
        print("add_camera_event: ID = %d name = %s" % (cam_id, name))

    def initActions(self):
        # Create actions
        self.resetAction = QtGui.QAction('Reset', self)
        self.resetAction.triggered.connect(self.resetEvent)

        self.stepAction = QtGui.QAction('Step', self)
        self.stepAction.triggered.connect(self.stepEvent)

        self.playAction = QtGui.QAction('Play', self)
        self.playAction.setCheckable(True)
        self.playAction.setShortcut('Space')

        self.animAction = QtGui.QAction('Anim', self)
        self.animAction.setCheckable(True)

        self.captureAction = QtGui.QAction('Capture', self)
        self.captureAction.setCheckable(True)

        self.emptyAction = QtGui.QAction('Empty', self)
        self.emptyAction.triggered.connect(self.emptyEvent)

        self.movieAction = QtGui.QAction('Movie', self)
        self.movieAction.triggered.connect(self.movieEvent)

        self.screenshotAction = QtGui.QAction('Screenshot', self)
        self.screenshotAction.triggered.connect(self.screenshotEvent)

        self.printCamAction = QtGui.QAction('Print Camera', self)
        self.printCamAction.triggered.connect(self.printCamEvent)

    def initToolbar(self):
        # Create a toolbar
        self.toolbar = self.addToolBar('Control')
        self.toolbar.addAction(self.resetAction)
        self.toolbar.addAction(self.stepAction)
        self.toolbar.addAction(self.playAction)
        self.toolbar.addAction(self.animAction)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.screenshotAction)
        self.toolbar.addAction(self.captureAction)
        self.toolbar.addAction(self.emptyAction)
        self.toolbar.addAction(self.movieAction)

        self.rangeSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.rangeSlider.valueChanged[int].connect(self.rangeSliderEvent)
        self.toolbar.addWidget(self.rangeSlider)

    def initMenu(self):
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        fileMenu.addSeparator()

        # Camera menu
        self.cameraMenu = menubar.addMenu('&Camera')
        self.cameraMenu.addAction(self.printCamAction)
        self.cameraMenu.addSeparator()

        # Recording menu
        recordingMenu = menubar.addMenu('&Recording')
        recordingMenu.addAction(self.screenshotAction)
        recordingMenu.addSeparator()
        recordingMenu.addAction(self.captureAction)
        recordingMenu.addAction(self.movieAction)

    def idleTimerEvent(self):
        doCapture = False

        # Do animation
        if self.animAction.isChecked():
            v = self.rangeSlider.value()
            v += 1
            if v <= self.rangeSlider.maximum():
                self.rangeSlider.setValue(v)
            else:
                self.animAction.setChecked(False)
            doCapture = True

        # Do play
        elif self.playAction.isChecked():
            bIsDone = self.safe_call_callback('step')
            if (bIsDone is not None) and (not bIsDone):
                print("step() returns False: stop...")
                self.playAction.setChecked(False)
            # capture_rate = 10
            doCapture = (self.sim.frame % self.capture_rate() == 0)
            # doCapture = True

        if self.captureAction.isChecked() and doCapture:
            self.capture()

    def renderTimerEvent(self):
        self.glwidget.updateGL()

        msg = self.safe_call_callback('status')
        if msg is None:
            msg = str(self.sim)
        self.statusBar().showMessage(msg)

        n = self.safe_call_callback('num_frames')
        if n is not None:
            self.rangeSlider.setRange(0, n - 1)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            print('Escape key pressed! Bye.')
            self.close()
            return
        if 0 <= event.key() and event.key() < 256:  # If key is ascii
            key = chr(event.key())
            self.safe_call_callback("on_key_press", key)

    def rangeSliderEvent(self, frame):
        self.safe_call_callback("set_frame", frame)

    def screenshotEvent(self):
        self.glwidget.capture("%s_frame" % self.prob_name())

    def prob_name(self):
        name = self.safe_call_callback("name")
        return name if name is not None else "robot"

    def capture(self, ):
        self.glwidget.capture(self.prob_name())

    def emptyEvent(self):
        cmd = 'rm %s/*.png' % self.capture_dir()
        print('cmd = %s' % cmd)
        os.system(cmd)

    def movieEvent(self):
        """
        Ubuntu tested only: avconv is required
        """
        if not os.path.isdir(self.capture_dir()):
            os.makedirs(self.capture_dir())

        name = self.prob_name()
        # yuv420p for compatibility for outdated codecs
        cmt_fmt = 'avconv -r %d' % self.capture_rate()
        cmt_fmt += ' -i %s/%s.%%04d.png'
        cmt_fmt += ' -pix_fmt yuv420p'
        cmt_fmt += ' %s.mp4'
        print(self.capture_dir())
        cmd = cmt_fmt % (self.capture_dir(), name, name)
        print('cmd = %s' % cmd)
        os.system(cmd)

    def resetEvent(self):
        self.after_reset = True
        self.rangeSlider.setValue(0)
        # if hasattr(self.sim, 'reset'):
        #     self.sim.reset()
        # if self.callbacks['reset'] is not None:
        #     self.callbacks['reset']()
        self.safe_call_callback('reset')

    def stepEvent(self):
        self.safe_call_callback("step")

    def camera_event(self, cam_id):
        self.glwidget.scene.set_camera(cam_id)

    def printCamEvent(self):
        print('printCamEvent')
        print('----')
        print(repr(self.glwidget.scene.tb))
        print('----')

    def run(self, ):
        self.run_application()
