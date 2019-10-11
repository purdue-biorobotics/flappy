def launch_window(sim, win, default_camera):
    if default_camera is not None:
        win.scene.set_camera(default_camera)
    win.run()


def launch_pyqt4(sim, title=None, default_camera=None):
    from pydart2.gui.pyqt4.window import PyQt4Window
    win = PyQt4Window(sim, title)
    launch_window(sim, win, default_camera)


def launch_pyqt5(sim, title=None, default_camera=None):
    from pydart2.gui.pyqt5.window import PyQt5Window

    win = PyQt5Window(sim, title)
    launch_window(sim, win, default_camera)


def launch_glut(sim, title=None, default_camera=None):
    from pydart2.gui.glut.window import GLUTWindow
    win = GLUTWindow(sim, title)
    launch_window(sim, win, default_camera)


def launch(sim, title=None, default_camera=None):
    """
    default is PyQt5
    """
    # launch_pyqt4(sim, title, default_camera)
    # launch_pyqt5(sim, title, default_camera)
    launch_glut(sim, title, default_camera)
