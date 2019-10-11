from __future__ import absolute_import
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group

from pydart2.gui import opengl
assert(opengl)

try:
    from pydart2.gui import glut
    assert(glut)
except Exception:
    print("[Warn] fail to load GLUT -- check the installation")

# try:
#     from pydart2.gui import pyqt4
#     assert(pyqt4)
# except:
#     print("[Warn] fail to load PyQt4 -- check the installation")

try:
    from pydart2.gui import pyqt5
    assert(pyqt5)
except:
    print("[Warn] fail to load PyQt5 (pip install pyqt5)")

try:
    from pydart2.gui import viewer
    assert(viewer)
except:
    print("[Warn] fail to load viewer")


# assert(pydart2.gui.opengl)
# assert(pydart2.gui.pyqt4)

# from pydart2.gui import viewer
# from pydart2.gui import side_panel
# assert viewer
# assert side_panel
