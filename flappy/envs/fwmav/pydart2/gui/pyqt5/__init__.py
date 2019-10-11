from __future__ import absolute_import

try:
    from pydart2.gui.pyqt5 import window
    from pydart2.gui.pyqt5 import side_panel

    assert window
    assert side_panel
except Exception:
    print("failed to load pyqt5")
