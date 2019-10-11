from PyQt4 import QtCore
from PyQt4 import QtGui


class SidePanel(object):

    def __init__(self, width=300):
        self.width = width
        self.widgets = dict()

        self.layout = list()  # layout of widgets
        self.line = list()  # temporary buffer
        self.vbox = None

    def register_widget(self, widget, name, next_line):
        self.widgets[name] = widget
        self.line.append(widget)
        if next_line:
            self.process_line()
            self.line = list()

    def process_line(self,):
        if len(self.line) == 0:
            return
        self.layout.append(self.line)

    def push_line(self,):
        self.process_line()
        self.line = list()

    def build_layout(self,):
        self.process_line()  # Push if not processed yet
        self.vbox = QtGui.QVBoxLayout()
        for i, line in enumerate(self.layout):
            if len(line) == 1:
                self.vbox.addWidget(line[0])
            elif len(line) > 1:
                hbox = QtGui.QHBoxLayout()
                for w in line:
                    hbox.addWidget(w)
                self.vbox.addLayout(hbox)
        self.vbox.addStretch(1)
        return self.vbox

    def add_label(self, name, label=None, next_line=True):
        label = name if label is None else label
        w = QtGui.QLabel(label)
        self.register_widget(w, name, next_line)
        return w

    def add_push_button(self, name, callback=None, label=None, next_line=True):
        label = name if label is None else label
        w = QtGui.QPushButton(label)
        if callback is not None:
            QtCore.QObject.connect(w, QtCore.SIGNAL('clicked()'), callback)
        self.register_widget(w, name, next_line)
        return w

    def add_toggle_button(self, name, default=True,
                          callback=None, label=None,
                          next_line=True):
        w = self.add_push_button(name,
                                 callback=callback,
                                 label=label,
                                 next_line=next_line)
        w.setCheckable(True)
        if default is not None:
            w.setChecked(default)
        return w

    def add_checkbox(self, name, default=True,
                     callback=None, label=None,
                     next_line=True):
        label = name if label is None else label
        w = QtGui.QCheckBox(label)
        self.register_widget(w, name, next_line)

        if callback is not None:
            QtCore.QObject.connect(w, QtCore.SIGNAL('clicked()'), callback)
        if default is not None:
            w.setChecked(default)
        return w

    def add_combobox(self, name, items, default=0,
                     label=True,
                     callback=None, next_line=True):
        if label:
            self.add_label(label if isinstance(label, str)
                           else name, next_line=False)

        w = QtGui.QComboBox()
        w.addItems(items)
        w.setCurrentIndex(default)
        if callback is not None:
            QtCore.QObject.connect(w,
                                   QtCore.SIGNAL('currentIndexChanged(int)'),
                                   callback)
        self.register_widget(w, name, next_line)
        return w

    def add_spinbox(self, name, min, max, default=None, step=None,
                    label=True,
                    prefix=None, suffix=None, callback=None, next_line=True):
        if label:
            self.add_label(label if isinstance(label, str)
                           else name, next_line=False)
        w = QtGui.QSpinBox()
        w.setMinimum(min)
        w.setMaximum(max)
        if default is not None:
            w.setValue(default)
        if step is not None:
            w.setSingleStep(step)
        if prefix is not None:
            w.setPrefix(prefix)
        if suffix is not None:
            w.setSuffix(suffix)
        if callback is not None:
            QtCore.QObject.connect(w,
                                   QtCore.SIGNAL('valueChanged(int)'),
                                   callback)
        self.register_widget(w, name, next_line)
        return w

    def add_double_spinbox(self, name, min, max, default=None, step=None,
                           label=True,
                           prefix=None, suffix=None, callback=None,
                           next_line=True):
        if label:
            self.add_label(label if isinstance(label, str)
                           else name, next_line=False)
        w = QtGui.QDoubleSpinBox()
        w.setMinimum(min)
        w.setMaximum(max)
        if default is not None:
            w.setValue(default)
        if step is not None:
            w.setSingleStep(step)
        if prefix is not None:
            w.setPrefix(prefix)
        if suffix is not None:
            w.setSuffix(suffix)
        if callback is not None:
            QtCore.QObject.connect(w,
                                   QtCore.SIGNAL('valueChanged(double)'),
                                   callback)
        self.register_widget(w, name, next_line)
        return w

    def set_text(self, name, label):
        self.widgets[name].setText(label)

    def value(self, name):
        return self.widgets[name].value()

    def set_value(self, name, value):
        self.widgets[name].setValue(value)

    def is_checked(self, name):
        return self.widgets[name].isChecked()

    def set_checked(self, name, value):
        self.widgets[name].setChecked(value)

    def set_enabled(self, name, enabled=True):
        self.widgets[name].setEnabled(enabled)
