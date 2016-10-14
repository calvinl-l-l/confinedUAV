import sys
import data_displayer_form as form
from pyqtgraph import QtGui, QtCore
from pyqtgraph.ptime import time
import pyqtgraph as pg

app = QtGui.QApplication([])
UI = form.disp_form()
UI.show()

def update():
    UI.interface()
    # UI.update_plot()

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(100)


if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        app.instance().exec_()