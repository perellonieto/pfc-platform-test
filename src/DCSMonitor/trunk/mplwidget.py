#******************************************************************************
#  FILE          : mplwidget.py
#  DESCRIPTION   : main program
#  CPU TYPE      : 
#  AUTHOR        : Miquel Perello Nieto
#  PROJECT       : 
#  COMPANY       : Automatic Control Department,
#                 Technical University of Catalonia
#
#     REVISION HISTORY:
#              VERSION: 1
#               AUTHOR: Miquel Perello Nieto
#                 DATE: December 2011
#             COMMENTS: 
# *****************************************************************************/
# Python Qt4 bindings for GUI objects
from PyQt4 import QtGui

# import the Qt4Agg FigureCanvas object, that binds Figure to
# Qt4Agg backend. It also inherits from QWidget
from matplotlib.backends.backend_qt4agg \
    import FigureCanvasQTAgg as FigureCanvas

# Matplotlib Figure object
from matplotlib.figure import Figure

class MplCanvas(FigureCanvas):
    """Class to represent the FigureCanvas widget"""
    def __init__(self):
        # setup Matplotlib Figure and Axis
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.grid()
        
        self.fig.set_animated(True)
        #maxy = 500
        #self.ax.set_ylim(-5, 5)
        #self.ax.set_xlim(-2, 0)
        
        #self.ax.set_ylabel('value')
        #self.ax.set_xlabel('time')
        
        # initialization of the canvas
        FigureCanvas.__init__(self, self.fig)
        # we define the widget as expandable
        FigureCanvas.setSizePolicy(self,
                                    QtGui.QSizePolicy.Expanding,
                                    QtGui.QSizePolicy.Expanding)
        # notify the system of updated policy
        FigureCanvas.updateGeometry(self)

class MplWidget(QtGui.QWidget):
    """Widget defined in Qt Designer"""
    def __init__(self, parent = None):
        # initialization of Qt MainWindow widget
        QtGui.QWidget.__init__(self, parent)
        # set the canvas to the Matplotlib widget
        self.canvas = MplCanvas()
        # create a vertical box layout
        self.vbl = QtGui.QVBoxLayout()
        # add mpl widget to vertical box
        self.vbl.addWidget(self.canvas)
        # set the layout to th vertical box
        self.setLayout(self.vbl)
