#!/usr/bin/python

# ================================================= #
#    Distributed Control Systems Monitor Main        #
# ================================================= #

# used to parse files more easily
from __future__ import with_statement

# Numpy module
import numpy as np

# for command-line arguments
import sys

# to generate random numbers
import random

# for communicate with pic
import serial

# for recive data from pic
import array

# for sleep
import time

# to unpack the data that sends dsPIC
import struct

# Qt4 bindings for core Qt functionalities (non-GUI)
from PyQt4 import QtCore
# Python Qt4 bindings for GUI objects
from PyQt4 import QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

# import the MainWindow widget from the converted .ui files
from dcsmmainwindow import Ui_MainWindow

class DesignerMainWindow(QtGui.QMainWindow, Ui_MainWindow):
    """Customization for Qt Designer created window"""
    graph_timer = QtCore.QTimer()
    data_timer = QtCore.QTimer()
    updated_data = 0
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=10)
    # TODO : he iniciat el valor de time, nomes per veure una grafica bona
    time = 0
    
    # values of microcontroller
    graf_t  = array.array('f')    # time value        Unsigned Long (4bytes)
    graf_r  = array.array('f')    # reference value    float (4bytes) 
    graf_x0 = array.array('f')    # x0 value            float (4bytes) 
    graf_x1 = array.array('f')    # x1 value            float (4bytes)
    graf_u  = array.array('f')    # u value            float (4bytes)
    
    #values for the graphic
    #xdata, ydata, ymax, ywarn, ymin, ycrit = [], [], [], [], [], []
    

    
    def __init__(self, parent = None):
        # initialization of the superclass
        super(DesignerMainWindow, self).__init__(parent)
        # setup the GUI --> function generated by pyuic4
        self.setupUi(self)
        
        # connect the signals with the slots
        QtCore.QObject.connect(self.pushButton_reload, QtCore.SIGNAL("clicked()"), self.update_graph)
        QtCore.QObject.connect(self.graph_timer, QtCore.SIGNAL("timeout()"), self.update_graph)
        QtCore.QObject.connect(self.data_timer, QtCore.SIGNAL("timeout()"), self.calculate_data)
        QtCore.QObject.connect(self.textBrowser, QtCore.SIGNAL("textChanged()"), self.move_scroll_bar_down)
        #QtCore.QObject.connect(self.mplactionOpen, QtCore.SIGNAL('triggered()'), self.select_file)
        #QtCore.QObject.connect(self.mplactionQuit, QtCore.SIGNAL('triggered()'), QtGui.qApp, QtCore.SLOT("quit()"))
        
        # clear data of the serial port
        self.ser.flushInput()
        # start timer data calculate
        self.data_timer.start(20)
        # start timer graphic refresh
        self.graph_timer.start(100)
        

        self.referenceLine, = self.mplWidget.canvas.ax.plot([], [], animated=True, lw=2, color='blue',      label="r")
        self.x0Line,        = self.mplWidget.canvas.ax.plot([], [], animated=True, lw=1, color='grey',      label="x0")
        self.x1Line,        = self.mplWidget.canvas.ax.plot([], [], animated=True, lw=1, color='orange',    label="x1")
        self.uLine,         = self.mplWidget.canvas.ax.plot([], [], animated=True, lw=1, color='red',       label="u")
        
    
    def generate_data(self):
        """Parse a text file to extract letters frequencies"""
        
        data = array.array('c')
        data.append(self.ser.read(1))
        while data[0] != chr(1):
            data[0] = self.ser.read(1)
    
        data = self.ser.read(23-1)

        t  = struct.unpack('I', data[3]+data[2]+data[1]+data[0])
        r  = struct.unpack('f', data[4]+data[5]+data[6]+data[7])
        x0 = struct.unpack('f', data[8]+data[9]+data[10]+data[11])
        x1 = struct.unpack('f', data[12]+data[13]+data[14]+data[15])
        u  = struct.unpack('f', data[16]+data[17]+data[18]+data[19])
        
        #TODO: he cambiat el valor de les coordenades de temps
        #self.time = t[0]*25e-9
        self.time = self.time+0.01
        
        aux_str  = " t = "+str(self.time)+"\t"
        aux_str += " r = "+str(r[0])+"\t"
        aux_str += " x0 = "+str(x0[0])+"\t"
        aux_str += " x1 = "+str(x1[0])+"\t"
        aux_str += " u = "+str(u[0])+"\n"
        
        self.textBrowser.insertPlainText(aux_str)
        
        self.graf_t.append(self.time)
        self.graf_r.append(r[0])
        self.graf_x0.append(x0[0])
        self.graf_x1.append(x1[0])
        self.graf_u.append(u[0])
        
        self.referenceLine.set_data(self.graf_t, self.graf_r)
        self.x0Line.set_data(self.graf_t, self.graf_x0)
        self.x1Line.set_data(self.graf_t, self.graf_x1)
        self.uLine.set_data(self.graf_t, self.graf_u)
        
        # reload number of samples lavel
        self.label_samples_value.setText(str(self.graf_t.buffer_info()[1]))
        
        # 
        self.updated_data = 1
        
    
    def update_graph(self):
        """Updates the graph with new letters frequencies"""
        if self.updated_data == 1 :
            # clear the Axes
            #self.mplWidget.canvas.ax.clear()
            
            # enable grid only on the Y axis
            #self.mplWidget.canvas.ax.get_yaxis().grid(True)
            self.mplWidget.canvas.ax.set_ylim(-1, 1)
            self.mplWidget.canvas.ax.set_xlim(self.time-1.5, self.time)
            
            # force an image redraw
            self.mplWidget.canvas.draw()
            
            try:
                #Draw the lines
                if self.checkBox_R.isChecked():
                    self.mplWidget.canvas.ax.draw_artist(self.referenceLine)
                if self.checkBox_x0.isChecked():
                    self.mplWidget.canvas.ax.draw_artist(self.x0Line)
                if self.checkBox_U.isChecked():
                    self.mplWidget.canvas.ax.draw_artist(self.uLine)
                if self.checkBox_x1.isChecked():
                    self.mplWidget.canvas.ax.draw_artist(self.x1Line)
            except AssertionError:
                pass
            try:
                self.mplWidget.canvas.blit(self.mplWidget.canvas.ax.bbox)
            except AttributeError:
                pass
                     
            
            self.updated_data = 0
            
            
        # activate the periodical update graph
        self.graph_timer.start(100)
        
    def calculate_data(self):
        """Updates the data"""
        if self.updated_data == 0 :
            # get the letters frequencies
            self.generate_data()
            
            self.updated_data = 1
            # activate the periodical update graph
            self.data_timer.start(100)
        else:
            # activate the periodical update graph
            self.data_timer.start(20)
        
    def update_percent(self):
        """Updates the value percentage of Bus Saturation"""
        
    def move_scroll_bar_down(self):
        """Moves down the scroll bar"""
        scroll = self.textBrowser.verticalScrollBar()
        scroll.setSliderPosition(scroll.maximum())
        
          
# create the GUI application
app = QtGui.QApplication(sys.argv)

# instantiate the main window
dmw = DesignerMainWindow()

# show it
dmw.show()

# start the Qt main loop execution, exiting from this script
# with the same return code of Qt application
sys.exit(app.exec_())