# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'dcsmmainwindow.ui'
#
# Created: Wed Dec  7 15:42:19 2011
#      by: PyQt4 UI code generator 4.8.3
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(752, 563)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.verticalLayout_6 = QtGui.QVBoxLayout()
        self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
        self.groupBox_3 = QtGui.QGroupBox(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_3.sizePolicy().hasHeightForWidth())
        self.groupBox_3.setSizePolicy(sizePolicy)
        self.groupBox_3.setMinimumSize(QtCore.QSize(0, 200))
        self.groupBox_3.setObjectName(_fromUtf8("groupBox_3"))
        self.verticalLayout_4 = QtGui.QVBoxLayout(self.groupBox_3)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.mplWidget = MplWidget(self.groupBox_3)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.mplWidget.sizePolicy().hasHeightForWidth())
        self.mplWidget.setSizePolicy(sizePolicy)
        self.mplWidget.setObjectName(_fromUtf8("mplWidget"))
        self.verticalLayout_4.addWidget(self.mplWidget)
        self.verticalLayout_6.addWidget(self.groupBox_3)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.label_saturation = QtGui.QLabel(self.centralwidget)
        self.label_saturation.setObjectName(_fromUtf8("label_saturation"))
        self.horizontalLayout.addWidget(self.label_saturation)
        self.slider_saturation = QtGui.QSlider(self.centralwidget)
        self.slider_saturation.setOrientation(QtCore.Qt.Horizontal)
        self.slider_saturation.setObjectName(_fromUtf8("slider_saturation"))
        self.horizontalLayout.addWidget(self.slider_saturation)
        self.label_percent_value = QtGui.QLabel(self.centralwidget)
        self.label_percent_value.setMinimumSize(QtCore.QSize(18, 0))
        self.label_percent_value.setLineWidth(2)
        self.label_percent_value.setMidLineWidth(2)
        self.label_percent_value.setText(_fromUtf8("0"))
        self.label_percent_value.setTextFormat(QtCore.Qt.AutoText)
        self.label_percent_value.setScaledContents(True)
        self.label_percent_value.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_percent_value.setMargin(0)
        self.label_percent_value.setObjectName(_fromUtf8("label_percent_value"))
        self.horizontalLayout.addWidget(self.label_percent_value)
        self.label_percent_symbol = QtGui.QLabel(self.centralwidget)
        self.label_percent_symbol.setObjectName(_fromUtf8("label_percent_symbol"))
        self.horizontalLayout.addWidget(self.label_percent_symbol)
        self.verticalLayout_6.addLayout(self.horizontalLayout)
        self.textBrowser = QtGui.QTextBrowser(self.centralwidget)
        self.textBrowser.setMaximumSize(QtCore.QSize(16777215, 200))
        self.textBrowser.setObjectName(_fromUtf8("textBrowser"))
        self.verticalLayout_6.addWidget(self.textBrowser)
        self.horizontalLayout_2.addLayout(self.verticalLayout_6)
        self.verticalLayout_5 = QtGui.QVBoxLayout()
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.groupBox_2 = QtGui.QGroupBox(self.centralwidget)
        self.groupBox_2.setMaximumSize(QtCore.QSize(300, 16777215))
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.groupBox_2)
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.listWidget_link = QtGui.QListWidget(self.groupBox_2)
        self.listWidget_link.setMaximumSize(QtCore.QSize(300, 16777215))
        self.listWidget_link.setObjectName(_fromUtf8("listWidget_link"))
        self.verticalLayout_3.addWidget(self.listWidget_link)
        self.pushButton_reload = QtGui.QPushButton(self.groupBox_2)
        self.pushButton_reload.setObjectName(_fromUtf8("pushButton_reload"))
        self.verticalLayout_3.addWidget(self.pushButton_reload)
        self.groupBox_4 = QtGui.QGroupBox(self.groupBox_2)
        self.groupBox_4.setObjectName(_fromUtf8("groupBox_4"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout(self.groupBox_4)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.checkBox_R = QtGui.QCheckBox(self.groupBox_4)
        self.checkBox_R.setChecked(True)
        self.checkBox_R.setTristate(False)
        self.checkBox_R.setObjectName(_fromUtf8("checkBox_R"))
        self.verticalLayout_2.addWidget(self.checkBox_R)
        self.checkBox_U = QtGui.QCheckBox(self.groupBox_4)
        self.checkBox_U.setChecked(True)
        self.checkBox_U.setObjectName(_fromUtf8("checkBox_U"))
        self.verticalLayout_2.addWidget(self.checkBox_U)
        self.checkBox_x0 = QtGui.QCheckBox(self.groupBox_4)
        self.checkBox_x0.setChecked(True)
        self.checkBox_x0.setObjectName(_fromUtf8("checkBox_x0"))
        self.verticalLayout_2.addWidget(self.checkBox_x0)
        self.checkBox_x1 = QtGui.QCheckBox(self.groupBox_4)
        self.checkBox_x1.setChecked(True)
        self.checkBox_x1.setObjectName(_fromUtf8("checkBox_x1"))
        self.verticalLayout_2.addWidget(self.checkBox_x1)
        self.pushButton_monitor = QtGui.QPushButton(self.groupBox_4)
        self.pushButton_monitor.setObjectName(_fromUtf8("pushButton_monitor"))
        self.verticalLayout_2.addWidget(self.pushButton_monitor)
        self.horizontalLayout_3.addLayout(self.verticalLayout_2)
        self.verticalLayout_3.addWidget(self.groupBox_4)
        self.groupBox = QtGui.QGroupBox(self.groupBox_2)
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.verticalLayout = QtGui.QVBoxLayout(self.groupBox)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.formLayout = QtGui.QFormLayout()
        self.formLayout.setObjectName(_fromUtf8("formLayout"))
        self.label_C = QtGui.QLabel(self.groupBox)
        self.label_C.setObjectName(_fromUtf8("label_C"))
        self.formLayout.setWidget(0, QtGui.QFormLayout.LabelRole, self.label_C)
        self.label_C_value = QtGui.QLabel(self.groupBox)
        self.label_C_value.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_C_value.setObjectName(_fromUtf8("label_C_value"))
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.label_C_value)
        self.label_A = QtGui.QLabel(self.groupBox)
        self.label_A.setObjectName(_fromUtf8("label_A"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.label_A)
        self.label_A_value = QtGui.QLabel(self.groupBox)
        self.label_A_value.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_A_value.setObjectName(_fromUtf8("label_A_value"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.label_A_value)
        self.label_S = QtGui.QLabel(self.groupBox)
        self.label_S.setObjectName(_fromUtf8("label_S"))
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.label_S)
        self.label_S_value = QtGui.QLabel(self.groupBox)
        self.label_S_value.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_S_value.setObjectName(_fromUtf8("label_S_value"))
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.label_S_value)
        self.label_T = QtGui.QLabel(self.groupBox)
        self.label_T.setObjectName(_fromUtf8("label_T"))
        self.formLayout.setWidget(3, QtGui.QFormLayout.LabelRole, self.label_T)
        self.label_T_value = QtGui.QLabel(self.groupBox)
        self.label_T_value.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_T_value.setObjectName(_fromUtf8("label_T_value"))
        self.formLayout.setWidget(3, QtGui.QFormLayout.FieldRole, self.label_T_value)
        self.label_samples = QtGui.QLabel(self.groupBox)
        self.label_samples.setObjectName(_fromUtf8("label_samples"))
        self.formLayout.setWidget(4, QtGui.QFormLayout.LabelRole, self.label_samples)
        self.label_samples_value = QtGui.QLabel(self.groupBox)
        self.label_samples_value.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_samples_value.setObjectName(_fromUtf8("label_samples_value"))
        self.formLayout.setWidget(4, QtGui.QFormLayout.FieldRole, self.label_samples_value)
        self.verticalLayout.addLayout(self.formLayout)
        self.verticalLayout_3.addWidget(self.groupBox)
        self.verticalLayout_5.addWidget(self.groupBox_2)
        self.horizontalLayout_2.addLayout(self.verticalLayout_5)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 752, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QObject.connect(self.slider_saturation, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.label_percent_value.setNum)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "Distributed Control Systems Monitor", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox_3.setTitle(QtGui.QApplication.translate("MainWindow", "Grafica", None, QtGui.QApplication.UnicodeUTF8))
        self.label_saturation.setText(QtGui.QApplication.translate("MainWindow", "Saturació", None, QtGui.QApplication.UnicodeUTF8))
        self.label_percent_symbol.setText(QtGui.QApplication.translate("MainWindow", "%", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox_2.setTitle(QtGui.QApplication.translate("MainWindow", "Llaços de Control", None, QtGui.QApplication.UnicodeUTF8))
        self.pushButton_reload.setText(QtGui.QApplication.translate("MainWindow", "Actualitzar llista", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox_4.setTitle(QtGui.QApplication.translate("MainWindow", "Opcions", None, QtGui.QApplication.UnicodeUTF8))
        self.checkBox_R.setText(QtGui.QApplication.translate("MainWindow", "Referencia", None, QtGui.QApplication.UnicodeUTF8))
        self.checkBox_U.setText(QtGui.QApplication.translate("MainWindow", "Valor Entrada", None, QtGui.QApplication.UnicodeUTF8))
        self.checkBox_x0.setText(QtGui.QApplication.translate("MainWindow", "Primera Integral", None, QtGui.QApplication.UnicodeUTF8))
        self.checkBox_x1.setText(QtGui.QApplication.translate("MainWindow", "Segona Integral", None, QtGui.QApplication.UnicodeUTF8))
        self.pushButton_monitor.setText(QtGui.QApplication.translate("MainWindow", "Monitoritzar", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox.setTitle(QtGui.QApplication.translate("MainWindow", "Estadístiques", None, QtGui.QApplication.UnicodeUTF8))
        self.label_C.setText(QtGui.QApplication.translate("MainWindow", "Controladors ", None, QtGui.QApplication.UnicodeUTF8))
        self.label_C_value.setText(QtGui.QApplication.translate("MainWindow", "0", None, QtGui.QApplication.UnicodeUTF8))
        self.label_A.setText(QtGui.QApplication.translate("MainWindow", "Actuadors", None, QtGui.QApplication.UnicodeUTF8))
        self.label_A_value.setText(QtGui.QApplication.translate("MainWindow", "0", None, QtGui.QApplication.UnicodeUTF8))
        self.label_S.setText(QtGui.QApplication.translate("MainWindow", "Sensors", None, QtGui.QApplication.UnicodeUTF8))
        self.label_S_value.setText(QtGui.QApplication.translate("MainWindow", "0", None, QtGui.QApplication.UnicodeUTF8))
        self.label_T.setText(QtGui.QApplication.translate("MainWindow", "Dispositius TOTAL", None, QtGui.QApplication.UnicodeUTF8))
        self.label_T_value.setText(QtGui.QApplication.translate("MainWindow", "0", None, QtGui.QApplication.UnicodeUTF8))
        self.label_samples.setText(QtGui.QApplication.translate("MainWindow", "Mostres enllaç", None, QtGui.QApplication.UnicodeUTF8))
        self.label_samples_value.setText(QtGui.QApplication.translate("MainWindow", "0", None, QtGui.QApplication.UnicodeUTF8))

from mplwidget import MplWidget
