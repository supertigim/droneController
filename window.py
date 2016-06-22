# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'window.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(400, 279)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.btnWest = QtGui.QPushButton(self.centralwidget)
        self.btnWest.setGeometry(QtCore.QRect(100, 180, 113, 32))
        self.btnWest.setObjectName(_fromUtf8("btnWest"))
        self.btnEast = QtGui.QPushButton(self.centralwidget)
        self.btnEast.setGeometry(QtCore.QRect(260, 180, 113, 32))
        self.btnEast.setObjectName(_fromUtf8("btnEast"))
        self.btnNorth = QtGui.QPushButton(self.centralwidget)
        self.btnNorth.setGeometry(QtCore.QRect(180, 150, 113, 32))
        self.btnNorth.setObjectName(_fromUtf8("btnNorth"))
        self.btnSouth = QtGui.QPushButton(self.centralwidget)
        self.btnSouth.setGeometry(QtCore.QRect(180, 210, 113, 32))
        self.btnSouth.setObjectName(_fromUtf8("btnSouth"))
        self.btnRTL = QtGui.QPushButton(self.centralwidget)
        self.btnRTL.setGeometry(QtCore.QRect(210, 180, 51, 32))
        self.btnRTL.setObjectName(_fromUtf8("btnRTL"))
        self.lblAlt = QtGui.QLabel(self.centralwidget)
        self.lblAlt.setGeometry(QtCore.QRect(54, 100, 71, 16))
        self.lblAlt.setObjectName(_fromUtf8("lblAlt"))
        self.btnUp = QtGui.QPushButton(self.centralwidget)
        self.btnUp.setGeometry(QtCore.QRect(20, 150, 71, 31))
        self.btnUp.setObjectName(_fromUtf8("btnUp"))
        self.btnDown = QtGui.QPushButton(self.centralwidget)
        self.btnDown.setGeometry(QtCore.QRect(20, 170, 71, 32))
        self.btnDown.setObjectName(_fromUtf8("btnDown"))
        self.lblAltValue = QtGui.QLabel(self.centralwidget)
        self.lblAltValue.setGeometry(QtCore.QRect(130, 100, 161, 16))
        self.lblAltValue.setText(_fromUtf8(""))
        self.lblAltValue.setObjectName(_fromUtf8("lblAltValue"))
        self.lblLat = QtGui.QLabel(self.centralwidget)
        self.lblLat.setGeometry(QtCore.QRect(54, 50, 71, 16))
        self.lblLat.setObjectName(_fromUtf8("lblLat"))
        self.lblLon = QtGui.QLabel(self.centralwidget)
        self.lblLon.setGeometry(QtCore.QRect(54, 70, 71, 16))
        self.lblLon.setObjectName(_fromUtf8("lblLon"))
        self.lblLatValue = QtGui.QLabel(self.centralwidget)
        self.lblLatValue.setGeometry(QtCore.QRect(130, 50, 211, 16))
        self.lblLatValue.setText(_fromUtf8(""))
        self.lblLatValue.setObjectName(_fromUtf8("lblLatValue"))
        self.lblLongValue = QtGui.QLabel(self.centralwidget)
        self.lblLongValue.setGeometry(QtCore.QRect(130, 70, 211, 16))
        self.lblLongValue.setText(_fromUtf8(""))
        self.lblLongValue.setObjectName(_fromUtf8("lblLongValue"))
        self.lblFlightMode = QtGui.QLabel(self.centralwidget)
        self.lblFlightMode.setGeometry(QtCore.QRect(84, 10, 91, 16))
        self.lblFlightMode.setObjectName(_fromUtf8("lblFlightMode"))
        self.lblFlightModeValue = QtGui.QLabel(self.centralwidget)
        self.lblFlightModeValue.setGeometry(QtCore.QRect(170, 10, 151, 16))
        self.lblFlightModeValue.setText(_fromUtf8(""))
        self.lblFlightModeValue.setObjectName(_fromUtf8("lblFlightModeValue"))
        self.btnLaunch = QtGui.QPushButton(self.centralwidget)
        self.btnLaunch.setGeometry(QtCore.QRect(20, 206, 71, 32))
        self.btnLaunch.setObjectName(_fromUtf8("btnLaunch"))
        self.frame = QtGui.QFrame(self.centralwidget)
        self.frame.setGeometry(QtCore.QRect(29, 40, 341, 91))
        self.frame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtGui.QFrame.Raised)
        self.frame.setObjectName(_fromUtf8("frame"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 400, 22))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QtCore.QObject.connect(self.btnWest, QtCore.SIGNAL(_fromUtf8("clicked(bool)")), MainWindow.west_click)
        QtCore.QObject.connect(self.btnEast, QtCore.SIGNAL(_fromUtf8("clicked(bool)")), MainWindow.east_click)
        QtCore.QObject.connect(self.btnNorth, QtCore.SIGNAL(_fromUtf8("clicked(bool)")), MainWindow.north_click)
        QtCore.QObject.connect(self.btnSouth, QtCore.SIGNAL(_fromUtf8("clicked(bool)")), MainWindow.south_click)
        QtCore.QObject.connect(self.btnRTL, QtCore.SIGNAL(_fromUtf8("clicked(bool)")), MainWindow.rtl_click)
        QtCore.QObject.connect(self.btnUp, QtCore.SIGNAL(_fromUtf8("clicked(bool)")), MainWindow.up_click)
        QtCore.QObject.connect(self.btnDown, QtCore.SIGNAL(_fromUtf8("clicked(bool)")), MainWindow.down_click)
        QtCore.QObject.connect(self.btnLaunch, QtCore.SIGNAL(_fromUtf8("clicked(bool)")), MainWindow.launch_click)

        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Drone Controller", None))
        self.btnWest.setText(_translate("MainWindow", "Go West", None))
        self.btnEast.setText(_translate("MainWindow", "Go East", None))
        self.btnNorth.setText(_translate("MainWindow", "Go North", None))
        self.btnSouth.setText(_translate("MainWindow", "Go South", None))
        self.btnRTL.setText(_translate("MainWindow", "RTL", None))
        self.lblAlt.setText(_translate("MainWindow", "Altitude    : ", None))
        self.btnUp.setText(_translate("MainWindow", "UP", None))
        self.btnDown.setText(_translate("MainWindow", "DOWN", None))
        self.lblLat.setText(_translate("MainWindow", "Latitude    :", None))
        self.lblLon.setText(_translate("MainWindow", "Longitude :", None))
        self.lblFlightMode.setText(_translate("MainWindow", "Flight Mode :", None))
        self.btnLaunch.setText(_translate("MainWindow", "Launch", None))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

