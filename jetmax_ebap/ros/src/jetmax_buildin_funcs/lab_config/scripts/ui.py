# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.setWindowModality(QtCore.Qt.NonModal)
        Form.setEnabled(True)
        Form.resize(950, 620)
        Form.setMinimumSize(QtCore.QSize(0, 0))
        Form.setMaximumSize(QtCore.QSize(950, 620))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(11)
        Form.setFont(font)
        Form.setFocusPolicy(QtCore.Qt.NoFocus)
        Form.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        Form.setAcceptDrops(False)
        Form.setAutoFillBackground(False)
        Form.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.tabWidget = QtWidgets.QTabWidget(Form)
        self.tabWidget.setGeometry(QtCore.QRect(0, 10, 950, 600))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(12)
        self.tabWidget.setFont(font)
        self.tabWidget.setObjectName("tabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.label_8 = QtWidgets.QLabel(self.tab)
        self.label_8.setGeometry(QtCore.QRect(620, 10, 141, 31))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(16)
        self.label_8.setFont(font)
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.widget_12 = QtWidgets.QWidget(self.tab)
        self.widget_12.setGeometry(QtCore.QRect(40, 380, 385, 161))
        self.widget_12.setStyleSheet("#widget_12 {border:1px solid #242424;\n"
"border-color: rgb(200, 200, 200);}")
        self.widget_12.setObjectName("widget_12")
        self.label_25 = QtWidgets.QLabel(self.widget_12)
        self.label_25.setGeometry(QtCore.QRect(5, 30, 15, 31))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(14)
        self.label_25.setFont(font)
        self.label_25.setAlignment(QtCore.Qt.AlignCenter)
        self.label_25.setObjectName("label_25")
        self.label_26 = QtWidgets.QLabel(self.widget_12)
        self.label_26.setGeometry(QtCore.QRect(5, 80, 15, 31))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(14)
        self.label_26.setFont(font)
        self.label_26.setAlignment(QtCore.Qt.AlignCenter)
        self.label_26.setObjectName("label_26")
        self.label_27 = QtWidgets.QLabel(self.widget_12)
        self.label_27.setGeometry(QtCore.QRect(5, 133, 15, 21))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(14)
        self.label_27.setFont(font)
        self.label_27.setAlignment(QtCore.Qt.AlignCenter)
        self.label_27.setObjectName("label_27")
        self.horizontalSlider_LMin = QtWidgets.QSlider(self.widget_12)
        self.horizontalSlider_LMin.setGeometry(QtCore.QRect(20, 35, 169, 22))
        self.horizontalSlider_LMin.setMaximum(255)
        self.horizontalSlider_LMin.setPageStep(1)
        self.horizontalSlider_LMin.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_LMin.setObjectName("horizontalSlider_LMin")
        self.horizontalSlider_AMin = QtWidgets.QSlider(self.widget_12)
        self.horizontalSlider_AMin.setGeometry(QtCore.QRect(20, 85, 169, 22))
        self.horizontalSlider_AMin.setMaximum(255)
        self.horizontalSlider_AMin.setPageStep(1)
        self.horizontalSlider_AMin.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_AMin.setObjectName("horizontalSlider_AMin")
        self.horizontalSlider_BMin = QtWidgets.QSlider(self.widget_12)
        self.horizontalSlider_BMin.setGeometry(QtCore.QRect(20, 135, 169, 22))
        self.horizontalSlider_BMin.setMaximum(255)
        self.horizontalSlider_BMin.setPageStep(1)
        self.horizontalSlider_BMin.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_BMin.setObjectName("horizontalSlider_BMin")
        self.label_LMin = QtWidgets.QLabel(self.widget_12)
        self.label_LMin.setGeometry(QtCore.QRect(80, 10, 51, 20))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(14)
        self.label_LMin.setFont(font)
        self.label_LMin.setAlignment(QtCore.Qt.AlignCenter)
        self.label_LMin.setObjectName("label_LMin")
        self.label_AMin = QtWidgets.QLabel(self.widget_12)
        self.label_AMin.setGeometry(QtCore.QRect(80, 60, 51, 21))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(14)
        self.label_AMin.setFont(font)
        self.label_AMin.setAlignment(QtCore.Qt.AlignCenter)
        self.label_AMin.setObjectName("label_AMin")
        self.label_BMin = QtWidgets.QLabel(self.widget_12)
        self.label_BMin.setGeometry(QtCore.QRect(80, 110, 51, 21))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(14)
        self.label_BMin.setFont(font)
        self.label_BMin.setAlignment(QtCore.Qt.AlignCenter)
        self.label_BMin.setObjectName("label_BMin")
        self.horizontalSlider_LMax = QtWidgets.QSlider(self.widget_12)
        self.horizontalSlider_LMax.setGeometry(QtCore.QRect(210, 35, 169, 22))
        self.horizontalSlider_LMax.setMaximum(255)
        self.horizontalSlider_LMax.setPageStep(1)
        self.horizontalSlider_LMax.setProperty("value", 255)
        self.horizontalSlider_LMax.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_LMax.setObjectName("horizontalSlider_LMax")
        self.horizontalSlider_AMax = QtWidgets.QSlider(self.widget_12)
        self.horizontalSlider_AMax.setGeometry(QtCore.QRect(210, 85, 169, 22))
        self.horizontalSlider_AMax.setMaximum(255)
        self.horizontalSlider_AMax.setPageStep(1)
        self.horizontalSlider_AMax.setProperty("value", 255)
        self.horizontalSlider_AMax.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_AMax.setObjectName("horizontalSlider_AMax")
        self.horizontalSlider_BMax = QtWidgets.QSlider(self.widget_12)
        self.horizontalSlider_BMax.setGeometry(QtCore.QRect(210, 135, 169, 22))
        self.horizontalSlider_BMax.setMaximum(255)
        self.horizontalSlider_BMax.setPageStep(1)
        self.horizontalSlider_BMax.setProperty("value", 255)
        self.horizontalSlider_BMax.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_BMax.setObjectName("horizontalSlider_BMax")
        self.label_LMax = QtWidgets.QLabel(self.widget_12)
        self.label_LMax.setGeometry(QtCore.QRect(270, 10, 51, 20))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(14)
        self.label_LMax.setFont(font)
        self.label_LMax.setAlignment(QtCore.Qt.AlignCenter)
        self.label_LMax.setObjectName("label_LMax")
        self.label_AMax = QtWidgets.QLabel(self.widget_12)
        self.label_AMax.setGeometry(QtCore.QRect(270, 60, 51, 20))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(14)
        self.label_AMax.setFont(font)
        self.label_AMax.setAlignment(QtCore.Qt.AlignCenter)
        self.label_AMax.setObjectName("label_AMax")
        self.label_BMax = QtWidgets.QLabel(self.widget_12)
        self.label_BMax.setGeometry(QtCore.QRect(270, 110, 51, 20))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(14)
        self.label_BMax.setFont(font)
        self.label_BMax.setAlignment(QtCore.Qt.AlignCenter)
        self.label_BMax.setObjectName("label_BMax")
        self.widget_15 = QtWidgets.QWidget(self.tab)
        self.widget_15.setGeometry(QtCore.QRect(450, 380, 85, 161))
        self.widget_15.setStyleSheet("#widget_15 {border:1px solid #242424;\n"
"border-color: rgb(200, 200, 200);}")
        self.widget_15.setObjectName("widget_15")
        self.comboBox_color = QtWidgets.QComboBox(self.widget_15)
        self.comboBox_color.setGeometry(QtCore.QRect(7, 10, 69, 22))
        self.comboBox_color.setObjectName("comboBox_color")
        self.pushButton_labWrite = QtWidgets.QPushButton(self.widget_15)
        self.pushButton_labWrite.setGeometry(QtCore.QRect(5, 83, 75, 30))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(14)
        self.pushButton_labWrite.setFont(font)
        self.pushButton_labWrite.setStyleSheet("QPushButton{background-color: rgb(255, 165, 0);}\n"
"QPushButton:hover{background-color:  rgb(255, 210, 0);}\n"
"QPushButton{border-radius:5px;}")
        self.pushButton_labWrite.setObjectName("pushButton_labWrite")
        self.pushButton_Apply = QtWidgets.QPushButton(self.widget_15)
        self.pushButton_Apply.setGeometry(QtCore.QRect(5, 43, 75, 30))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(14)
        self.pushButton_Apply.setFont(font)
        self.pushButton_Apply.setStyleSheet("QPushButton{background-color: rgb(255, 165, 0);}\n"
"QPushButton:hover{background-color:  rgb(255, 210, 0);}\n"
"QPushButton{border-radius:5px;}")
        self.pushButton_Apply.setObjectName("pushButton_Apply")
        self.pushButton_AddColor = QtWidgets.QPushButton(self.widget_15)
        self.pushButton_AddColor.setGeometry(QtCore.QRect(5, 123, 75, 30))
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(14)
        self.pushButton_AddColor.setFont(font)
        self.pushButton_AddColor.setStyleSheet("QPushButton{background-color: rgb(255, 165, 0);}\n"
"QPushButton:hover{background-color:  rgb(255, 210, 0);}\n"
"QPushButton{border-radius:5px;}")
        self.pushButton_AddColor.setObjectName("pushButton_AddColor")
        self.label_process = QtWidgets.QLabel(self.tab)
        self.label_process.setGeometry(QtCore.QRect(50, 50, 400, 300))
        self.label_process.setText("")
        self.label_process.setObjectName("label_process")
        self.label_orign = QtWidgets.QLabel(self.tab)
        self.label_orign.setGeometry(QtCore.QRect(500, 50, 400, 300))
        self.label_orign.setText("")
        self.label_orign.setObjectName("label_orign")
        self.widget_15.raise_()
        self.widget_12.raise_()
        self.label_8.raise_()
        self.label_process.raise_()
        self.label_orign.raise_()
        self.tabWidget.addTab(self.tab, "")

        self.retranslateUi(Form)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "LAB_Tool 1.0"))
        self.label_8.setText(_translate("Form", "400x300"))
        self.label_25.setText(_translate("Form", "L"))
        self.label_26.setText(_translate("Form", "A"))
        self.label_27.setText(_translate("Form", "B"))
        self.label_LMin.setText(_translate("Form", "0"))
        self.label_AMin.setText(_translate("Form", "0"))
        self.label_BMin.setText(_translate("Form", "0"))
        self.label_LMax.setText(_translate("Form", "255"))
        self.label_AMax.setText(_translate("Form", "255"))
        self.label_BMax.setText(_translate("Form", "255"))
        self.pushButton_labWrite.setText(_translate("Form", "Save"))
        self.pushButton_Apply.setText(_translate("Form", "Apply"))
        self.pushButton_AddColor.setText(_translate("Form", "Add"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("Form", "Camera"))
