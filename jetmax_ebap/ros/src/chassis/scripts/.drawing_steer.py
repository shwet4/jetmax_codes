#!/usr/bin/env python3
import sys
import time
import math
import signal
import hiwonder
import threading
from PyQt5.QtCore import Qt, QPoint, QRect
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPainter, QPixmap, QColor,QPen,QFont

# 画线行驶

# 实例化底盘驱动库
chassis = hiwonder.MecanumChassis()

# 角度转换函数
def azimuthAngle(p1, p2):
    x1 = p1[0]
    y1 = -p1[1]
    x2 = p2[0]
    y2 = -p2[1]

    angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
    if angle < 0:
        angle = 360 + angle
   
    return angle

class Winform(QWidget):
    def __init__(self, parent=None):
        super(Winform, self).__init__(parent)
        self.setWindowTitle("绘图画板")
        self.pix = QPixmap()  # 实例化一个 QPixmap 对象
        self.lastPoint = QPoint()  # 起始点
        self.endPoint = QPoint()  # 终点
        self.initUi()
        self.pp = QPainter(self.pix)
        self.runner = None
        self.num = 1
        self.st = True
        self.coords = []
    
    def initUi(self):
        # 窗口大小设置为600*500
        self.resize(800, 600)
        # 画布大小为400*400，背景为白色
        self.pix = QPixmap(800, 600)
        self.pix.fill(Qt.white)

    # 重绘的复写函数 主要在这里绘制
    def paintEvent(self, event):
        # 根据鼠标指针前后两个位置绘制直线
        self.pp.setPen(QPen(QColor(0, 0, 0),3))
        self.pp.drawLine(self.lastPoint, self.endPoint)
        # 让前一个坐标值等于后一个坐标值，
        # 这样就能实现画出连续的线
        self.lastPoint = self.endPoint
        painter = QPainter(self)
        painter.drawPixmap(0, 0, self.pix)  # 在画布上画出

    # 鼠标按压事件
    def mousePressEvent(self, event):
        # 鼠标左键按下
        if event.button() == Qt.LeftButton:
            if self.st:
                self.lastPoint = event.pos()
                self.endPoint = self.lastPoint
                self.st = False
            else:
                self.endPoint = event.pos()
                # 进行重新绘制
                self.update()
            x, y = (event.pos().x(), event.pos().y())
            self.coords.append((x,y))
            self.pp.setPen(QPen(QColor(255, 0, 0),3))  
            self.pp.drawEllipse(x-7, y-7, 14, 14)
            self.pp.drawText(x,y-12,str(self.num))
            self.num += 1
            self.update()

        # 鼠标右键按下
        elif event.button() == Qt.RightButton:
            if self.runner is None:
                if len(self.coords) > 0:
                    self.runner = threading.Thread(target=move, daemon=True)
                    self.runner.start()

# 移动控制函数
def move():
    #print(form.coords)
    form.pp.setPen(QPen(QColor(0, 255, 0),3)) 
    form.pp.drawEllipse(form.coords[0][0]-7, form.coords[0][1]-7, 14, 14)
    form.update() #更新显示
    for i in range(len(form.coords)-1): #逐一运行所有线段
        x1 = form.coords[i][0]
        x2 = form.coords[i+1][0]
        y1 = form.coords[i][1]
        y2 = form.coords[i+1][1]
        dx = x2 - x1
        dy = y2 - y1
        D = int(math.sqrt((dx**2)+(dy**2)))
        angle = int(azimuthAngle(form.coords[i], form.coords[i+1])) #计算偏转角
        print('D:',D, 'angle:',angle)
        
        form.pp.setPen(QPen(QColor(0, 0, 255),3))  #设置正在运行的线段为蓝色
        form.pp.drawLine(x1,y1, x2,y2)
        form.update() #更新显示
        chassis.set_velocity(100,angle,0) #驱动底盘电机
        t = D / 80   
        time.sleep(t)
        chassis.set_velocity(0,0,0) # 停止移动
        time.sleep(0.02)

        form.pp.setPen(QPen(QColor(0, 255, 0),3)) #设置运行过了的线段为绿色
        form.pp.drawLine(x1,y1, x2,y2)
        form.pp.drawEllipse(x2-7, y2-7, 14, 14)
        form.update() #更新显示

    form.pp.setPen(QPen(QColor(0, 255, 0),3))
    form.pp.drawEllipse(form.coords[-1][0]-7, form.coords[-1][1]-7, 14, 14)
    form.update()
    print('Finish')
    time.sleep(0.8)
    form.runner = None
    form.coords.clear()
    QPainter.eraseRect(form.pp, QRect(0, 0, 800, 600)) #清除内容
    form.update()
    form.num = 1
    form.st = True


# 停止运行函数
def shutdown(signum, frame):
    print('shutdown')
    chassis.set_velocity(0,0,0)
    sys.exit()    

signal.signal(signal.SIGINT, shutdown)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    form = Winform()
    form.show()
    sys.exit(app.exec_())
    

