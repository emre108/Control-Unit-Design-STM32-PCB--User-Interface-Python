import sys
import datetime
import serial
import serial.tools.list_ports
from PyQt5 import QtWidgets,QtGui,QtCore
 
class serialThreadClass(QtCore.QThread):  #Qthread for receiving data from the serial port
 
    message = QtCore.pyqtSignal(str)
    def __init__(self,parent = None):
 
        super(serialThreadClass,self).__init__(parent)
        self.serialPort = serial.Serial()
        self.stopflag = False
    def stop(self):
        self.stopflag = True
    def run(self):
        while True:
            if (self.stopflag):
                self.stopflag = False
                break
            elif(self.serialPort.isOpen()): 
                    self.data = self.serialPort.readline() 
                     
class Pencere(QtWidgets.QWidget):  # interface class
    def __init__(self):
        super().__init__()
        self.initUi()
 
    def initUi(self): #serial port selection
        self.portComboBox = QtWidgets.QComboBox()
        self.ports = serial.tools.list_ports.comports() 
        for i in self.ports:
            self.portComboBox.addItem(str(i))
        self.baudComboBox = QtWidgets.QComboBox()
        baud = ["9600", "19200", "38400", "57600", "74880", "115200"]
        for i in baud:
            self.baudComboBox.addItem(i)  
        self.baudComboBox.setCurrentText(baud[2]) #selecting 115200 as smt32 defult uart baud-rate
        #all the buttons on the interface
        #connection buttons
        self.baglan = QtWidgets.QPushButton("Connect")
        self.baglantiKes = QtWidgets.QPushButton("Disconnect")
        #dcYuk motors manual control buttons
        self.DCYUK1on = QtWidgets.QPushButton("DCYUK1 ON")
        self.DCYUK1off = QtWidgets.QPushButton("DCYUK1 OFF")
        self.DCYUK2on = QtWidgets.QPushButton("DCYUK2 ON")
        self.DCYUK2off = QtWidgets.QPushButton("DCYUK2 OFF")
        self.DCYUK3on = QtWidgets.QPushButton("DCYUK3 ON")
        self.DCYUK3off = QtWidgets.QPushButton("DCYUK3 OFF")
        self.DCYUK4on = QtWidgets.QPushButton("DCYUK4 ON")
        self.DCYUK4off = QtWidgets.QPushButton("DCYUK4 OFF")
        #led1-2-3-4 manual control buttuns
        self.led1on = QtWidgets.QPushButton("LED1 ON")
        self.led1off = QtWidgets.QPushButton("LED1 OFF")
        self.led2on = QtWidgets.QPushButton("LED2 ON")
        self.led2off = QtWidgets.QPushButton("LED2 OFF")
        self.led3on = QtWidgets.QPushButton("LED3 ON")
        self.led3off = QtWidgets.QPushButton("LED3 OFF")
        self.led4on = QtWidgets.QPushButton("LED4 ON")
        self.led4off = QtWidgets.QPushButton("LED4 OFF")
        
        self.label1 = QtWidgets.QLabel('<font color=red>COM port is not Connected!</font>')#before user connect the board
 
        portVbox = QtWidgets.QVBoxLayout()
        portVbox.addWidget(self.portComboBox) #port list
        portVbox.addWidget(self.baudComboBox) #baud rate list
        portVbox.addWidget(self.baglan)
        portVbox.addWidget(self.baglantiKes)
        portVbox.addWidget(self.label1)
 
        self.portGroup = QtWidgets.QGroupBox("Select a Port")
        self.portGroup.setLayout(portVbox) #after pushing to "select a port" port layout is on screen
        #leds
        buttonHbox1 = QtWidgets.QHBoxLayout()
        buttonHbox1.addWidget(self.led1on)
        buttonHbox1.addWidget(self.led1off)
 
        buttonHbox2 = QtWidgets.QHBoxLayout()
        buttonHbox2.addWidget(self.led2on)
        buttonHbox2.addWidget(self.led2off)
 
        buttonHbox3 = QtWidgets.QHBoxLayout()
        buttonHbox3.addWidget(self.led3on)
        buttonHbox3.addWidget(self.led3off)
 
        buttonHbox4 = QtWidgets.QHBoxLayout()
        buttonHbox4.addWidget(self.led4on)
        buttonHbox4.addWidget(self.led4off)
        
        #dcYuks
        buttonHbox9= QtWidgets.QHBoxLayout()
        buttonHbox9.addWidget(self.DCYUK1on)
        buttonHbox9.addWidget(self.DCYUK1off)
 
        buttonHbox10 = QtWidgets.QHBoxLayout()
        buttonHbox10.addWidget(self.DCYUK2on)
        buttonHbox10.addWidget(self.DCYUK2off)
 
        buttonHbox11 = QtWidgets.QHBoxLayout()
        buttonHbox11.addWidget(self.DCYUK3on)
        buttonHbox11.addWidget(self.DCYUK3off)
 
        buttonHbox12 = QtWidgets.QHBoxLayout()
        buttonHbox12.addWidget(self.DCYUK4on)
        buttonHbox12.addWidget(self.DCYUK4off)
 
        self.message = QtWidgets.QTextEdit()
        self.messageTitle = QtWidgets.QLabel("Receiving Datas")
 
        self.title1 = QtWidgets.QLabel('<font black=blue>Test Software Interface</font>')
        self.title1.setFont(QtGui.QFont("Arial",15,QtGui.QFont.Bold))
        self.title2 = QtWidgets.QLabel('<font black=blue>STM32-Python</font>')
        self.title2.setFont(QtGui.QFont("Arial", 10, QtGui.QFont.Bold))
 
        vBox = QtWidgets.QVBoxLayout()
        vBox.addStretch()
        vBox.addWidget(self.title1)
        vBox.addWidget(self.title2)
        vBox.addWidget(self.portGroup)
        vBox.addLayout(buttonHbox1)
        vBox.addLayout(buttonHbox2)
        vBox.addLayout(buttonHbox3)
        vBox.addLayout(buttonHbox4)
        vBox.addLayout(buttonHbox9)
        vBox.addLayout(buttonHbox10)
        vBox.addLayout(buttonHbox11)
        vBox.addLayout(buttonHbox12)
        vBox.addWidget(self.messageTitle)
        vBox.addWidget(self.message)
        vBox.addStretch()
 
        hBox = QtWidgets.QHBoxLayout()
        hBox.addStretch()
        hBox.addLayout(vBox)
        hBox.addStretch()
 
 
        self.setLayout(hBox)
        self.setWindowTitle("TestSoftwareInterface")
        #therec class in user interface
        self.mySerial = serialThreadClass()  
        self.mySerial.message.connect(self.messageTextEdit) 
        
        self.mySerial.start()    
        #click actions          
        self.baglan.clicked.connect(self.serialConnect) 
        self.baglantiKes.clicked.connect(self.serialDisconnect)
        self.led1on.clicked.connect(lambda: self.leds(self.led1on))   
        self.led1off.clicked.connect(lambda: self.leds(self.led1off)) 
        self.led2on.clicked.connect(lambda: self.leds(self.led2on)) 
        self.led2off.clicked.connect(lambda: self.leds(self.led2off)) 
        self.led3on.clicked.connect(lambda: self.leds(self.led3on))
        self.led3off.clicked.connect(lambda: self.leds(self.led3off))
        self.led4on.clicked.connect(lambda: self.leds(self.led4on))
        self.led4off.clicked.connect(lambda: self.leds(self.led4off))
        
        self.DCYUK1on.clicked.connect(lambda: self.leds(self.DCYUK1on))   
        self.DCYUK1off.clicked.connect(lambda: self.leds(self.DCYUK1off)) 
        self.DCYUK2on.clicked.connect(lambda: self.leds(self.DCYUK2on))  
        self.DCYUK2off.clicked.connect(lambda: self.leds(self.DCYUK2off)) 
        self.DCYUK3on.clicked.connect(lambda: self.leds(self.DCYUK3on))
        self.DCYUK3off.clicked.connect(lambda: self.leds(self.DCYUK3off))
        self.DCYUK4on.clicked.connect(lambda: self.leds(self.DCYUK4on))
        self.DCYUK4off.clicked.connect(lambda: self.leds(self.DCYUK4off))
 
 
        self.show()
    def serialConnect(self): #connecting..
        self.portText = self.portComboBox.currentText() #port selected
        self.port = self.portText.split() 
        self.baudrate = self.baudComboBox.currentText()# baud rate selected
        self.mySerial.serialPort.baudrate = int(self.baudrate) 
        self.mySerial.serialPort.port = self.port[0]
        try:
            self.mySerial.serialPort.open() 
        except:
            self.message.append("Connection ERROR")
        if(self.mySerial.serialPort.isOpen()):
            self.label1.setText('<font color=green>CONNECTED</font>') 
            self.baglan.setEnabled(False)       
            self.portComboBox.setEnabled(False)
            self.baudComboBox.setEnabled(False) 
 
    def serialDisconnect(self): #dissmis the connection
        if self.mySerial.serialPort.isOpen():
            self.mySerial.serialPort.close()
            if self.mySerial.serialPort.isOpen()== False:
                self.label1.setText('<font color=red>Connection is Lost</font>')
                self.baglan.setEnabled(True)
                self.portComboBox.setEnabled(True)
                self.baudComboBox.setEnabled(True)
                
    def messageTextEdit(self):   #receiving datas
        self.incomingMessage = str(self.mySerial.data.decode())
        self.message.append(self.incomingMessage)
        
    def leds(self,led): #Led and dcYuk implementation
        if led == self.led1on:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("1".encode())  
            else:
                self.message.append("Seril Port is not Connected.")
        elif led == self.led1off:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("2".encode())
            else:
                self.message.append("Seril Port is not Connected.")
        elif led == self.led2on:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("3".encode())
            else:
                self.message.append("Seril Port is not Connected.")
        elif led == self.led2off:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("4".encode())
            else:
                self.message.append("Seril Port is not Connected.")
        elif led == self.led3on:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("5".encode())
            else:
                self.message.append("Seril Port is not Connected.")
        elif led == self.led3off:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("6".encode())
            else:
                self.message.append("Seril Port is not Connected.")
        elif led == self.led4on:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("7".encode())
            else:
                self.message.append("Seril Port is not Connected.")
        elif led == self.led4off:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("8".encode())
            else:
                self.message.append("Seril Port is not Connected.")
                
        elif led == self.DCYUK1on:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("A".encode())
            else:
                self.message.append("Seril Port is not Connected..")
        elif led == self.DCYUK1off:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("B".encode())
            else:
                self.message.append("Seril Port is not Connected..")
        elif led == self.DCYUK2on:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("C".encode())
            else:
                self.message.append("Seril Port is not Connected..")
        elif led == self.DCYUK2off:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("D".encode())
            else:
                self.message.append("Seril Port is not Connected..")
        elif led == self.DCYUK3on:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("E".encode())
            else:
                self.message.append("Seril Port is not Connected..")
        elif led == self.DCYUK3off:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("F".encode())
            else:
                self.message.append("Seril Port is not Connected..")
        elif led == self.DCYUK4on:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("G".encode())
            else:
                self.message.append("Seril Port is not Connected..")
        elif led == self.DCYUK4off:
            if self.mySerial.serialPort.isOpen():
                self.mySerial.serialPort.write("H".encode())
            else:
                self.message.append("Seril Port is not Connected..")        
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    pen = Pencere()
    sys.exit(app.exec_())