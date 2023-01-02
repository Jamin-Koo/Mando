import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QLineEdit,  QGridLayout
from PyQt5.QtCore import Qt


class MyApp(QWidget):

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        btn1 = QPushButton('&input P', self)
        btn1.setCheckable(True)

        btn2 = QPushButton('&input PD', self)
        btn2.setCheckable(True)

        btn3 = QPushButton('&input PI', self)
        btn3.setCheckable(True)

        label1 = QLabel('P : ', self)
        label1.setAlignment(Qt.AlignCenter)

        font1 = label1.font()
        font1.setPointSize(15)
        label1.setFont(font1)

        label2 = QLabel('PI : ', self)
        label2.setAlignment(Qt.AlignCenter)
        label2.setFont(font1)

        label3 = QLabel('PD : ', self)
        label3.setAlignment(Qt.AlignCenter)
        label3.setFont(font1)

        qle1 = QLineEdit(self)
        qle2 = QLineEdit(self)
        qle3 = QLineEdit(self)

        grid = QGridLayout()
        self.setLayout(grid)
        grid.addWidget(label1, 0, 0)
        grid.addWidget(label2, 1, 0)
        grid.addWidget(label3, 2, 0)

        grid.addWidget(qle1, 0, 1)
        grid.addWidget(qle2, 1, 1)
        grid.addWidget(qle3, 2, 1)

        grid.addWidget(btn1, 0, 2)
        grid.addWidget(btn2, 1, 2)
        grid.addWidget(btn3, 2, 2)

        self.setWindowTitle('PID Control')
        self.setGeometry(300, 300, 500, 400)
        self.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MyApp()
    sys.exit(app.exec_())