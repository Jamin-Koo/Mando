#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <math.h>
#include <iostream>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    pPort=new QSerialPort(this);

    CBGroup = new QButtonGroup(this);
    CBGroup->addButton(ui ->SpeedCB);
    CBGroup->addButton(ui ->PositionCB);

    QObject::connect(ui->openPB,SIGNAL(clicked()),this,SLOT(openPort_2()));
    QObject::connect(ui->stopPB,SIGNAL(clicked()),this,SLOT(closePort()));
    connect(ui->SpeedSendPB,SIGNAL(clicked()),this,SLOT(speedMode()));
    connect(ui->PositionSendPB,SIGNAL(clicked()),this,SLOT(positionMode()));

    connect(ui->SpeedStopPB,SIGNAL(clicked()),this,SLOT(speedStop()));
    connect(ui->PositionStopPB,SIGNAL(clicked()),this,SLOT(positionStop()));


    QObject::connect(CBGroup, QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked), this, &MainWindow::CB_clicked);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::speedMode(void)
{
    // 문자열을 저장할 QString 객체 myStr, myStr2 생성
    QString speedData;
    int Speednum = 0;
    if(ui -> SpeedCB -> isChecked()){
        // QLineEdit 객체에 입력된 문자열을 myStr 변수에 저장
        speedData=ui->SpeedSB->text();

        bool ok;
        Speednum = speedData.toInt(&ok);

        ui->SpeedLE->setText(QString::number(Speednum));

//        // 데이터를 전송하기 위한 DATA 구조체 생성
//        DATA *send = new DATA;
//        // send 구조체의 data_array1 변수에 myStr2 값을 double 형으로 변환하여 저장
//        send->data_array1 = (unsigned char)speedData.toDouble();

//        // send 구조체의 포인터를 char 형으로 캐스팅하여 packet 변수에 저장
//        auto packet = reinterpret_cast<char*>(send);
//        // packet의 크기를 구조체의 크기로 설정하여 데이터 전송
//        pPort->write(packet,sizeof (DATA));
//        // 디버그용 출력문


           send_value.data = Speednum;
           dataArray[0] = selectCB_Button;
           dataArray[1] = send_value.a[0];
           dataArray[2] = send_value.a[1];
           QByteArray sendData;
           QDataStream stream(&sendData, QIODevice::WriteOnly);
           stream.writeRawData(reinterpret_cast<const char*>(dataArray),3 * sizeof(char));
           pPort->write(sendData); // 시리얼 포트를 통해 데이터를 전송합니다.



        qDebug()<<"\n send Data";
        qDebug() <<sendData;
        // send 구조체 삭제
//        delete send;
    }
}

void MainWindow::positionMode(void)
{
    // 문자열을 저장할 QString 객체 myStr, myStr2 생성
    QString positionData = 0;
    int Positionnum = 0;

    if(ui -> PositionCB -> isChecked()){
        // QLineEdit 객체에 입력된 문자열을 myStr 변수에 저장
        positionData=ui->PositionSB->text();

        bool ok;
        Positionnum = positionData.toInt(&ok);

        ui->PositionLE->setText(QString::number(Positionnum));



//        // 데이터를 전송하기 위한 DATA 구조체 생성
//        DATA *send = new DATA;
//        // send 구조체의 data_array1 변수에 myStr2 값을 double 형으로 변환하여 저장
//        send->data_array1 = (unsigned char)positionData.toDouble();

//        // send 구조체의 포인터를 char 형으로 캐스팅하여 packet 변수에 저장
//        auto packet = reinterpret_cast<char*>(send);
//        // packet의 크기를 구조체의 크기로 설정하여 데이터 전송
//        pPort->write(packet,sizeof (DATA));
//        // 디버그용 출력문

        send_value.data = Positionnum;
        dataArray[0] = selectCB_Button;
        dataArray[1] = send_value.a[0];
        dataArray[2] = send_value.a[1];
        QByteArray sendData;
        QDataStream stream(&sendData, QIODevice::WriteOnly);
        stream.writeRawData(reinterpret_cast<const char*>(dataArray),3 * sizeof(char));
        pPort->write(sendData); // 시리얼 포트를 통해 데이터를 전송합니다.

        qDebug()<<"\n send Data";
        qDebug() <<sendData;
        // send 구조체 삭제
//        delete send;
    }
}

void MainWindow::speedStop(void)
{
    int Speednum = 0;
    if(ui -> SpeedCB -> isChecked()){


        ui->SpeedLE->setText(QString::number(Speednum));

        send_value.data = Speednum;
        dataArray[0] = selectCB_Button;
        dataArray[1] = send_value.a[0];
        dataArray[2] = send_value.a[1];
        QByteArray sendData;
        QDataStream stream(&sendData, QIODevice::WriteOnly);
        stream.writeRawData(reinterpret_cast<const char*>(dataArray),3 * sizeof(char));
        pPort->write(sendData); // 시리얼 포트를 통해 데이터를 전송합니다.



        qDebug()<<"\n send Data";
        qDebug() <<sendData;
        // send 구조체 삭제
//        delete send;
    }
}


void MainWindow::positionStop(void)
{
    int Positionnum = 0;
    if(ui -> PositionCB -> isChecked()){


        ui->PositionLE->setText(QString::number(Positionnum));

        send_value.data = Positionnum;
        dataArray[0] = selectCB_Button;
        dataArray[1] = send_value.a[0];
        dataArray[2] = send_value.a[1];
        QByteArray sendData;
        QDataStream stream(&sendData, QIODevice::WriteOnly);
        stream.writeRawData(reinterpret_cast<const char*>(dataArray),3 * sizeof(char));
        pPort->write(sendData); // 시리얼 포트를 통해 데이터를 전송합니다.

        qDebug()<<"\n send Data";
        qDebug() <<sendData;
    }
}


void MainWindow::openPort_2()
{
    QString portName=ui->usbCB->currentText();
    QString portBaud = ui->baudCB->currentText();
    pPort->setPortName(portName);//or ttyACM1, check qDebug Message.

    QSerialPort::BaudRate baudRate = static_cast<QSerialPort::BaudRate>(portBaud.toInt());
    pPort->setBaudRate(baudRate);//changed BaudRate
    //pPort->setBaudRate(QSerialPort::Baud115200);

    pPort->setDataBits(QSerialPort::Data8);
    pPort->setParity(QSerialPort::NoParity);
    if(!(pPort->open(QIODevice::ReadWrite)))
    qDebug()<<"\n Serial Port Open Error";
    else
    qDebug()<<"\n Serial Port Open Success";

}

void MainWindow::CB_clicked(QAbstractButton* button)
{
    if (button == ui->SpeedCB) {
        // radioButton1이 선택된 경우
        selectCB_Button = 1;
        qDebug()<<"\n MODE1";

    } else if (button == ui->PositionCB) {
        // radioButton2가 선택된 경우
        selectCB_Button = 2;
        qDebug()<<"\n MODE2";
    }
}

void MainWindow::closePort()
{
    pPort->close();
    qDebug()<<"\n Serial Port close close";
}


