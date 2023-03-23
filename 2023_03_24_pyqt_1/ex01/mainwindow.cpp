#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <math.h>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    pPort=new QSerialPort(this);


    ui->leout01->setText("0");
    ui->leout02->setText("0");
    ui->leout03->setText("0");
    ui->leout04->setText("0");
    ui->leout05->setText("0");
    ui->leout06->setText("0");
    ui->leout07->setText("0");
    ui->leout08->setText("0");

    ui->leBinary->setText("0 0 0 0 0 0 0 0");
    ui->leDecimal->setText("0");
    ui->leHEX->setText("0");
    ui->pbclearOUT->setText("Clear");
    ui->leADC->setText("0");

    QObject::connect(ui->pbclearOUT, SIGNAL(clicked()), this, SLOT(slotClearText()));
    QObject::connect(ui->pbsnedOUT, SIGNAL(clicked()), this, SLOT(setTextBinary()));
    QObject::connect(ui->pbsnedOUT, SIGNAL(clicked()), this, SLOT(setTextDecimal()));
    QObject::connect(ui->pbsendINPUT, SIGNAL(clicked()), this, SLOT(updateFrames()));
    QObject::connect(ui->pbclearINPUT, SIGNAL(clicked()), this, SLOT(slotClearText_input()));
    QObject::connect(ui->pbOPENPORT,SIGNAL(clicked()),this,SLOT(openPort_2()));
    QObject::connect(ui->pbSTOP,SIGNAL(clicked()),this,SLOT(closePort()));


    connect(ui->pbout01, SIGNAL(clicked()), this, SLOT(slotSetText01()));
    connect(ui->pbout02, SIGNAL(clicked()), this, SLOT(slotSetText02()));
    connect(ui->pbout03, SIGNAL(clicked()), this, SLOT(slotSetText03()));
    connect(ui->pbout04, SIGNAL(clicked()), this, SLOT(slotSetText04()));
    connect(ui->pbout05, SIGNAL(clicked()), this, SLOT(slotSetText05()));
    connect(ui->pbout06, SIGNAL(clicked()), this, SLOT(slotSetText06()));
    connect(ui->pbout07, SIGNAL(clicked()), this, SLOT(slotSetText07()));
    connect(ui->pbout08, SIGNAL(clicked()), this, SLOT(slotSetText08()));
    connect(pPort, SIGNAL(readyRead()), this, SLOT(readData())); // readyRead() 시그널과 readData() 슬롯 함수를 연결
}


MainWindow::~MainWindow()
{
    delete ui;
}

unsigned int OUTPUT_buff[16];

void MainWindow::calculate(int data, int count)
{
    for(int i = 0;i < count;i++)
    {
        OUTPUT_buff[i] = data % 2;
        data /= 2;
    }
}

void MainWindow::initOUTPUT_buff(void){

    for(int i=0; i<8;i++){
        OUTPUT_buff[i] = 0;
    }
}

int MainWindow::map(int data, int in_min,int in_max,int out_min,int out_max )
{
    return (data - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void MainWindow::readData() {
    QByteArray data1 = pPort->readAll(); // 수신된 데이터를 읽음
    unsigned char* buffer = reinterpret_cast<unsigned char*>(data1.data());
    int bufferSize = data1.size();
    QString output[bufferSize];
    QString ADC;
    int value[bufferSize];
    int data,data2;
    initOUTPUT_buff();

    for (int i = 0; i < bufferSize; i++) {
        output[i] = QString("%1").arg(buffer[i], 2, 10, QChar('0'));
        value[i] = output[i].toInt();
        qDebug() << value[i];
    }

    qDebug() << "start\n\n";
    qDebug() << value[4];

    if((value[0] == 174) && (value[1] == 184))
        {
            if(value[2] == 105)
            {
                data = value[4];
            }
            else data = 0;
        }


    if(data >= 128)                     calculate(data,8);
    else if(data >= 64 && data < 128)   calculate(data,7);
    else if(data >= 32 && data < 64)    calculate(data,6);
    else if(data >= 16 && data < 32)    calculate(data,5);
    else if(data >= 8  && data < 16)    calculate(data,4);
    else if(data >= 4  && data < 8 )    calculate(data,3);
    else if(data >= 2  && data < 4 )    calculate(data,2);
    else if(data == 1)                  OUTPUT_buff[0] = 1;


    // frame 포인터 배열
    QFrame* frames[8] = {ui->LED01, ui->LED02, ui->LED03, ui->LED04,
    ui->LED05, ui->LED06, ui->LED07, ui->LED08};

    // 배열의 값을 기반으로 frame 색상 설정
    for (int i = 0; i < 8; i++) {

        if (OUTPUT_buff[i] == 0)
        frames[i]->setStyleSheet("background-color: red;");

        else
        frames[i]->setStyleSheet("background-color: blue;");
        }

    for (int i = 0; i < bufferSize; i++) {
        output[i] = QString("%1").arg(buffer[i], 2, 10, QChar('0'));
        value[i] = output[i].toInt();
        qDebug() << value[i];
    }

    qDebug() << "start\n\n";
    qDebug() << value[4];

    if((value[8] == 174) && (value[9] == 184))
        {
            if(value[10] == 105)
            {
               data2 =value[12];
            }
            else data2 = 0;
        }

    data2 = map(data2,0,255,0,4096);
    ADC = QString::number(data2);

    ui->leADC->setText(ADC);


    // 입력값을 10자리의 2진수 문자열로 변환
//    QString binaryValue = QString("%1").arg(decValue.toUInt(), 8, 2, QLatin1Char('0')); //toUInt(nullptr, 10)

//    // 변환된 2진수 문자열에서 각 자리의 값을 배열에 저장
//    int values[8];
//    for (int i = 0; i < 8; i++) {
//    values[i] = binaryValue.mid(i, 1).toInt();
//    }

    //qDebug() << output; // 수신된 데이터를 출력

}


void MainWindow::slotClearText(void)
{
    ui->leout01->setText("0");
    ui->leout02->setText("0");
    ui->leout03->setText("0");
    ui->leout04->setText("0");
    ui->leout05->setText("0");
    ui->leout06->setText("0");
    ui->leout07->setText("0");
    ui->leout08->setText("0");

    ui->leBinary->setText("0 0 0 0 0 0 0 0");
    ui->leDecimal->setText("0");
}

void MainWindow::slotClearText_input(void)
{
    ui->leHEX->setText("0");
}


void MainWindow::slotSetText01(void)
{
    ui->leout01->setText("1");
}

void MainWindow::slotSetText02(void)
{
    ui->leout02->setText("1");
}

void MainWindow::slotSetText03(void)
{
    ui->leout03->setText("1");
}

void MainWindow::slotSetText04(void)
{
    ui->leout04->setText("1");
}

void MainWindow::slotSetText05(void)
{
    ui->leout05->setText("1");
}

void MainWindow::slotSetText06(void)
{
    ui->leout06->setText("1");
}

void MainWindow::slotSetText07(void)
{
    ui->leout07->setText("1");
}

void MainWindow::slotSetText08(void)
{
    ui->leout08->setText("1");
}

void MainWindow::setTextBinary(void)
{
    QString myStr;
    myStr+=ui->leout01->text();
    myStr+=" ";
    myStr+=ui->leout02->text();
    myStr+=" ";
    myStr+=ui->leout03->text();
    myStr+=" ";
    myStr+=ui->leout04->text();
    myStr+=" ";
    myStr+=ui->leout05->text();
    myStr+=" ";
    myStr+=ui->leout06->text();
    myStr+=" ";
    myStr+=ui->leout07->text();
    myStr+=" ";
    myStr+=ui->leout08->text();



    ui->leBinary->setText(myStr);
}

void MainWindow::setTextDecimal(void)
{
    QString myStr, myStr2;
    myStr+=ui->leout08->text();
    myStr+=ui->leout07->text();
    myStr+=ui->leout06->text();
    myStr+=ui->leout05->text();
    myStr+=ui->leout04->text();
    myStr+=ui->leout03->text();
    myStr+=ui->leout02->text();
    myStr+=ui->leout01->text();

    int oac = 0, i = 0, n, strTonum;
    strTonum = myStr.toInt();

        while (strTonum != 0) {
            n = strTonum % 10;
            strTonum /= 10;
            oac += n * pow(2, i);
            ++i;
        }
    myStr2 = myStr2.setNum(oac);

    ui->leDecimal->setText(myStr2);

    DATA *send = new DATA;

        send->data_array1 = (unsigned char)myStr2.toDouble();

        auto packet = reinterpret_cast<char*>(send);
        pPort->write(packet,sizeof (DATA));
        qDebug()<<"\n send Data";

        delete send;
}

void MainWindow::updateFrames(void) {

    // HEX_INPUT의 입력값을 가져옴
    QString decValue = ui->leHEX->text();

    // 입력값을 10자리의 2진수 문자열로 변`환
    QString binaryValue = QString("%1").arg(decValue.toUInt(), 8, 2, QLatin1Char('0')); //toUInt(nullptr, 10)

    // 변환된 2진수 문자열에서 각 자리의 값을 배열에 저장
    int values[8];
    for (int i = 0; i < 8; i++) {
    values[i] = binaryValue.mid(i, 1).toInt();
    }


    // frame 포인터 배열
    QFrame* frames[8] = {ui->LED01, ui->LED02, ui->LED03, ui->LED04,
    ui->LED05, ui->LED06, ui->LED07, ui->LED08};

    // 배열의 값을 기반으로 frame 색상 설정
    for (int i = 0; i < 8; i++) {

        if (values[i] == 0)
        frames[i]->setStyleSheet("background-color: red;");

        else
        frames[i]->setStyleSheet("background-color: blue;");
        }
}



void MainWindow::openPort_2()
{
    QString portName=ui->cbCOM->currentText();
    QString portBaud = ui->CBBAUD->currentText();
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


void MainWindow::closePort()
{
    pPort->close();
    qDebug()<<"\n Serial Port close close";
}
