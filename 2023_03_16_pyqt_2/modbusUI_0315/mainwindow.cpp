#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <math.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);

    ui->le01->setText("0");
    ui->le02->setText("0");
    ui->le03->setText("0");
    ui->le04->setText("0");
    ui->le05->setText("0");
    ui->le06->setText("0");
    ui->le07->setText("0");
    ui->le08->setText("0");

    ui->teout->setText("0 0 0 0 0 0 0 0");
    ui->teout_2->setText("0");
    ui->pbclear->setText("Clear");

    QObject::connect(ui->pbclear, SIGNAL(clicked()), this, SLOT(slotClearText()));
    QObject::connect(ui->pbsendoutput, SIGNAL(clicked()), this, SLOT(setTextBinary()));
    QObject::connect(ui->pbsendoutput, SIGNAL(clicked()), this, SLOT(setTextDecimal()));
    QObject::connect(ui->pbsendinput, SIGNAL(clicked()), this, SLOT(updateFrames()));
    QObject::connect(ui->pbclearinput, SIGNAL(clicked()), this, SLOT(slotClearText_input()));

    connect(ui->pb01, SIGNAL(clicked()), this, SLOT(slotSetText01()));
    connect(ui->pb02, SIGNAL(clicked()), this, SLOT(slotSetText02()));
    connect(ui->pb03, SIGNAL(clicked()), this, SLOT(slotSetText03()));
    connect(ui->pb04, SIGNAL(clicked()), this, SLOT(slotSetText04()));
    connect(ui->pb05, SIGNAL(clicked()), this, SLOT(slotSetText05()));
    connect(ui->pb06, SIGNAL(clicked()), this, SLOT(slotSetText06()));
    connect(ui->pb07, SIGNAL(clicked()), this, SLOT(slotSetText07()));
    connect(ui->pb08, SIGNAL(clicked()), this, SLOT(slotSetText08()));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::slotClearText(void)
{
    ui->le01->setText("0");
    ui->le02->setText("0");
    ui->le03->setText("0");
    ui->le04->setText("0");
    ui->le05->setText("0");
    ui->le06->setText("0");
    ui->le07->setText("0");
    ui->le08->setText("0");

    ui->teout->setText("0 0 0 0 0 0 0 0");
    ui->teout_2->setText("0");
}

void MainWindow::slotClearText_input(void)
{
    ui->leinput->setText("");
}


void MainWindow::slotSetText01(void)
{
    ui->le01->setText("1");
}

void MainWindow::slotSetText02(void)
{
    ui->le02->setText("1");
}

void MainWindow::slotSetText03(void)
{
    ui->le03->setText("1");
}

void MainWindow::slotSetText04(void)
{
    ui->le04->setText("1");
}

void MainWindow::slotSetText05(void)
{
    ui->le05->setText("1");
}

void MainWindow::slotSetText06(void)
{
    ui->le06->setText("1");
}

void MainWindow::slotSetText07(void)
{
    ui->le07->setText("1");
}

void MainWindow::slotSetText08(void)
{
    ui->le08->setText("1");
}

void MainWindow::setTextBinary(void)
{
    QString myStr;
    myStr+=ui->le01->text();
    myStr+=" ";
    myStr+=ui->le02->text();
    myStr+=" ";
    myStr+=ui->le03->text();
    myStr+=" ";
    myStr+=ui->le04->text();
    myStr+=" ";
    myStr+=ui->le05->text();
    myStr+=" ";
    myStr+=ui->le06->text();
    myStr+=" ";
    myStr+=ui->le07->text();
    myStr+=" ";
    myStr+=ui->le08->text();



    ui->teout->setText(myStr);
}

void MainWindow::setTextDecimal(void)
{
    QString myStr, myStr2;
    myStr+=ui->le08->text();
    myStr+=ui->le07->text();
    myStr+=ui->le06->text();
    myStr+=ui->le05->text();
    myStr+=ui->le04->text();
    myStr+=ui->le03->text();
    myStr+=ui->le02->text();
    myStr+=ui->le01->text();

    int oac = 0, i = 0, n, strTonum;
    strTonum = myStr.toInt();

        while (strTonum != 0) {
            n = strTonum % 10;
            strTonum /= 10;
            oac += n * pow(2, i);
            ++i;
        }
    myStr2 = myStr2.setNum(oac);

    ui->teout_2->setText(myStr2);
}

void MainWindow::updateFrames(void) {

    // HEX_INPUT의 입력값을 가져옴
    QString hexValue = ui->leinput->text();

    // 입력값을 10자리의 2진수 문자열로 변환
    QString binaryValue = QString("%1").arg(hexValue.toUInt(nullptr, 2), 8, 2, QLatin1Char('0'));

    // 변환된 2진수 문자열에서 각 자리의 값을 배열에 저장
    int values[8];
    for (int i = 0; i < 8; i++) {
    values[i] = binaryValue.mid(i, 1).toInt();
    }


// frame 포인터 배열
QFrame* frames[8] = {ui->LED1, ui->LED2, ui->LED3, ui->LED4,
ui->LED5, ui->LED6, ui->LED7, ui->LED8};

// 배열의 값을 기반으로 frame 색상 설정
for (int i = 0; i < 8; i++) {
if (values[i] == 0) {
frames[i]->setStyleSheet("background-color: red;");
}
else {
frames[i]->setStyleSheet("background-color: blue;");
}
}
}
