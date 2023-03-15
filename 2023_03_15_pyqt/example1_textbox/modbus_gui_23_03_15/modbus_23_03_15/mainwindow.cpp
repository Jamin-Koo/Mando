#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->leName->setText("00 00 00 00 00 00 00 00");
    ui->pbClean->setText("Clear");

    QObject::connect(ui->pbClean, SIGNAL(clicked()), this, SLOT(slotClearText()));
    connect(ui->pbInput, SIGNAL(clicked()), this, SLOT(slotSetText()));

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::slotClearText(void)
{
      ui->teMemo->clear();
}

void MainWindow::slotSetText(void)
{
    QString myStr;
    myStr=ui->leName->text();
    myStr+="\n input hex code";
    ui->teMemo->setText(myStr);
}
