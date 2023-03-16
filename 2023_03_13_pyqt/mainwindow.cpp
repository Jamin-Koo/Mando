#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtCore/QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}
//checkBox_2
void MainWindow::on_pushButton_pressed(void)
{
    if(ui->pushButton->isChecked()){
        qDebug() << "read is checked";
    }

    else {
        qDebug() << "read is not checked";
    }

    if(ui->pushButton_2->isChecked()){
        qDebug() << "write is checked";
    }

    else {
        qDebug() << "write is not checked";
    }
}

