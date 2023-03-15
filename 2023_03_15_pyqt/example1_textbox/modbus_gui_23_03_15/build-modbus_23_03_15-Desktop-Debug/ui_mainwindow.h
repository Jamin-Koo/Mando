/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QTextEdit>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *lbName;
    QLineEdit *leName;
    QTextEdit *teMemo;
    QPushButton *pbClean;
    QPushButton *pbInput;
    QToolBar *mainToolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(339, 391);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        lbName = new QLabel(centralWidget);
        lbName->setObjectName(QString::fromUtf8("lbName"));
        lbName->setGeometry(QRect(50, 40, 41, 31));
        leName = new QLineEdit(centralWidget);
        leName->setObjectName(QString::fromUtf8("leName"));
        leName->setGeometry(QRect(80, 40, 191, 31));
        teMemo = new QTextEdit(centralWidget);
        teMemo->setObjectName(QString::fromUtf8("teMemo"));
        teMemo->setGeometry(QRect(50, 100, 231, 181));
        pbClean = new QPushButton(centralWidget);
        pbClean->setObjectName(QString::fromUtf8("pbClean"));
        pbClean->setGeometry(QRect(40, 320, 89, 25));
        pbInput = new QPushButton(centralWidget);
        pbInput->setObjectName(QString::fromUtf8("pbInput"));
        pbInput->setGeometry(QRect(190, 320, 89, 25));
        MainWindow->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        lbName->setText(QApplication::translate("MainWindow", "hex", 0, QApplication::UnicodeUTF8));
        pbClean->setText(QApplication::translate("MainWindow", "Clear", 0, QApplication::UnicodeUTF8));
        pbInput->setText(QApplication::translate("MainWindow", "Input", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
