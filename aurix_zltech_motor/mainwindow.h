#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
//#include <qserialportinfo.h>
#include <QDebug>
#include <QButtonGroup>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void speedMode(void);
    void positionMode(void);
    void CB_clicked(QAbstractButton* button);

    void speedStop(void);
    void positionStop(void);


    void openPort_2(void);
    void closePort(void);

private:
    Ui::MainWindow *ui;

    QSerialPort *pPort;

    QButtonGroup *CBGroup;
    int selectCB_Button;

    struct DATA{
            unsigned char data_array1;
        };

    char dataArray[3] = {0,};
        union
        {
            uint16_t data;
            uint8_t a[2];
        }send_value;

};


#endif // MAINWINDOW_H
