#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <qserialportinfo.h>
#include <QDebug>

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
      void slotClearText(void);
      void slotSetText01(void);
      void slotSetText02(void);
      void slotSetText03(void);
      void slotSetText04(void);
      void slotSetText05(void);
      void slotSetText06(void);
      void slotSetText07(void);
      void slotSetText08(void);

      void openPort_2(void);
      void closePort(void);
      void setTextBinary(void);
      void setTextDecimal(void);
      void updateFrames(void);
      void slotClearText_input(void);
      void readData(void);
      void initOUTPUT_buff(void);

private:
    Ui::MainWindow *ui;

    QSerialPort *pPort;

    struct DATA{
            unsigned char data_array1;
        };

    union
        {
            uint16_t data;
            uint8_t arr[2];
        }ADC;

};

#endif // MAINWINDOW_H
