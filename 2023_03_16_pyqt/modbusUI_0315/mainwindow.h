#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
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

      void setTextBinary(void);
      void setTextDecimal(void);
      void updateFrames(void);


private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
