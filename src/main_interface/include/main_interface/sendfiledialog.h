#ifndef SENDFILEDIALOG_H
#define SENDFILEDIALOG_H

#include <QDialog>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QThread>
#include <QKeyEvent>
#include "my_lineedit.h"
#include "mykeydialog.h"
namespace Ui {
class SendfileDialog;
}

class SendfileDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SendfileDialog(QWidget *parent = 0);
    ~SendfileDialog();
    double random(double,double);
    void servo_control1();
    void servo_control2();
    void writeVerificationCode(int v_code);
    void setTextMode();
    void textContent();
    void closeSerial();
public slots:
    void show_numberkeyboard_ui(QString data);     //显示键盘
    void confirmString(QString gemfield);   //接收键盘发过来的数据
private Q_SLOTS:
    void on_BackButton_clicked();
    void on_enter_address_clicked();
    void on_enter_phone_clicked();
    void on_text_Button_clicked();

private:
    Ui::SendfileDialog *ui;
    QSerialPort serial;			//声明串口类
    QSerialPort serial_servo;
    MyKeyDialog *numberkeyboard;
    //QKeyEvent *event;
    QString lineEdit_object;
    bool boxCoverstate = false;
};

#endif // SENDFILEDIALOG_H
