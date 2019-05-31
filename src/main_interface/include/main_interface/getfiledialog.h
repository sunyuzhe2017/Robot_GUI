#ifndef GETFILEDIALOG_H
#define GETFILEDIALOG_H

#include <QDialog>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include "my_lineedit.h"
#include "mykeydialog.h"
namespace Ui {
class GetfileDialog;
}

class GetfileDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GetfileDialog(QWidget *parent = 0);
    ~GetfileDialog();
    void servo_control1();
    void servo_control2();
public slots:
    void show_numberkeyboard_ui(QString data);     //显示键盘
    void confirmString(QString gemfield);   //接收键盘发过来的数据
    void readVerificationCode();   //read text的验证码
private Q_SLOTS:
    void on_BackButton_clicked();

    void on_enterButton_clicked();

private:
    Ui::GetfileDialog *ui;
    QSerialPort serial_servo;
    MyKeyDialog *numberkeyboard;
    int v_code;
};

#endif // GETFILEDIALOG_H
