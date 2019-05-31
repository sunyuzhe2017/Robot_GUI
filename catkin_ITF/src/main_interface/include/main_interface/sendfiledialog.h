#ifndef SENDFILEDIALOG_H
#define SENDFILEDIALOG_H

#include <QDialog>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QThread>
namespace Ui {
class SendfileDialog;
}

class SendfileDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SendfileDialog(QWidget *parent = 0);
    ~SendfileDialog();

private Q_SLOTS:
  //srialport relation convertion
  void QStringtoHex(QByteArray& sendData,QString str);//转十六进制不可更改
  char ConvertHexChar(char ch);

    void on_BackButton_clicked();
    void on_enter_address_clicked();
    void on_enter_phone_clicked();


    void on_go_Button_clicked();

private:
    Ui::SendfileDialog *ui;
    QSerialPort serial;			//声明串口类

};

#endif // SENDFILEDIALOG_H
