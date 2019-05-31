#include "../include/main_interface/getfiledialog.h"
#include "ui_getfiledialog.h"
#include <QMessageBox>
#include <QDebug>
#include <fstream>//added for verification text file
#include <iostream>

GetfileDialog::GetfileDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GetfileDialog)
{
    ui->setupUi(this);
    setWindowFlags(Qt::FramelessWindowHint);
    readVerificationCode();
    serial_servo.setPortName("/dev/Servo");	//设置COM口 改为（"/dev/Servo")
    serial_servo.setBaudRate(QSerialPort::Baud9600,QSerialPort::AllDirections);//设置波特率和读写方向
    serial_servo.setDataBits(QSerialPort::Data8);		//数据位为8位
    serial_servo.setFlowControl(QSerialPort::NoFlowControl);//无流控制
    serial_servo.setParity(QSerialPort::NoParity);	//无校验位
    serial_servo.setStopBits(QSerialPort::OneStop);	//一位停止位

    if(serial_servo.isOpen())//先关串口，再打开，可以保证串口不被其它函数占用。
    {serial_servo.close();}
    if (serial_servo.open(QIODevice::ReadWrite)) {
      QMessageBox::information (this,tr("Info"),tr("Servo Connect successfully!"),QMessageBox::Ok);
    } else {
      QMessageBox::information (this,tr("Info"),tr("Servo Connect Failed!"),QMessageBox::Ok);
    }

    numberkeyboard = new MyKeyDialog();
    connect(ui->enter_lineEdit,SIGNAL(send_show(QString)),this,SLOT(show_numberkeyboard_ui(QString)));
    connect(numberkeyboard,SIGNAL(sendMessage(QString)),this,SLOT(confirmString(QString)));
}

GetfileDialog::~GetfileDialog()
{
    delete ui;
  serial_servo.close();
}

void GetfileDialog::on_BackButton_clicked()
{
    accept();
    serial_servo.close();
}

void GetfileDialog::on_enterButton_clicked()
{
  int line_text = ui->enter_lineEdit->text().toInt();
  if(line_text == v_code){
  //驱动舵机打开储物箱，发送语音：正在打开，请取文件！
    servo_control1();
    int valid = QMessageBox::information (this,tr("Info"),tr("Please get your file,and then push OK!"));
    if(valid == QMessageBox::Ok)
      {
      servo_control2();
      ui->BackButton->setFocus ();
      }
  } else{
      int valid = QMessageBox::information (this,tr("Info"),tr("Verification code is wrong!"),QMessageBox::Ok);
      if(valid == QMessageBox::Ok)
      {
          ui->enter_lineEdit->setFocus ();
      }
    }
    /*if(ui->enter_lineEdit->text () == ""){
        int valid = QMessageBox::information (this,tr("Info"),tr("This can't be empty!"),QMessageBox::Ok);
        if(valid == QMessageBox::Ok)
            ui->enter_lineEdit->setFocus ();
    }*/

}
void GetfileDialog::show_numberkeyboard_ui(QString data)
{
  numberkeyboard->lineEdit_window->setText(data);
  //numberkeyboard->resize(400,300);
  numberkeyboard->move(800,300);
  numberkeyboard->exec();
}
void GetfileDialog::confirmString(QString gemfield)
{
   ui->enter_lineEdit->setText(gemfield);
}
void GetfileDialog::servo_control1()
{
  char nub[] = "0";
  serial_servo.write(nub);
  qDebug()<<"servo opening!"<<endl;
}
void GetfileDialog::servo_control2()
{
  char nub[] = "1";
  serial_servo.write(nub);
  //serial_servo.close();
  qDebug()<<"servo closed!"<<endl;
 // QThread::msleep( 500 );
}
void GetfileDialog::readVerificationCode()
{
  std::ifstream infile;
  infile.open("verificationCode.txt");
  infile>>v_code;
  infile.close();
  qDebug()<<"v_coded is :"<<v_code;
}
