#include "../include/main_interface/sendfiledialog.h"
#include "ui_sendfiledialog.h"
#include <QDebug>
#include <QMessageBox>

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <string>
#include <stdio.h>
#include <cstdlib>//added for verification code
#include <fstream>//added for verification text file
#include <iostream>

SendfileDialog::SendfileDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SendfileDialog)
{
    ui->setupUi(this);
    setWindowFlags(Qt::FramelessWindowHint);
    //set the serialport
    serial.setPortName("/dev/base_controller");	//设置COM口 改为（"/dev/sim4g")
    serial.setBaudRate(QSerialPort::Baud115200,QSerialPort::AllDirections);//设置波特率和读写方向
    serial.setDataBits(QSerialPort::Data8);		//数据位为8位
    serial.setFlowControl(QSerialPort::NoFlowControl);//无流控制
    serial.setParity(QSerialPort::NoParity);	//无校验位
    serial.setStopBits(QSerialPort::OneStop);	//一位停止位

    if(serial.isOpen())//先关串口，再打开，可以保证串口不被其它函数占用。
    {serial.close();
    qDebug()<<"Original opened!"<<endl;}
    if (serial.open(QIODevice::ReadWrite)) {
      QMessageBox::information (this,tr("Info"),tr("Text Connect successfully!"),QMessageBox::Ok);
    } else {
      QMessageBox::information (this,tr("Info"),tr("Text Connect Failed!"),QMessageBox::Ok);
    }

    serial_servo.setPortName("/dev/servo");	//设置COM口 改为（"/dev/Servo")
    serial_servo.setBaudRate(QSerialPort::Baud9600,QSerialPort::AllDirections);//设置波特率和读写方向
    serial_servo.setDataBits(QSerialPort::Data8);		//数据位为8位
    serial_servo.setFlowControl(QSerialPort::NoFlowControl);//无流控制
    serial_servo.setParity(QSerialPort::NoParity);	//无校验位
    serial_servo.setStopBits(QSerialPort::OneStop);	//一位停止位

    if(serial_servo.isOpen())//先关串口，再打开，可以保证串口不被其它函数占用。
    {serial_servo.close();
    qDebug()<<"Original opened!"<<endl;}
    if (serial_servo.open(QIODevice::ReadWrite)) {
      QMessageBox::information (this,tr("Info"),tr("Servo Connect successfully!"),QMessageBox::Ok);
      QThread::msleep( 500 );// 休息1秒
      servo_control1();
    } else {
      QMessageBox::information (this,tr("Info"),tr("Servo Connect Failed!"),QMessageBox::Ok);
    }

    numberkeyboard = new MyKeyDialog();
    connect(ui->lineEdit_address,SIGNAL(send_show(QString)),this,SLOT(show_numberkeyboard_ui(QString)));
    connect(ui->lineEdit_phone,SIGNAL(send_show(QString)),this,SLOT(show_numberkeyboard_ui(QString)));

    connect(numberkeyboard,SIGNAL(sendMessage(QString)),this,SLOT(confirmString(QString)));
}

SendfileDialog::~SendfileDialog()
{   //qDebug()<<"3"<<endl;
    serial.close();
    serial_servo.close();
    delete ui;
    //qDebug()<<"4"<<endl;

}
void SendfileDialog::show_numberkeyboard_ui(QString data)
{
  lineEdit_object = sender()->objectName();
  numberkeyboard->lineEdit_window->setText(data);
  //numberkeyboard->resize(400,300);
  numberkeyboard->move(800,300);
  numberkeyboard->exec();
}
void SendfileDialog::confirmString(QString gemfield){
    if(lineEdit_object==ui->lineEdit_address->objectName())
        ui->lineEdit_address->setText(gemfield);
    else
        ui->lineEdit_phone->setText(gemfield);
}
void SendfileDialog::on_BackButton_clicked()
{
  serial.close();
  if (boxCoverstate)
    servo_control2();
  qDebug()<<boxCoverstate<<endl;
  //QThread::msleep( 10000 );// 休息1秒
  if(!boxCoverstate){

    qDebug()<<"1"<<endl;
  serial_servo.close();}
  //closeSerial();}
  accept();

}
void SendfileDialog::closeSerial()
{
  qDebug()<<"Close Serial!"<<endl;
  //serial_servo.close();
}
void SendfileDialog::on_enter_address_clicked()
{
    //判断输入地址是否有效
    if(ui->lineEdit_address->text ()!= ""){
    qDebug()<<tr("Address:")<<ui->lineEdit_address->text ();
    //judge: if (the door number is in the map record)
    //yes:   get the door's coordinate,set x,y
    //No:    QMessage door number isn't exsist,Please re-input:
    //       ui->lineEdit_address->setFocus();
      ui->lineEdit_phone->setFocus ();
      on_text_Button_clicked();
    }
    else{
        QMessageBox::information (this,tr("Info"),tr("Address can't be empty!"),QMessageBox::Ok);
            ui->lineEdit_address->setFocus ();
    }
    ui->lineEdit_address->clear ();
}

void SendfileDialog::on_enter_phone_clicked()
{
  //Send phone number to serial
  QString  send_number= ui->lineEdit_phone->text();
  char*  char_number;
  QByteArray buff = send_number.toLatin1();
  char_number=buff.data();

  char  cmgs[256]="at+cmgs=\"";   //AT+CMGS="号码"
  strcat(cmgs,char_number);
  strcat(cmgs,"\"\r");    //cmgs:at+cmgs="号码"
  serial.write(cmgs);//
  qDebug()<<"cmsg :%s\n"<<cmgs<<endl;
  QThread::msleep( 1000 );////休息1秒
  //判断输入手机号是否为空
  if(ui->lineEdit_phone->text ()!= ""){
  qDebug()<<tr("Contact Phone:")<<ui->lineEdit_phone->text ();
  QString  send_number= ui->lineEdit_phone->text();
  char*  char_number;
  QByteArray buff = send_number.toLatin1();
  char_number=buff.data();

  char  cmgs[256]="at+cmgs=\"";   //AT+CMGS="号码"
  strcat(cmgs,char_number);
  strcat(cmgs,"\"\r");    //cmgs:at+cmgs="号码"
  serial.write(cmgs);//
  qDebug()<<"cmsg: "<<cmgs<<endl;
  QThread::msleep( 1000 );
  QMessageBox msg;
  int valid = msg.information(this,tr("Info"),send_number,QMessageBox::Ok);
  //int valid = QMessageBox::information (this,tr("Info"),tr(send_number),QMessageBox::Ok);
  if(valid == QMessageBox::Ok)
  {
    qDebug()<<"GO!";
    servo_control2();
    textContent();
    //此处加入send_goal（x,y）,让小车移动，并返回界面
  }
  ui->enter_phone->setFocus ();
  }
  else{
      int info = QMessageBox::information (this,tr("Info"),tr("Please enter phone!"),QMessageBox::Ok);
      if(info == QMessageBox::Ok)
          ui->lineEdit_phone->setFocus ();
  }

    ui->lineEdit_phone->clear ();
}
///disabled this button!
void SendfileDialog::on_text_Button_clicked()
{ //setTextMode()
  char  cmgf[]="AT+CMGF=1\r";  //设置短信发送模式为text
  serial.write(cmgf);
  QThread::msleep( 1000 );// 休息1秒

  char cscs[]="AT+CSCS=\"";
  strcat(cscs,"GSM\"\r");
  serial.write(cscs);
  QThread::msleep( 1000 );// 休息1秒

  char  csmp[]="AT+CSMP=17,167,0,0\r";
  serial.write(csmp);
  QThread::msleep( 1000 );// 休息1秒
}

void SendfileDialog::servo_control1()
{
  char nub[] = "0";
  serial_servo.write(nub);
  boxCoverstate = true;
  qDebug()<<"servo opening!"<<endl;
}
void SendfileDialog::servo_control2()
{
  char nub[] = "1";
  serial_servo.write(nub);
  boxCoverstate = false;
  qDebug()<<"servo closed!"<<endl;
 // QThread::msleep( 500 );
}
double SendfileDialog::random(double start, double end)
{
  return start+(end-start)*rand()/(RAND_MAX + 1.0);
}
void SendfileDialog::writeVerificationCode(int v_code)
{
  std::ofstream outfile;
  outfile.open("verificationCode.txt");
  if(!outfile)
    qDebug()<<"open error!"<<endl;
  outfile<<v_code;
  outfile.close();
}
void SendfileDialog::setTextMode()
{
  char  cmgf[]="AT+CMGF=1\r";  //设置短信发送模式为text
  serial.write(cmgf);
  QThread::msleep( 1000 );// 休息1秒

  char cscs[]="AT+CSCS=\"";
  strcat(cscs,"GSM\"\r");
  serial.write(cscs);
  QThread::msleep( 1000 );// 休息1秒

  char  csmp[]="AT+CSMP=17,167,0,0\r";
  serial.write(csmp);
  QThread::msleep( 1000 );// 休息1秒
}
void SendfileDialog::textContent()
{
  srand(unsigned(time(0)));
  int code_int = int(random(1000,9999));
  qDebug()<<code_int<<endl;
  QString code = QString::number(code_int);
  qDebug()<<code<<endl;
  writeVerificationCode(code_int);//将验证码wite to text 文件

  QString xinxi= "..........Robot TIANTIAN..........\n Your file is on the way,please get it to your office door late!\n Verification Code:";
  xinxi += code;
  QByteArray send;
  send=xinxi.toLatin1();
  serial.write(send);

  QString sendData = "1A";
  QByteArray sendbuff;
  sendbuff = QByteArray::fromHex(sendData.toLatin1().data());
  serial.write(sendbuff);
}
