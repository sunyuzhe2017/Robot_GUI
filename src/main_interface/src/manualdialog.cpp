#include "../include/main_interface/manualdialog.h"
#include "../include/main_interface/qnode.hpp"
#include <QMessageBox>
#include <QDebug>
//namespace Ui {
//class ManualDialog;
//}

ManualDialog::ManualDialog(QWidget *parent) :
    QDialog(parent),ui(new Ui::ManualDialog)
{
    ui->setupUi(this);
    setWindowFlags(Qt::FramelessWindowHint);
    serial_servo.setPortName("/dev/ttyUSB0");	//设置COM口 改为（"/dev/Servo")
    serial_servo.setBaudRate(QSerialPort::Baud9600,QSerialPort::AllDirections);//设置波特率和读写方向
    serial_servo.setDataBits(QSerialPort::Data8);		//数据位为8位
    serial_servo.setFlowControl(QSerialPort::NoFlowControl);//无流控制
    serial_servo.setParity(QSerialPort::NoParity);	//无校验位
    serial_servo.setStopBits(QSerialPort::OneStop);	//一位停止位
    if(serial_servo.isOpen())//先关串口，再打开，可以保证串口不被其它函数占用。
    {
      qDebug()<<"serial is opening!"<<endl;
      serial_servo.close();}
    if (serial_servo.open(QIODevice::ReadWrite)) {
      QMessageBox::information (this,tr("Info"),tr("Servo Connect successfully!"),QMessageBox::Ok);

    } else {
      QMessageBox::information (this,tr("Info"),tr("Servo Connect Failed!"),QMessageBox::Ok);
    }

}

ManualDialog::~ManualDialog()
{
    delete ui;
  serial_servo.close();
}


void ManualDialog::on_BackButton_clicked()
{
    accept();
    serial_servo.close();
}

void ManualDialog::on_pushButton_up_clicked()
{
    qnode_ = new main_interface::QNode(0,0);
    qnode_->runUp();
}

void ManualDialog::on_pushButton_left_clicked()
{
    qnode_ = new main_interface::QNode(0,0);
    qnode_->runLeft();
}

void ManualDialog::on_pushButton_right_clicked()
{
    qnode_ = new main_interface::QNode(0,0);
    qnode_->runRight();
}

void ManualDialog::on_pushButton_down_clicked()
{
    qnode_ = new main_interface::QNode(0,0);
    qnode_->runDown();
}

void ManualDialog::on_pushButton_clicked()
{
    qnode_ = new main_interface::QNode(0,0);
    qnode_->stop();
}

void ManualDialog::on_pushButton_servoopen_clicked()
{
  char nub[] = "0";
  serial_servo.write(nub);
  qDebug()<<"servo opening!"<<endl;
  ui->pushButton_servoclose->setEnabled(true);
  ui->pushButton_servoopen->setEnabled(false);
}

void ManualDialog::on_pushButton_servoclose_clicked()
{
  char nub[] = "1";
  serial_servo.write(nub);
  qDebug()<<"servo opening!"<<endl;
  ui->pushButton_servoopen->setEnabled(true);
  ui->pushButton_servoclose->setEnabled(false);
}
