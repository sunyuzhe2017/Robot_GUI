#include "../include/main_interface/sendfiledialog.h"
#include "ui_sendfiledialog.h"
#include <QDebug>
#include <QMessageBox>

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

SendfileDialog::SendfileDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SendfileDialog)
{
    ui->setupUi(this);
    //set the serialport
    serial.setPortName("/dev/ttyUSB0");	//设置COM口 改为（"/dev/ttyUSB0")
    serial.setBaudRate(QSerialPort::Baud9600,QSerialPort::AllDirections);//设置波特率和读写方向
    serial.setDataBits(QSerialPort::Data8);		//数据位为8位
    serial.setFlowControl(QSerialPort::NoFlowControl);//无流控制
    serial.setParity(QSerialPort::NoParity);	//无校验位
    serial.setStopBits(QSerialPort::OneStop);	//一位停止位
    serial.close();					//先关串口，再打开，可以保证串口不被其它函数占用。
    serial.open(QIODevice::ReadWrite);

      QString moshi="AT+CSCS='GSM'";
      QByteArray send;
      moshi += "\r\n";
      send=moshi.toLatin1();
      serial.write(send);
      QThread::msleep( 1000 );// 休息1秒

      QString wenben= "AT+CMGF=1";
      QByteArray send1;
       wenben += "\r\n";
      send1=wenben.toLatin1();
      serial.write(send1);
      qDebug("here");
      QThread::msleep( 1000 );// 休息1秒
}

SendfileDialog::~SendfileDialog()
{
    delete ui;
}

void SendfileDialog::on_BackButton_clicked()
{
    accept();
}

void SendfileDialog::on_enter_address_clicked()
{
    //判断输入地址是否有效
    if(ui->lineEdit_address->text ()!= ""){
    qDebug()<<tr("Address:")<<ui->lineEdit_address->text ();
      ui->lineEdit_phone->setFocus ();
      //ui->enter_phone->setFocus ();
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
    //QString sendData= ui->lineEdit_phone->text().toLatin1();//->toPlainText().toLatin1();//以ASCII码的形式通过串口发送出去。
    QString sendData= ui->lineEdit_phone->text();
    sendData = "AT+CMGS='"+sendData+"'";
    qDebug()<<"sendData is :"<<sendData<<endl;
    sendData.toLatin1();
    QByteArray send;
    sendData += "\r\n";
    qDebug()<<"sendData is :"<<sendData<<endl;
    send=sendData.toLatin1();
    serial.write(send);//

    //判断输入手机号是否为空
    if(ui->lineEdit_phone->text ()!= ""){
    qDebug()<<tr("Contact Phone:")<<ui->lineEdit_phone->text ();
    ui->enter_phone->setFocus ();
    }
    else{
        int info = QMessageBox::information (this,tr("Info"),tr("Please enter phone!"),QMessageBox::Ok);
        if(info == QMessageBox::Ok)
            ui->lineEdit_phone->setFocus ();
    }
    ui->lineEdit_phone->clear ();
}

void SendfileDialog::on_go_Button_clicked()
{
    QString xinxi= "ROBOT";
    QByteArray send;
    send=xinxi.toLatin1();
    serial.write(send);

    QString sendData = "1A";
    QByteArray sendbuff;
    sendbuff = QByteArray::fromHex(sendData.toLatin1().data());
    serial.write(sendbuff);
}
void SendfileDialog::QStringtoHex(QByteArray &sendData, QString str)//转十六进制不可更改
{
    char hstr,lstr,hdata,ldata;
    int len = str.length();
    int sendnum = 0;
    QByteArray temp;
    temp.resize(len/2);//设置大小，len/2会大于实际16进制字符
    //sendData.resize(len/2);
    for(int i=0;i<len;)
    {
        //hstr = str[i].toAscii();
        hstr = str[i].toLatin1();
        if(hstr == ' ')
        {
            ++i;
            continue;
        }
        ++i;
        if(i >= len)
        {
            break;
        }
        lstr = str[i].toLatin1();

        hdata = ConvertHexChar(hstr);
        ldata = ConvertHexChar(lstr);
        if(-1 == hdata || -1 == ldata)
        {
            break;
        }
        ++i;
        temp[sendnum] = hdata<<4|ldata;
        sendnum++;
    }
    sendData.reserve(sendnum);
    sendData = temp.left(sendnum);//去掉多余字符
}
char SendfileDialog::ConvertHexChar(char ch)
{
    if(ch>='a'&&ch<='f')
    {
        return ch-'a'+10;
    }
    else if(ch>='A'&&ch<='F')
    {
        return ch-'A'+10;
    }
    else if(ch>='0'&&ch<='9')
    {
        return ch-'0';
    }
    else{
        return -1;
    }
}
