#include "../include/main_interface/mykeydialog.h"

QString number[12]={"0","1","2","3","4","5","6","7","8","9","."};
MyKeyDialog::MyKeyDialog() :
    QDialog(0,Qt::Tool | Qt::WindowStaysOnTopHint | Qt::FramelessWindowHint),button_group(new QButtonGroup(this))
{
    setupUi(this);
    initGb ();
}

MyKeyDialog::~MyKeyDialog()
{
    //delete ui;
}

void MyKeyDialog::initGb()
{
    QPushButton *pushButton=new QPushButton(this);          //创建一个按钮对象
    pushButton->hide();                                     //按钮隐藏起来
    pushButton=pushButton_ok;
    for(int i=1;i<14;i++)
    {
        pushButton->setAutoFillBackground(true);
        pushButton->setFocusPolicy(Qt::NoFocus);
        button_vector.push_back(pushButton);      //按钮集合，push_back（）～～ It is equivalent to append(value)，往按钮集合中添加一个按钮
        button_group->addButton(pushButton,i);    //添加一个按钮到button_group中，并且给按钮编号
        pushButton=qobject_cast<QPushButton *>(pushButton->nextInFocusChain());     //焦点指向下一个button
    }
    connect(button_group,SIGNAL(buttonClicked(int)),SLOT(buttonClickResponse(int)));
}
void MyKeyDialog::buttonClickResponse(int gemfield)
{
    if(gemfield==1)
    {
        affirmString ();
        return;
    }
    else if(gemfield==13)
    {
        deleteString ();
        return;
    }
    else
    {
        event=new QKeyEvent(QEvent::KeyPress, 0, Qt::NoModifier,number[gemfield-2]);
    }
     lineEdit_window->setFocus();
     QApplication::sendEvent(focusWidget(),event);
}
void MyKeyDialog::affirmString()
{
    emit sendMessage(lineEdit_window->text());               //发送信号给界面
    this->hide();
}
void MyKeyDialog::deleteString ()
{
    event=new QKeyEvent(QEvent::KeyPress, Qt::Key_Backspace, Qt::NoModifier);       //新建一个键盘事件
    lineEdit_window->setFocus();
    QApplication::sendEvent(focusWidget(),event);
}
