#ifndef MY_LINEEDIT_H
#define MY_LINEEDIT_H
#include <QWidget>
#include <QLineEdit>
//#include </home/sun/catkin_ITF/src/main_interface/include/main_interface/my_lineedit.h>

class My_lineEdit : public QLineEdit
{
    Q_OBJECT
public:
    My_lineEdit(QWidget *parent = 0);

protected:
    void mousePressEvent(QMouseEvent *event);

signals:
    void send_show(QString data);

};

#endif // MY_LINEEDIT_H
