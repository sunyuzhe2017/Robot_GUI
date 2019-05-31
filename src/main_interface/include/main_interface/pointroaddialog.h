#ifndef POINTROADDIALOG_H
#define POINTROADDIALOG_H

#include <QDialog>
//#include "my_lineedit.h"
#include "mykeydialog.h"
namespace main_interface {
  class QNode;
}
namespace rviz
{
class My_lineEdit;
class Display;
class RenderPanel;
class VisualizationManager;
}
namespace Ui {
class PointroadDialog;
}

class PointroadDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PointroadDialog(QWidget *parent = 0);
    ~PointroadDialog();
public slots:
    void show_numberkeyboard_ui(QString data);     //显示键盘
    void confirmString(QString gemfield);   //接收键盘发过来的数据
private Q_SLOTS:
    void on_pButton_go_clicked();

    void on_pButton_back_clicked();

    void on_pButton_OK_clicked();

private:
    Ui::PointroadDialog *ui;
    main_interface::QNode* qnode_;
    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* grid_;
    rviz::Display* laserscan_;
    rviz::Display* map_;
    rviz::Display* localcostmap_;
    rviz::Display* tf_;
    rviz::Display* nav_p;
    rviz::Display* foot_p;
    rviz::Display* posearray_;
    MyKeyDialog *numberkeyboard;
    QString lineEdit_object;
};

#endif // POINTROADDIALOG_H
