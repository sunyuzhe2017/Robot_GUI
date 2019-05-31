#ifndef POINTROADDIALOG_H
#define POINTROADDIALOG_H

#include <QDialog>
namespace main_interface {
  class QNode;
}
namespace rviz
{
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
    rviz::Display* tf_;
    rviz::Display* nav_p;
    rviz::Display* foot_p;
};

#endif // POINTROADDIALOG_H
