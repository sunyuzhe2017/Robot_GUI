#ifndef MANUALDIALOG_H
#define MANUALDIALOG_H

#include <QDialog>
#include"ui_manualdialog.h"

namespace main_interface {
  class QNode;
}
namespace Ui {
class ManualDialog;
}
class ManualDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ManualDialog(QWidget *parent = 0);
    ~ManualDialog();

private Q_SLOTS:
    void on_BackButton_clicked();

    void on_pushButton_up_clicked();

    void on_pushButton_left_clicked();

    void on_pushButton_right_clicked();

    void on_pushButton_down_clicked();

    void on_pushButton_clicked();

private:
    Ui::ManualDialog *ui;
    main_interface::QNode* qnode_;
};

#endif // MANUALDIALOG_H
