#ifndef GETFILEDIALOG_H
#define GETFILEDIALOG_H

#include <QDialog>

namespace Ui {
class GetfileDialog;
}

class GetfileDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GetfileDialog(QWidget *parent = 0);
    ~GetfileDialog();

private Q_SLOTS:
    void on_BackButton_clicked();

    void on_enterButton_clicked();

private:
    Ui::GetfileDialog *ui;
};

#endif // GETFILEDIALOG_H
