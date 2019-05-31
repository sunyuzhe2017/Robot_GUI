#include "../include/main_interface/manualdialog.h"
#include "../include/main_interface/qnode.hpp"

//namespace Ui {
//class ManualDialog;
//}

ManualDialog::ManualDialog(QWidget *parent) :
    QDialog(parent),ui(new Ui::ManualDialog)
{
    ui->setupUi(this);

}

ManualDialog::~ManualDialog()
{
    delete ui;
}


void ManualDialog::on_BackButton_clicked()
{
    accept();
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
