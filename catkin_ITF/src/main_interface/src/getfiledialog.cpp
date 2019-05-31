#include "../include/main_interface/getfiledialog.h"
#include "ui_getfiledialog.h"
#include<QMessageBox>
GetfileDialog::GetfileDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GetfileDialog)
{
    ui->setupUi(this);
}

GetfileDialog::~GetfileDialog()
{
    delete ui;
}

void GetfileDialog::on_BackButton_clicked()
{
    accept();
}

void GetfileDialog::on_enterButton_clicked()
{
    if(ui->enter_lineEdit->text () == ""){
        int valid = QMessageBox::information (this,tr("Info"),tr("This can't be empty!"),QMessageBox::Ok);
        if(valid == QMessageBox::Ok)
            ui->enter_lineEdit->setFocus ();
    }

}
