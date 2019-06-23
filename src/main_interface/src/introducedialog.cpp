#include "../include/main_interface/introducedialog.h"
#include "ui_introducedialog.h"
#include <QMediaPlayer>
#include <QDebug>

IntroduceDialog::IntroduceDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::IntroduceDialog)
{
  ui->setupUi(this);
  player = new QMediaPlayer;
}

IntroduceDialog::~IntroduceDialog()
{
  delete ui;
}

void IntroduceDialog::on_pushButtonBack_clicked()
{
    //accept();
  close();
  delete player;

}
void IntroduceDialog::Playvideo(QString filepath)
{
    player->setMedia(QUrl::fromLocalFile(filepath));
    player->setVolume (50);
    player->play();
}

void IntroduceDialog::on_pushButtonRead_clicked()
{
    int currentpage = ui->tabWidget->currentIndex();
     QString filepath;
     switch (currentpage) {
     case 0:
       filepath = "/home/sun/AA/OpenIntroduce/001.mp3";
       qDebug("filepath:%d",currentpage);
       Playvideo(filepath);
       break;
     case 1:
       filepath = "/home/sun/AA/OpenIntroduce/002.mp3";
       qDebug("filepath:%d",currentpage);
       Playvideo(filepath);
       break;
     case 2:
       filepath = "/home/sun/AA/OpenIntroduce/003.mp3";
       qDebug("filepath:%d",currentpage);
       Playvideo(filepath);
       break;
     case 3:
       filepath = "/home/sun/AA/OpenIntroduce/004.mp3";
       qDebug("filepath:%d",currentpage);
       Playvideo(filepath);
       break;
     case 4:
       filepath = "/home/sun/AA/OpenIntroduce/005.mp3";
       qDebug("filepath:%d",currentpage);
       Playvideo(filepath);
       break;
     default:
       break;
     }
}


void IntroduceDialog::on_tabWidget_currentChanged(int index)
{
   player->pause();
}

void IntroduceDialog::on_horizontalSlider_valueChanged(int value)
{
    player->setVolume(value);
}
