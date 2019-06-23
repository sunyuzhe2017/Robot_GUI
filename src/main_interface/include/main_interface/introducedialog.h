#ifndef INTRODUCEDIALOG_H
#define INTRODUCEDIALOG_H

#include <QDialog>
class QMediaPlayer;

namespace Ui {
class IntroduceDialog;
}

class IntroduceDialog : public QDialog
{
  Q_OBJECT

public:
  explicit IntroduceDialog(QWidget *parent = 0);
  ~IntroduceDialog();
  void Playvideo(QString filepath);
private Q_SLOTS:
    void on_pushButtonBack_clicked();

    void on_pushButtonRead_clicked();

    void on_tabWidget_currentChanged(int index);

    void on_horizontalSlider_valueChanged(int value);

private:
  Ui::IntroduceDialog *ui;
  QMediaPlayer *player;
};

#endif // INTRODUCEDIALOG_H
