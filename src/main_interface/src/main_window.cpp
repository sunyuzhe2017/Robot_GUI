#include <QtGui>
#include <QMessageBox>
#include <QMouseEvent>
#include <QTimer>
#include <QTime>
#include <iostream>
#include "../include/main_interface/main_window.hpp"
#include "../include/main_interface/manualdialog.h"
#include "../include/main_interface/sendfiledialog.h"
#include "../include/main_interface/getfiledialog.h"
#include "../include/main_interface/pointroaddialog.h"
#include "../include/main_interface/myviz.h"
#include <QDebug>

//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/highgui.hpp>//opencv3 changed place
namespace main_interface {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this);
    qRegisterMetaType<QVector<int> >("QVector<int>");
    ui.dock_status->move(1165,496);

    ui.statusbar->close();
    //added statusbar information
    QStatusBar* pStatusBar = ui.statusbar;
    pStatusBar->showMessage("Tianjin University School of Electrical Automation and Information Engineering");
    //  ui.dock_status->geometry().moveBottomRight(QPoint(1165,496));
    //ui.dock_status->geometry().y()= 496;
    ReadSettings();//读取配置
    setWindowTitle(tr("Service Robot"));
    setWindowIcon(QIcon(":/images/icon.png"));
    //ui.tab_manager->setCurrentIndex(0);
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(ui.action_shutdown,SIGNAL(triggered()),this,SLOT(shutdown()));
    QObject::connect(ui.action_restart,SIGNAL(triggered()),this,SLOT(restart()));
    QObject::connect(ui.actionlaunch_base,SIGNAL(triggered()),this,SLOT(launch_base()));
    QObject::connect(ui.actionlaunch_lidar,SIGNAL(triggered()),this,SLOT(launch_lidar()));
    QObject::connect(ui.actionlaunch_kartoslam,SIGNAL(triggered()),this,SLOT(launch_kartoslam()));
    QObject::connect(ui.actionlaunch_amcl,SIGNAL(triggered()),this,SLOT(launch_amcl()));
    /******* Logging *******/

    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
    //LCD时间显示
    QTimer *timer = new QTimer(this);
    QObject::connect(timer,SIGNAL(timeout()),this,SLOT(timerUpdate()));
    timer->start(1000);
    //camera 控制
    timer_c=new QTimer(this);
    QObject::connect(timer_c,SIGNAL(timeout()),this,SLOT(readFarme()));
    QObject::connect(ui.open,SIGNAL(clicked()),this,SLOT(openCamara()));
    QObject::connect(ui.close,SIGNAL(clicked()),this,SLOT(closeCamara()));
    QObject::connect(&qnode,SIGNAL(loggingCamera()),this,SLOT(updateLogcamera()));
}

MainWindow::~MainWindow() {}

void MainWindow::displayCamera(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_ = image.copy();
    //qimage_.setPixel();
    ui.label_camera->setPixmap(QPixmap::fromImage(qimage_));
    //ui.label_camera->resize(ui.label_camera->pixmap()->size());
    ui.label_camera->resize(320,240);
    qimage_mutex_.unlock();
}
void MainWindow::updateLogcamera()
{
    displayCamera(qnode.image);
}
void MainWindow::openCamara()
{
    cam = cvCreateCameraCapture(0);
    //cam=cvCreateCameraCapture(0);
    timer_c->start(33);
    ui.open->setEnabled(false);
    ui.close->setEnabled(true);
    ui.statusbar->show();
    QStatusBar* pStatusBar = ui.statusbar;
    pStatusBar->showMessage("Camera is opening...");
}
void MainWindow::readFarme()
{
    frame = cvQueryFrame(cam);
    QImage imag=QImage((const uchar*)frame->imageData,frame->width,frame->height,QImage::Format_RGB888).rgbSwapped();
    ui.label_camera->setPixmap(QPixmap::fromImage(imag));
    //ui.label_camera->setWindowFlags(Qt::WindowStaysOnBottomHint);
}
void MainWindow::closeCamara()
{
    timer_c->stop();
    cvReleaseCapture(&cam);//释放内存；
    ui.close->setEnabled(false);
    ui.open->setEnabled(true);
    ui.label_camera->clear();
    ui.statusbar->clearMessage();
    ui.statusbar->close();
}
//LCD显示时间
void MainWindow::timerUpdate ()
{
    QDateTime date = QDateTime::currentDateTime ();
    QTime time = QTime::currentTime ();
    QString date1 = date.toString ("yyyy-MM-dd-ddd");
    ui.label_date->setText(date1);
    QString text = time.toString ("hh:mm:ss");
    if ((time.second () % 2) == 0) text[5] = ' ';//每个一秒就将“ ：”显示为空格
    ui.lcdNumber_time->display(text);
    //ui.lcdNumber_time->setWindowFlags(Qt::WindowStaysOnTopHint);
}
//鼠标双击界面全屏
void MainWindow::mouseDoubleClickEvent (QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton){
        if(ui.menubar->isHidden())
            ui.menubar->show();
        else ui.menubar->close();
        if(windowState() != Qt::WindowFullScreen)
            setWindowState(Qt::WindowFullScreen);
        //else setWindowState(Qt::WindowMaximized);
    }
}
void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
		}
	}
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
}

void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>Service Robot Generation:1.0 </h2><p>Copyright QingDao </p><p>School of Electrical and Information Engineering,Tianjin University</p>"));
}

/*** Implementation [Configuration] ***/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "main_interface");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "main_interface");
    settings.setValue("master_url",ui.line_edit_master->text());
    //qDebug()<<"Masterurl:"<<ui.line_edit_master;
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}


void MainWindow::on_ManualButton_clicked()
{
    QStatusBar* pStatusBar = ui.statusbar;
    pStatusBar->showMessage("Manual Control is processing...");
    ui.statusbar->show();
    ManualDialog* dlg_m = new ManualDialog();//Core code for run!
    dlg_m->setWindowTitle("ManualControl");
    dlg_m->exec();
    if(!dlg_m->isActiveWindow())
    {ui.statusbar->clearMessage();
    ui.statusbar->close();}

}
void MainWindow::on_SendfileButton_clicked()
{
    QStatusBar* pStatusBar = ui.statusbar;
    pStatusBar->showMessage("Sendfile Window is processing...");
    ui.statusbar->show();
    QMessageBox::information (this,tr("Info"),tr("Please put your file first!!!"),QMessageBox::Ok);
    SendfileDialog* dlg_s = new SendfileDialog();
    dlg_s->setWindowTitle("SendFile");
    dlg_s->exec();
    if(!dlg_s->isActiveWindow())
    {ui.statusbar->clearMessage();
    ui.statusbar->close();}

}
void MainWindow::on_GetfileButton_clicked()
{
    QStatusBar* pStatusBar = ui.statusbar;
    pStatusBar->showMessage("GetFile Window is processing...");
    ui.statusbar->show();
    GetfileDialog* dlg_g = new GetfileDialog();
    dlg_g->setWindowTitle("GetFile");
    dlg_g->exec();
    if(!dlg_g->isActiveWindow())
    {ui.statusbar->clearMessage();
    ui.statusbar->close();}

}
void MainWindow::on_PointButton_clicked()
{
    /*MyViz* myviz = new MyViz();
    myviz->show();
    myviz->resize(600,400);
    myviz->setWindowTitle("Gmapping Show");*/
    QStatusBar* pStatusBar = ui.statusbar;
    pStatusBar->showMessage("PointRoad Window is processing...");
    ui.statusbar->show();
    PointroadDialog* dlg_p = new PointroadDialog();
    dlg_p->setWindowTitle(tr("PointRoad"));
    dlg_p->exec();
    if(!dlg_p->isActiveWindow())
    {ui.statusbar->clearMessage();
    ui.statusbar->close();}
}
void MainWindow::on_pushButton_camera_clicked()
{
 system("gnome-terminal -x bash -c 'source ~/catkin_ITF/devel/setup.bash;sudo chmod 777 /dev/video0;rosrun usb_cam camera_image.launch'&");
}
void MainWindow::shutdown()
{
    QMessageBox::StandardButton reply_s;
    reply_s = QMessageBox::warning(this,tr("WARNING"),tr("Are you sure to shutdown?"),QMessageBox::Yes | QMessageBox::No,QMessageBox::No                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               );
    if (reply_s == QMessageBox::Yes)
           //ROS_INFO("Shutdown");
    system("gnome-terminal -x bash -c '/home/sun/shutdown.sh'&");
       else
           ROS_INFO("Cancelled!");
}
void MainWindow::restart()
{
    QMessageBox::StandardButton reply_r;
    reply_r = QMessageBox::warning(this,tr("WARNING"),tr("Are you sure to restart?"),QMessageBox::Yes | QMessageBox::No,QMessageBox::No                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               );
    if (reply_r == QMessageBox::Yes)
           //ROS_INFO("Restart");
    system("gnome-terminal -x bash -c '/home/sun/restart.sh'&");
       else
           ROS_INFO("Cancelled!");
}
void MainWindow::launch_base()
{
    system("gnome-terminal -x bash -c 'source ~/catkin_ws/devel/setup.bash;rosrun base_controller base_controller'&");
}
void MainWindow::launch_lidar()
{
    system("gnome-terminal -x bash -c 'source ~/catkin_ws/devel/setup.bash;roslaunch my_robot_nav my_robot_rplidar_laser.launch'&");
}
void MainWindow::launch_kartoslam()
{
    system("gnome-terminal -x bash -c 'source ~/catkin_ws/devel/setup.bash;roslaunch my_robot_nav my_robot_rplidar_laser.launch'&");
}
void MainWindow::launch_amcl()
{
    system("gnome-terminal -x bash -c 'source ~/catkin_ws/devel/setup.bash;roslaunch my_robot_nav my_robot_rplidar_laser.launch'&");
}
}  // namespace main_interface
