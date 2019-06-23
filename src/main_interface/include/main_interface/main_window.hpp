/**
 * @file /include/main_interface/main_window.hpp
 *
 * @brief Qt based gui for main_interface.
 *
 * @date November 2010
 **/
#ifndef main_interface_MAIN_WINDOW_H
#define main_interface_MAIN_WINDOW_H
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <opencv2/highgui/highgui.hpp>
//for image_view in qt label 2018.6.7
#include <QImage>
#include <QMutex>

//#include <opencv/highgui.h>
namespace main_interface {

/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updateLogcamera();//added
    void displayCamera(const QImage& image);//added

private Q_SLOTS:
    void on_ManualButton_clicked();
    void timerUpdate();
    void shutdown();//shut down computer immediately
    void restart();//restart the computer
    void launch_base();//launch base_controller
    void launch_lidar();//launch rplidar
    void launch_kartoslam();//launch kartoslam
    void launch_amcl();//launch amcl
    void on_SendfileButton_clicked();
    void on_PointButton_clicked();
    void on_GetfileButton_clicked();
    void on_pushButton_camera_clicked();

    void openCamara(); //打开时摄像头
    void readFarme();  //读取当前帧信息
    void closeCamara();//关闭摄像头*/
    //void on_pushButton_shutdown_clicked();

    void on_IntroduceButton_clicked();

private:
	Ui::MainWindowDesign ui;
    QNode qnode;
    QTimer *timer;
    QTimer *timer_c;
    QImage *imag;
    CvCapture *cam;//视频获取结构，用来作为视频获取函数的一个参数
    IplImage *frame;//申请IplImage类型指针，就是申请内存空间来存放每一帧图像

    QSerialPort serial;			//声明串口类

    QImage qimage_;//added
    mutable QMutex qimage_mutex_;//added

protected:
    void mouseDoubleClickEvent (QMouseEvent *event);
    //void to
};

}  // namespace main_interface

#endif // main_interface_MAIN_WINDOW_H
