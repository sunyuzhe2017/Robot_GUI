#include "../include/main_interface/pointroaddialog.h"
#include "../include/main_interface/qnode.hpp"
#include "../include/main_interface/my_lineedit.h"
#include "ui_pointroaddialog.h"

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>

PointroadDialog::PointroadDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PointroadDialog)
{
    ui->setupUi(this);
    setWindowFlags(Qt::FramelessWindowHint);
    numberkeyboard = new MyKeyDialog();
    connect(ui->lineEdit_door,SIGNAL(send_show(QString)),this,SLOT(show_numberkeyboard_ui(QString)));
    connect(ui->lineEdit_x,SIGNAL(send_show(QString)),this,SLOT(show_numberkeyboard_ui(QString)));
    connect(ui->lineEdit_y,SIGNAL(send_show(QString)),this,SLOT(show_numberkeyboard_ui(QString)));
    connect(numberkeyboard,SIGNAL(sendMessage(QString)),this,SLOT(confirmString(QString)));

    // Construct and lay out render panel.
    render_panel_ = new rviz::RenderPanel();
    //layout
    QGridLayout* controls_layout = new QGridLayout();
    controls_layout->addWidget( ui->pButton_back, 0, 3 );
    controls_layout->addWidget( ui->label_door, 0, 0 );
    controls_layout->addWidget( ui->lineEdit_door, 0, 1 );
    controls_layout->addWidget( ui->pButton_OK, 0, 2 );

    QHBoxLayout* pos_layout  = new QHBoxLayout;
    pos_layout->addWidget(ui->label_x);
    pos_layout->addWidget(ui->lineEdit_x);
    pos_layout->addWidget(ui->label_y);
    pos_layout->addWidget(ui->lineEdit_y);
    pos_layout->addWidget(ui->pButton_go);
    QVBoxLayout* main_layout = new QVBoxLayout;
    main_layout->addLayout( controls_layout );
    main_layout->addLayout(pos_layout);
    main_layout->addWidget( render_panel_ );
    setLayout(main_layout);

    //configure the map!
    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->setFixedFrame("map");
    manager_->initialize();
    manager_->startUpdate();

    // Create a Grid display.
    grid_ = manager_->createDisplay( "rviz/Grid", "grid", true );
    ROS_ASSERT( grid_ != NULL );
    //Create a LaserScan display.
    laserscan_ = manager_->createDisplay("rviz/LaserScan","laserscan", true);
    ROS_ASSERT( laserscan_ != NULL );
    //Create a Map display.
    map_ = manager_->createDisplay("rviz/Map","map", true);
    ROS_ASSERT( map_ != NULL );
    localcostmap_ = manager_->createDisplay("rviz/Map","localcostmap", true);
    ROS_ASSERT( localcostmap_ != NULL );
    tf_ = manager_->createDisplay("rviz/TF","tf", true);
    ROS_ASSERT( tf_ != NULL );
    nav_p = manager_->createDisplay("rviz/Path","nav_plan",true);
    ROS_ASSERT( nav_p != NULL );
    foot_p = manager_->createDisplay("rviz/Polygon","foot_print",true);
    ROS_ASSERT( foot_p != NULL );
    posearray_ = manager_->createDisplay("rviz/PoseArray","posearray",true);
    ROS_ASSERT( posearray_ != NULL );

    //Configure the robot PoseArray
    posearray_->subProp("Topic")->setValue("/particlecloud");
    posearray_->subProp("Color")->setValue(QColor(255,25,0));

    //Configure the robot footprint
    foot_p->subProp("Topic")->setValue("/move_base_node/local_costmap/footprint");
    foot_p->subProp("Color")->setValue(QColor(0,0,255));

    //Configure the Navigation Plan
    nav_p->subProp("Topic")->setValue("/move_base_node/NavfnROS/plan");
    nav_p->subProp("Line Style")->setValue("Lines");
    nav_p->subProp("Color")->setValue(QColor(255,0,0));

    // Configure the TFDisplay the way we like it.
    tf_->subProp("Show Axes")->setValue(true);
    tf_->subProp("Show Arrows")->setValue(true);
    tf_->subProp("Show Names")->setValue(true);
    tf_->subProp("Marker Scale")->setValue(5);

    // Configure the MapDisplay the way we like it.
    map_->subProp("Topic")->setValue("/map");
    map_->subProp("Color Scheme")->setValue("map");
    map_->subProp("Draw Behind")->setValue(true);

    //Configure the LocalCostMapDisplay the way we like it.
    localcostmap_->subProp("Topic")->setValue("/move_base_node/local_costmap/costmap");
    localcostmap_->subProp("Color Scheme")->setValue("costmap");
    localcostmap_->subProp("Draw Behind")->setValue(false);

    // Configure the LaserScanDisplay the way we like it.
    laserscan_->subProp("Topic")->setValue("/base_scan");
    laserscan_->subProp("Size (m)")->setValue(0.2);
    laserscan_->subProp("Color")->setValue(QColor(255,255,0));

    // Configure the GridDisplay the way we like it.
    grid_->subProp( "Line Style" )->setValue( "Lines" );
    grid_->subProp("Reference Frame")->setValue("map");
    grid_->subProp("Plane Cell Count")->setValue(50);
    grid_->subProp("Offset")->subProp("X")->setValue(25);
    grid_->subProp("Offset")->subProp("Y")->setValue(25);


}

PointroadDialog::~PointroadDialog()
{
    delete ui;
    delete manager_;
}

void PointroadDialog::on_pButton_back_clicked()
{
    accept();
}
void PointroadDialog::show_numberkeyboard_ui(QString data)
{
  lineEdit_object = sender()->objectName();
  numberkeyboard->lineEdit_window->setText(data);
  //numberkeyboard->resize(400,300);
  numberkeyboard->move(800,300);
  numberkeyboard->exec();
}
void PointroadDialog::confirmString(QString gemfield){
    if(lineEdit_object==ui->lineEdit_door->objectName())
        ui->lineEdit_door->setText(gemfield);
    else if(lineEdit_object==ui->lineEdit_x->objectName())
        ui->lineEdit_x->setText(gemfield);
    else
        ui->lineEdit_y->setText(gemfield);
}
void PointroadDialog::on_pButton_go_clicked()
{
    qnode_ = new main_interface::QNode(0,0);
    double x = ui->lineEdit_x->text().toDouble();
    double y = ui->lineEdit_y->text().toDouble();
    std::cout<<"x:"<<x<<"y:"<<y<<std::endl;
    if(x && y){
    qDebug("x=%f,y=%f",x,y);
    qnode_->send_goal(x,y);}
    else{
     QMessageBox::information(this,tr("Info"),tr("InPut is NULL!"));
    }
}

void PointroadDialog::on_pButton_OK_clicked()
{
    qnode_ = new main_interface::QNode(0,0);
    QString d;
    static double x;
    static double y;
    d = ui->lineEdit_door->text();
    if (d!=NULL){
    int dd = d.toInt();
    switch (dd) {
      case 442: x = 2.0; y = 3.0;
        break;
      case 443: x = 1.0; y = 5.0;
        break;
      case 444: x = 0.5; y = 0.6;
        break;
      default:QMessageBox::information(this,tr("Info"),tr("Please put the right door number(Integer)!"));
        break;
      }
    qnode_->send_goal(x,y);
    qDebug("x=%f,y=%f",x,y);}
    else
    {
     QMessageBox::information(this,tr("Info"),tr("InPut is NULL!"));
    }
}
