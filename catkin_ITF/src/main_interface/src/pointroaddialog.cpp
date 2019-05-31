#include "../include/main_interface/pointroaddialog.h"
#include "../include/main_interface/qnode.hpp"
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
    tf_ = manager_->createDisplay("rviz/TF","tf", true);
    ROS_ASSERT( tf_ != NULL );
    nav_p = manager_->createDisplay("rviz/Path","nav_plan",true);
    ROS_ASSERT( nav_p != NULL );
    foot_p = manager_->createDisplay("rviz/Polygon","foot_print",true);
    ROS_ASSERT( foot_p != NULL );

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

    // Configure the MapDisplay the way we like it.
    map_->subProp("Topic")->setValue("/map");
    map_->subProp("Color Scheme")->setValue("map");
    map_->subProp("Draw Behind")->setValue(true);

    // Configure the LaserScanDisplay the way we like it.
    laserscan_->subProp("Topic")->setValue("/base_scan");
    laserscan_->subProp("Size (m)")->setValue(0.1);
    laserscan_->subProp("Color")->setValue(QColor(255,255,0));

    // Configure the GridDisplay the way we like it.
    grid_->subProp( "Line Style" )->setValue( "Lines" );
    grid_->subProp("Reference Frame")->setValue("odom");
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

void PointroadDialog::on_pButton_go_clicked()
{
    qnode_ = new main_interface::QNode(0,0);
    double x = ui->lineEdit_x->text().toDouble();
    double y = ui->lineEdit_y->text().toDouble();
    qDebug("x=%f,y=%f",x,y);
    //qnode_->send_goal(x,y);
}

void PointroadDialog::on_pButton_OK_clicked()
{
    QString d;
    static double x=0;
    static double y=0;
    d = ui->lineEdit_door->text();
    int dd = d.toInt();
    switch (dd) {
      case 442: x = 1; y = 1;
        break;
    case 443: x = 5; y = 5;
      break;
    case 444: x = 0; y = 0;
      break;
      default:QMessageBox::information(this,tr("Info"),tr("Please put the right door number(Integer)!"));
        break;
      }
    //qnode_->send_goal(x,y);
    qDebug("x=%f,y=%f",x,y);
}
