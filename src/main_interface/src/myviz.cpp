#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "../include/main_interface/myviz.h"

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : QWidget( parent )
{
  // Construct and lay out labels and slider controls.
  /*QLabel* thickness_label = new QLabel( "Line Thickness" );
  QSlider* thickness_slider = new QSlider( Qt::Horizontal );
  thickness_slider->setMinimum( 1 );
  thickness_slider->setMaximum( 100 );
  QLabel* cell_size_label = new QLabel( "Cell Size" );
  QSlider* cell_size_slider = new QSlider( Qt::Horizontal );
  cell_size_slider->setMinimum( 1 );
  cell_size_slider->setMaximum( 100 );
  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( thickness_label, 0, 0 );
  controls_layout->addWidget( thickness_slider, 0, 1 );
  controls_layout->addWidget( cell_size_label, 1, 0 );
  controls_layout->addWidget( cell_size_slider, 1, 1 );*/
  QLabel *label_info = new QLabel(tr("Please Touch The Map To Chose Your Detination!"));
  QPushButton *button_quit = new QPushButton(tr("QUIT"));
  button_quit->resize(80,100);
  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( label_info, 0, 0 );
  controls_layout->addWidget( button_quit, 0, 1 );
  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );
  // Make signal/slot connections.
  connect( button_quit, SIGNAL( clicked()), this, SLOT(close()));

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->setFixedFrame("odom");
  manager_->initialize();
  manager_->startUpdate();

  //manager_->addDisplay("rviz/Image",true);
  //img_ = manager_->createDisplay("rviz/Image","image", true);
  //ROS_ASSERT( img_ != NULL );
  //img_->subProp("Image Topic")->setValue("camera/image");

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
  ROS_ASSERT( map_ != NULL );

  // Configure the TFDisplay the way we like it.
  tf_->subProp("Show Axes")->setValue(true);
  tf_->subProp("Show Arrows")->setValue(true);
  tf_->subProp("Show Names")->setValue(true);

  // Configure the MapDisplay the way we like it.
  map_->subProp("Topic")->setValue("/map");
  map_->subProp("Color Scheme")->setValue("map");
  map_->subProp("Draw Behind")->setValue(true);

  // Configure the LaserScanDisplay the way we like it.
  laserscan_->subProp("Topic")->setValue("/scan");

  // Configure the GridDisplay the way we like it.
  grid_->subProp( "Line Style" )->setValue( "Lines" );
  grid_->subProp("Reference Frame")->setValue("odom");

}
// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}
