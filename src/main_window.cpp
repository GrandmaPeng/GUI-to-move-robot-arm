/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/ros_gui/main_window.hpp"
#include <sstream>
#include "../include/ros_gui/qnode.hpp"\
//#include <tf/tf.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_gui {

using namespace Qt;
//using namespace tf;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));

  ui.logInfo->setReadOnly(true);

  ui.refFrame->addItem("panda_hand");
  ui.refFrame->addItem("camera_frame");
  ui.refFrame->addItem("world");
  QComboBox::connect(ui.refFrame, SIGNAL(currentIndexChanged(QString)), this, SLOT(refFrameChanged(QString)));


  ros::init(argc, argv, "Marker_publisher");
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  ros::NodeHandle nh;
  pose_pub = nh.advertise<ros_gui::ARMarker>("ar_pose_marker", 1);
  vision_client = nh.serviceClient<ros_gui::LocalizePart>("localize_part");
  nh.param<std::string>("reference_frame", reference_frame, "panda_hand");
  nh.setParam("base_frame", "world");

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	/*********************
	** Logging
	**********************/

    QPushButton::connect(ui.pubButton, SIGNAL(clicked()), this, SLOT(publishPose()));
    QPushButton::connect(ui.moveButton, SIGNAL(clicked()), this, SLOT(move()));
    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);

}

//    ui.refFrame->setCurrentIndex(0);
   // ros::spin();
    //ros::waitForShutdown();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

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
			ui.line_edit_topic->setReadOnly(true);
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
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */

void MainWindow::refFrameChanged(QString str){

  std::string ref_frame = str.toStdString();
  ros::NodeHandle pnh;
  pnh.setParam("reference_frame", ref_frame);
}


void MainWindow::publishPose(){
  ros::NodeHandle pnh;

  pnh.param<double>("x_pos", pose().position.x, ui.xPosition->text().toFloat());
  pnh.param<double>("y_pos", pose().position.y, ui.yPosition->text().toFloat());
  pnh.param<double>("z_pos", pose().position.z, ui.zPosition->text().toFloat());

  pose().orientation = tf::createQuaternionMsgFromRollPitchYaw(ui.rOrien->text().toFloat(), ui.pOrien->text().toFloat(), ui.yOrien->text().toFloat());

  geometry_msgs::Pose p = pose();
  ros_gui::ARMarker m;
  pnh.getParam("/reference_frame", reference_frame);
  m.header.frame_id = reference_frame;
  m.header.stamp = ros::Time::now();
  m.pose.pose = p;
  pose_pub.publish(m);
  std::stringstream ss;
  ss << "Marker published in the frame: " << m.header.frame_id << "\n" << m.pose.pose;
  ROS_INFO_STREAM("Marker published in the frame: " << m.header.frame_id << "\n" << m.pose.pose);
  ui.logInfo->appendPlainText(QString::fromStdString(ss.str()));
}

void MainWindow::move(){
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::NodeHandle pnh;
  ros_gui::LocalizePart srv;
  pnh.getParam("/base_frame", base_frame);
  srv.request.base_frame = base_frame;
  ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);
  if (!vision_client.call(srv))
  {
    ROS_ERROR("Could not localize part");
    return;
  }
  geometry_msgs::Pose move_target = srv.response.pose;
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  move_group.setPoseReferenceFrame(base_frame);
  move_group.setPoseTarget(move_target);
  move_group.setPlanningTime(2);
  move_group.move();
  ROS_INFO_STREAM("part localized: " << move_target);
  //ros::waitForShutdown();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "ros_gui");
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
    QSettings settings("Qt-Ros Package", "ros_gui");
    settings.setValue("master_url",ui.line_edit_master->text());
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

}  // namespace ros_gui

