/**
 * @file /include/ros_gui/main_window.hpp
 *
 * @brief Qt based gui for ros_gui.
 *
 * @date November 2010
 **/
#ifndef ros_gui_MAIN_WINDOW_H
#define ros_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <../../devel/include/ros_gui/ARMarker.h>
#include <../../devel/include/ros_gui/LocalizePart.h>
#include <moveit/move_group_interface/move_group_interface.h>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ros_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
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
        void publishPose();
        void move();
        void refFrameChanged(QString str);
   //     void visionCallback(const ros_gui::ARMarkerConstPtr& msg);
   //     bool localizePart(ros_gui::LocalizePart::Request& req,
  //                     ros_gui::LocalizePart::Response& res);
    /******************************************
    ** Manual connections
    *******************************************/


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
        ros::Publisher pose_pub;
        ros::ServiceClient vision_client;
        static geometry_msgs::Pose& pose()
        {
          static geometry_msgs::Pose pose;
          return pose;
        }

        std::string reference_frame;
        std::string base_frame;



};

}  // namespace ros_gui

#endif // ros_gui_MAIN_WINDOW_H
