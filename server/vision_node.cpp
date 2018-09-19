#include <ros/ros.h>
#include <ros_gui/ARMarker.h>
#include <tf/transform_listener.h>
#include <ros_gui/LocalizePart.h>

class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
      ar_sub_ = nh.subscribe<ros_gui::ARMarker>("ar_pose_marker",
 1, &Localizer::visionCallback, this);
      server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
  }

  void visionCallback(const ros_gui::ARMarkerConstPtr& msg)
  {
      last_msg_ = msg;
      //ROS_INFO_STREAM(last_msg_->pose.pose);
  }

  bool localizePart(ros_gui::LocalizePart::Request& req,
                    ros_gui::LocalizePart::Response& res){
  // Read last message
  ros_gui::ARMarkerConstPtr p = last_msg_;

  if (!p) return false;

  res.pose = p->pose.pose;

  tf::Transform cam_to_target;
  tf::poseMsgToTF(p->pose.pose, cam_to_target);
  tf::StampedTransform req_to_cam;
  listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);
  tf::Transform req_to_target;
  req_to_target = req_to_cam * cam_to_target;
  tf::poseTFToMsg(req_to_target, res.pose);

  return true;
  }


  ros::Subscriber ar_sub_;
  ros::ServiceServer server_;
  ros_gui::ARMarkerConstPtr last_msg_;
  tf::TransformListener listener_;
};

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "vision_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  // The Localizer class provides this node's ROS interfaces
  Localizer localizer(nh);

  ROS_INFO("Vision node starting");
  // Note: don't forget to leave ros::spin(); in place.

  // Don't exit the program.
  ros::spin();
}

