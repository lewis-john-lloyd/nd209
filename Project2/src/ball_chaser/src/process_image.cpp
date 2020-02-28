#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  ball_chaser::DriveToTarget service;
  service.request.linear_x = lin_x;
  service.request.angular_z = ang_z;
  if (!client.call(service))
    ROS_ERROR("Failed to call service drive_to_target");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
  
  const int white_pixel = 255;
  const int num_channels = img.step/img.width;
  double min_column = img.width;
  double max_column = 0;
  for (int row = 0; row < img.height; ++row) {
    for (int col = 0; col < img.width; ++col) {
      bool white = true;
      for (int channel = 0; channel < num_channels; ++channel){
	if (img.data[row*img.step + col*num_channels + channel] != white_pixel){
	  white = false;
	  break;
	}
      }
      if (not white) continue;
      min_column = std::min(static_cast<double>(col), min_column);
      max_column = std::max(static_cast<double>(col), max_column);
    }
  }
  if ( min_column > max_column) {
    ROS_INFO_STREAM("No Ball");
    drive_robot(0.0, 0.0);
    return;
  }
  ROS_INFO_STREAM("Chasing Ball");
  const double percent_across = ((min_column + max_column)/2.0)/static_cast<double>(img.width);
  if (percent_across < 0.333) {
    drive_robot(0.1, 0.75);
  } else if (percent_across > 0.666 ) {
    drive_robot(0.1, -0.75);
  } else {
    drive_robot(0.5, 0.0);
  }
}

int main(int argc, char** argv)
{
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;
}
