#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <fstream>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>

class PoseCSVLogger : public rclcpp::Node
{
public:
  PoseCSVLogger()
  : Node("pose_csv_logger")
  {
    count_ = 0;
    orientation_change_threshold = 10;
    distance_threshold = 0.8;

    std::cout<<"nodo avviato"<<std::endl;
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/global", 10, std::bind(&PoseCSVLogger::poseCallback, this, std::placeholders::_1));

    csv_file_.open("/home/marco/4d_thesis/src/saving_pose/poses_data.csv");  //labirinto
    //csv_file_.open("/home/marco/4d_thesis/src/saving_pose/goandturn.csv");   //labirinto test
    //csv_file_.open("/home/marco/4d_thesis/src/saving_pose/data_poses.csv"); // cropfield
    //csv_file_ << "X, Y, qZ, qW" << std::endl; 
  }

private:
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::cout<<"salvo messaggio"<<std::endl;
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

  
    if (count_ == 0)
    {
      last_x = x;
      last_y = y;
      last_qx = qx;
      last_qy = qy;
      last_qz = qz;
      last_qw = qw;
    }
    
    
    double delta_x = x - last_x;
    double delta_y = y - last_y;
    double theta = atan2(2*(qx*qy+qw*qz),1-2*(qy*qy+qz*qz)) * 180/M_PI;
    double last_theta = atan2(2*(last_qx*last_qy+last_qz*last_qw),1-2*(last_qy*last_qy+last_qz*last_qz)) * 180/M_PI;
    double delta_theta = theta-last_theta;

    if (delta_x * delta_x + delta_y * delta_y >= distance_threshold * distance_threshold ||
        delta_theta>= orientation_change_threshold || delta_theta<= -orientation_change_threshold)
      {
        std::ofstream csv_file_("/home/marco/4d_thesis/src/saving_pose/poses_data.csv", std::ios::app);
        if (csv_file_.is_open())                                        
        {
          // Write position x and position y divided by a into the file
          std::cout<<"scrivo nel file"<<std::endl;
          csv_file_ << x << ", " << y << ", " << qx << "," << qy<< "," << qz << " , "<< qw << std::endl;

          // Close the file
          csv_file_.close();

          last_x = x;
          last_y = y;
          last_qx = qx;
          last_qy = qy;
          last_qz = qz;
          last_qw = qw;
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to open position.csv file");
        }
        
       
      }
      count_++;

  
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::ofstream csv_file_;
  double last_x;
  double last_y;
  double last_qx;
  double last_qy;
  double last_qz;
  double last_qw;
  int count_;
  double orientation_change_threshold;
  double distance_threshold;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseCSVLogger>());
  rclcpp::shutdown();
  return 0;
}
