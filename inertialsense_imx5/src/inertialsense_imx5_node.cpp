#include "inertialsense_imx5.h"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{ 
  rclcpp::init(argc, argv);     
  auto node = std::make_shared<InertialSenseROS>();

  while (rclcpp::ok()) 
  {
    rclcpp::spin_some(node); 
    node->update();
  } 

  rclcpp::shutdown();
  std::cout << "good bye!" << std::endl; 
  return 0; 
}