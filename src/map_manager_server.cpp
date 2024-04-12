#include <iostream> // cout, cerr
#include <fstream> // ifstream
#include <sstream> // stringstream

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "map_manager/map_manager.hpp"

using namespace std::chrono_literals;
using namespace std;

using namespace map_manager;

class MapManagerServer : public rclcpp::Node
{
  public:
    MapManagerServer()
    : Node("map_manager_server")
    {    
        //Load static Image -- Terrain Type
        geometry_msgs::msg::Pose origin;
        origin.position.x = -0.495556;
        origin.position.y = -0.495556;
        origin.orientation.w = 1.0;
        float resolution = 0.495556;

        std::string filename = "/home/user/ros/marta_ws/src/map_manager/maps/mars_yard_traversability_areas_borders.pgm";
        std::string frame_id = "map";
        std::string target_topic = "/map_terrain_type";

        map_manager1_.loadImage(filename, resolution, origin, frame_id);
        //map_manager1_.printImage();

        //Define publisher
        publisher1_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(target_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
        timer1_ = this->create_wall_timer(500ms, std::bind(&MapManagerServer::publisher1_callback_, this));


        //Load static Image -- Slope
        filename = "/home/user/ros/marta_ws/src/map_manager/maps/mars_yard_traversability_areas_borders_slope.pgm";
        frame_id = "map";
        target_topic = "/map_slope";

        map_manager2_.loadImage(filename, resolution, origin, frame_id);
        //map_manager2_.printImage();

        publisher2_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(target_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
        timer2_ = this->create_wall_timer(500ms, std::bind(&MapManagerServer::publisher2_callback_, this));
    }

  private:
    void publisher1_callback_()
    {
      auto grid_ = map_manager1_.getOccupancyMessage();
      publisher1_->publish(grid_);
    }

    void publisher2_callback_()
    {
      auto grid_ = map_manager2_.getOccupancyMessage();
      publisher2_->publish(grid_);
    }
    
    MapManager map_manager1_, map_manager2_;
    rclcpp::TimerBase::SharedPtr timer1_, timer2_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher1_, publisher2_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapManagerServer>());
  rclcpp::shutdown();
  return 0;
}
