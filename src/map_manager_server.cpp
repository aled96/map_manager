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
#include "map_manager_srvs/srv/terrain_slope_update.hpp"
#include "map_manager_srvs/srv/terrain_type_update.hpp"

using namespace std::chrono_literals;
using namespace std;

using namespace map_manager;

class MapManagerServer : public rclcpp::Node
{
  public:
    MapManagerServer()
    : Node("map_manager_server")
    {    

        //TO ACQUIRE
        //Load Global Intial Static Image -- Terrain Type
        geometry_msgs::msg::Pose origin;
        origin.position.x = -0.495556;
        origin.position.y = -0.495556;
        origin.orientation.w = 1.0;
        float resolution = 0.495556;

        std::string filename = "/home/user/ros/marta_ws/src/map_manager/maps/mars_yard_traversability_areas_borders.pgm";
        std::string frame_id = "map";
        std::string target_topic = "/global_map_terrain_type";

        map_terrain_manager_.loadImage(filename, resolution, origin, frame_id);
        global_terrain_grid_ = map_terrain_manager_.getOccupancyMessage();

        //Define publisher
        publisher_global_terrain_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(target_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
        timer1_ = this->create_wall_timer(500ms, std::bind(&MapManagerServer::publisher_global_terrain_callback_, this));


        //Load Global Intial Static Image -- Slope Value
        filename = "/home/user/ros/marta_ws/src/map_manager/maps/mars_yard_traversability_areas_borders_slope.pgm";
        frame_id = "map";
        target_topic = "/global_map_slope";

        map_slope_manager_.loadImage(filename, resolution, origin, frame_id);
        global_slope_grid_ = map_slope_manager_.getOccupancyMessage();
        //map_slope_manager_.printImage();

        publisher_global_slope_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(target_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
        timer2_ = this->create_wall_timer(500ms, std::bind(&MapManagerServer::publisher_global_slope_callback_, this));

        //Define services to update the (global) map information
        update_slope_srv_ = this->create_service<map_manager_srvs::srv::TerrainSlopeUpdate>("/update_global_map_slope", std::bind(&MapManagerServer::update_slope_cb, this, std::placeholders::_1, std::placeholders::_2));
        update_type_srv_ = this->create_service<map_manager_srvs::srv::TerrainTypeUpdate>("/update_global_map_type", std::bind(&MapManagerServer::update_type_cb, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:
    void publisher_global_terrain_callback_()
    {
      publisher_global_terrain_->publish(global_terrain_grid_);
    }

    void publisher_global_slope_callback_()
    {
      publisher_global_slope_->publish(global_slope_grid_);
    }
  
    void update_slope_cb(const std::shared_ptr<map_manager_srvs::srv::TerrainSlopeUpdate::Request> request,
                         std::shared_ptr<map_manager_srvs::srv::TerrainSlopeUpdate::Response>      response)
    {
      float radius = request->inflation_radius[0];
      int limit = (int)(radius/global_slope_grid_.info.resolution);
      int max_radius = limit*limit;
      int index_base, index;

      //TODO put it into MapManager Class ?
      for(int i = 0; i < (int)(request->points.size()); i++){
        if(request->inflation_radius.size() > 1){
          radius = request->inflation_radius[i];
          limit = (int)(radius/global_slope_grid_.info.resolution);
        }
        
        index_base = ((int)((request->points[i].x - global_slope_grid_.info.origin.position.x)/global_slope_grid_.info.resolution)) +
                     ((int)((request->points[i].y - global_slope_grid_.info.origin.position.y)/global_slope_grid_.info.resolution))*global_slope_grid_.info.width;

        for(int j = -limit; j <= limit; j++){
          for(int k = -limit; k <= limit; k++){
            if(k*k+j*j > max_radius)
              continue;
            
            index = index_base + j + k*global_slope_grid_.info.width;

            if(index < 0 || index >= global_slope_grid_.info.width*global_slope_grid_.info.height)
              continue;

            global_slope_grid_.data[index] = request->slope_values[i];
          }
        }
      }

      publisher_global_slope_->publish(global_slope_grid_);
    }

    void update_type_cb(const std::shared_ptr<map_manager_srvs::srv::TerrainTypeUpdate::Request> request,
                        std::shared_ptr<map_manager_srvs::srv::TerrainTypeUpdate::Response>      response)
    {
      float radius = request->inflation_radius[0];
      int limit = (int)(radius/global_terrain_grid_.info.resolution);
      int max_radius = limit*limit;
      int index_base, index;

      //TODO put it into MapManager Class ?
      for(int i = 0; i < (int)(request->points.size()); i++){
        if(request->inflation_radius.size() > 1){
          radius = request->inflation_radius[i];
          limit = (int)(radius/global_terrain_grid_.info.resolution);
        }
        
        index_base = ((int)((request->points[i].x - global_terrain_grid_.info.origin.position.x)/global_terrain_grid_.info.resolution)) +
                     ((int)((request->points[i].y - global_terrain_grid_.info.origin.position.y)/global_terrain_grid_.info.resolution))*global_terrain_grid_.info.width;

        for(int j = -limit; j <= limit; j++){
          for(int k = -limit; k <= limit; k++){
            if(k*k+j*j > max_radius)
              continue;
            
            index = index_base + j + k*global_terrain_grid_.info.width;

            if(index < 0 || index >= global_terrain_grid_.info.width*global_terrain_grid_.info.height)
              continue;

            global_terrain_grid_.data[index] = request->terrain_types[i];
          }
        }
      }

      publisher_global_terrain_->publish(global_terrain_grid_);
    }


    MapManager map_terrain_manager_, map_slope_manager_;
    rclcpp::TimerBase::SharedPtr timer1_, timer2_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_global_terrain_, publisher_global_slope_;
    
    rclcpp::Service<map_manager_srvs::srv::TerrainSlopeUpdate>::SharedPtr update_slope_srv_;
    rclcpp::Service<map_manager_srvs::srv::TerrainTypeUpdate>::SharedPtr update_type_srv_;

    nav_msgs::msg::OccupancyGrid global_terrain_grid_, global_slope_grid_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapManagerServer>());
  rclcpp::shutdown();
  return 0;
}
