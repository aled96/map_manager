#ifndef MAP_MANAGER_HPP_
#define MAP_MANAGER_HPP_

#include <memory>
#include <string>
#include <chrono>

#include <iostream> // cout, cerr
#include <fstream> // ifstream
#include <sstream> // stringstream

#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;
using namespace std;

namespace map_manager
{

    class MapManager
    {
        public:
            MapManager();

            void loadImage(std::string filename, float resolution, geometry_msgs::msg::Pose origin, std::string frame_id);

            // NOTE: HEre Image is starting at the top-left corner and goes down per row
            std::vector<int> getImage(){ return image;}

            int getRows(){ return numrows;}
            int getCols(){ return numcols;}

            int getMaxVal(){ return maxVal;}

            void printImage();

            nav_msgs::msg::OccupancyGrid getOccupancyMessage(){ return occupancy_grid_;}
            
        private:
            void imageToOccupancy();
        
            std::vector<int> image;
            nav_msgs::msg::OccupancyGrid occupancy_grid_;
            
            float resolution_;
            geometry_msgs::msg::Pose origin_;
            std::string frame_id_;

            int row, col, numrows, numcols;
            int maxVal;
        };


}  // namespace map_manager

#endif  // MAP_MANAGER_HPP_
