#include "map_manager/map_manager.hpp"

namespace map_manager
{
    MapManager::MapManager()
    {
        row = 0;
        col = 0;
        numrows = 0;
        numcols = 0;
    }
    
    void MapManager::loadImage(std::string filename, float resolution, geometry_msgs::msg::Pose origin, std::string frame_id)
    {
        ifstream infile(filename);
        stringstream ss;
        string inputLine = "";

        // First line : version
        getline(infile,inputLine);
        // cout << "Version : " << inputLine << endl;

        // Second line : comment
        getline(infile,inputLine);
        // cout << "Comment : " << inputLine << endl;

        // Continue with a stringstream
        ss << infile.rdbuf();
        // Third line : size
        ss >> numcols >> numrows;
        // cout << numcols << " columns and " << numrows << " rows" << endl;

        ss >> maxVal;

        // cout << "Max value " << maxVal << endl;
        
        if(image.size() != numrows*numcols)
            image.resize(numrows*numcols);

        // Following lines : data
        for(row = 0; row < numrows; ++row)
            for (col = 0; col < numcols; ++col) 
                ss >> image[row + numrows*col];

        infile.close();

        frame_id_ = frame_id;
        resolution_ = resolution;
        origin_ = origin;

        imageToOccupancy();
    }

    void MapManager::printImage(){
        // Now print the array to see the result
        for(row = 0; row < numrows; ++row) {
            for(col = 0; col < numcols; ++col) {
            cout << (int)(image[row + numrows*col]) << " ";
            }
            cout << endl;
        }        
    }

    void MapManager::imageToOccupancy(){
        occupancy_grid_.header.frame_id = frame_id_;

        occupancy_grid_.info.resolution = resolution_;
        occupancy_grid_.info.width = numrows;
        occupancy_grid_.info.height = numcols;
        occupancy_grid_.info.origin = origin_;

        occupancy_grid_.data.resize(numrows*numcols);
        
        for(int i = 0; i < numrows; i ++){
            for(int j = 0; j < numcols; j ++){
                occupancy_grid_.data[numcols*(numrows-1 - i) +j] = 100 - (int)(100.0*(float)(image[i + numrows*j])/(float)(maxVal)); //Convert in 0-100
            }
        }
    }
}