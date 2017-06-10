#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <vector>
#include <string>
#include <ostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include "tr3/common.h"
#include <opencv2/opencv.hpp>

const float INCR = 0.2;
const float SENSOR_DISP = 0.15;

// x200
// 0 - obstacle
// 0.5 - dont know
// 1 - clear - 5 - robot pos (rastreio)
class Cell
{
private:
    double center_x_;
    double center_y_;
    double value_;
    size_t id_row_;
    size_t id_col_;

public:
    // in centimeters
    Cell(double cx, double cy, double value, size_t id_row = 0, size_t id_col = 0)
    { center_x_ = cx; center_y_ = cy; value_ = value; id_row_ = id_row, id_col_ = id_col;}

    double getCenterX() const { return center_x_;}
    double getCenterY() const { return center_y_;}
    double getValue() const { return value_;}
    void setValue(double value){value_ = value;}

    size_t getIdRow() const {return id_row_;}
    size_t getIdCol() const {return id_col_;}
};

class OccupancyGrid
{
private:
    sensor_msgs::LaserScan scan_msg_;
    nav_msgs::Odometry pose_msg_;


    std::vector< std::vector<Cell*> > cells_grid_;
    std::vector<cv::Mat> mat_cells_grid_;

    std::vector<double> nodesCenterX_;
    std::vector<double> nodesCenterY_;
    std::vector<double> nodesDispX_;
    std::vector<double> nodesDispY_;

    double robot_size_x_;
    double robot_size_y_;
    int actual_node_;

    // callback
    // de pose_msg_
    double ori_x_;
    double ori_y_;
    double ori_angle_;

    bool isInsideNode(double actual_pos_x, double actual_pos_y);
    double distFromSensor(double pos_x, double pos_y);
    void setCellByPos(double pos_x, double pos_y, double value);
    void updateResources();

    void obtainLaserSamples(int laser_size);

public:
    OccupancyGrid();
    ~OccupancyGrid();

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg){this->scan_msg_ = *laser_msg;}
    void odomCallback(const nav_msgs::Odometry::ConstPtr& pose_msg){this->pose_msg_ = *pose_msg;}

    void createCicGrid();
    void UpdateMap();
    void showImageNode();
    void saveGridsInText();

    void setNodesCenterX(std::vector<double> vec_x){nodesCenterX_ = vec_x;}
    void setNodesCenterY(std::vector<double> vec_y){nodesCenterY_ = vec_y;}
    void setNodesDispX(std::vector<double> vec_x){nodesDispX_ = vec_x;}
    void setNodesDispY(std::vector<double> vec_y){nodesDispY_ = vec_y;}

    void setActualNode(int node){actual_node_ = node;}
};

#endif // OCCUPANCY_GRID_H
