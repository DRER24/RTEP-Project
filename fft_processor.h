#ifndef PCL_VISUALIZATION_H
#define PCL_VISUALIZATION_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "radar_sensor.h"

class PCLVisualization {
public:
    PCLVisualization();
    void updatePointCloud(const std::vector<RadarData>& radarData);
    void run();

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

#endif