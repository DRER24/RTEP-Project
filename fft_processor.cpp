#include "pcl_visualization.h"
#include <cmath>

PCLVisualization::PCLVisualization() {
    viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("3D Radar Mapping"));
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

void PCLVisualization::updatePointCloud(const std::vector<RadarData>& radarData) {
    cloud->clear();
    for (const auto& data : radarData) {
        float x = data.distance * cos(data.angle * CV_PI / 180) * cos(data.elevation * CV_PI / 180);
        float y = data.distance * sin(data.angle * CV_PI / 180) * cos(data.elevation * CV_PI / 180);
        float z = data.distance * sin(data.elevation * CV_PI / 180);
        cloud->push_back(pcl::PointXYZ(x, y, z));
    }
    viewer->updatePointCloud(cloud, "radar_cloud");
}

void PCLVisualization::run() {
    viewer->addPointCloud(cloud, "radar_cloud");
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}