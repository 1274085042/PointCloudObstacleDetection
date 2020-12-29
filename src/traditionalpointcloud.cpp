#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "sensors/lidar.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"  // using templates for processPointClouds so including .cpp to help linker

namespace py = pybind11;

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();  // set camera position and angle
    int distance = 16;  // meters
    
    switch (setAngle)  // switching camera angle
    {
        case XY: viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown: viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side: viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case Fps: viewer->setCameraPosition(-10, 0, 10, -1, 0, 3);
    }

    if (setAngle!=Fps)
        viewer->addCoordinateSystem(1.0);
}


std::vector<std::vector<double>> objectDetectionXYZI(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointCloudXYZIProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, std::string filename, double ground_dis_threshold, int cluster_min_points, double cluster_dis_threshold)  
{

    renderPointCloud(viewer, inputCloud, "Cloud", Color(1, 1, 1));

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointCloudXYZIProcessor.FilterCloud(inputCloud,  Eigen::Vector2f(0, -20), Eigen::Vector2f(50, 10));  
    //pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointCloudXYZIProcessor.FilterCloud(inputCloud, Eigen::Vector2f(-20, 0), Eigen::Vector2f(40, 55));  

    // Segmenting ground and obstacles
    // setDistanceThreshold越大 分割的地面点越多
    pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedCloud = pointCloudXYZIProcessor.SegmentPlane(filteredCloud, 1000, ground_dis_threshold, filename); 
    //renderPointCloud(viewer, segmentedCloud, "groundPlaneCloud", Color(0, 1, 0));
    
    // Clustering obstacle points into objects
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacleClusters = pointCloudXYZIProcessor.Clustering(segmentedCloud, filename, cluster_dis_threshold, cluster_min_points);  
 
    std::vector<Color> colours = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};  
    int clusterId = 0;

    pointCloudXYZIProcessor.showalltime();

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : obstacleClusters)
    {
        // std::cout << "Cluster size: " << std::endl; pointCloudXYZProcessor.numPoints(cluster);

        // Rendering cluster
        renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colours[2]);
    
        // Rendering bounding box
        // Since all the detectable vehicles in this scene are along the same axis as our car, the simple already-set-up bounding box function should yield good results.
        Box box = pointCloudXYZIProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
    
    return pointCloudXYZIProcessor.returnres();
}


std::vector<std::vector<double>> objectDetectionXYZI( ProcessPointClouds<pcl::PointXYZI> pointCloudXYZIProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, std::string filename, double ground_dis_threshold, int cluster_min_points, double cluster_dis_threshold)  
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointCloudXYZIProcessor.FilterCloud(inputCloud,  Eigen::Vector2f(0, -20), Eigen::Vector2f(50, 10));  
    //pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointCloudXYZIProcessor.FilterCloud(inputCloud, Eigen::Vector2f(-20, 0), Eigen::Vector2f(40, 55));  

    // Segmenting ground and obstacles
    // setDistanceThreshold越大 分割的地面点越多
    pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedCloud = pointCloudXYZIProcessor.SegmentPlane(filteredCloud, 1000, ground_dis_threshold, filename); 
    
    // Clustering obstacle points into objects
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacleClusters = pointCloudXYZIProcessor.Clustering(segmentedCloud, filename, cluster_dis_threshold, cluster_min_points);  

    pointCloudXYZIProcessor.showalltime();
    
    return pointCloudXYZIProcessor.returnres();
}


std::vector< std::vector<double> > obstacledetection(std::string filepath , bool showpointcloud = false)
{

    std::cout << "Starting ......." << std::endl;

    ProcessPointClouds<pcl::PointXYZI> pointCloudXYZProcessor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcdCloud;

    int i=1;
    
    std::cout<<filepath<<std::endl;
    boost::filesystem::path bfilepath=filepath;
    std::cout<<bfilepath.stem()<<std::endl;

    pcdCloud = pointCloudXYZProcessor.loadPcd(filepath);

    if(showpointcloud==true)
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        CameraAngle setAngle = Fps;
        initCamera(setAngle, viewer);
        std::vector<std::vector<double>> obstacles = objectDetectionXYZI(viewer, pointCloudXYZProcessor, pcdCloud,filepath,  0.191, 10, 1);   //0.191, 10, 1.5  
        viewer->spinOnce(4000); 
        return obstacles;
    }
    else
    {
        std::vector<std::vector<double>> obstacles = objectDetectionXYZI(pointCloudXYZProcessor, pcdCloud,filepath,  0.191, 10, 1);   //0.191, 10, 1.5  
        return obstacles; 
    }
}


PYBIND11_MODULE(traditionalpointcloud, m) 
{
    m.doc() = "traditional point cloud process module (by TangJiHeDe)";  // optional module docstring

    m.def("obstacledetection", &obstacledetection, "obstacle detection",  py::arg("filepath") ,py::arg("showpointcloud")=false );
}