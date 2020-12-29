#include "sensors/lidar.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"  // using templates for processPointClouds so including .cpp to help linker


void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(10.0);
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


void objectDetectionXYZI(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointCloudXYZIProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, std::string filename, double ground_dis_threshold, int cluster_min_points, double cluster_dis_threshold)  
{

    renderPointCloud(viewer, inputCloud, "Cloud", Color(1, 1, 1));

    //pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointCloudXYZIProcessor.FilterCloud(inputCloud,  Eigen::Vector2f(-10, 0), Eigen::Vector2f( 20, 50));
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
        // std::cout << "Cluster size: " << std::endl; pointCloudXYZIProcessor.numPoints(cluster);

        // Rendering cluster
        renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colours[2]);
    
        // Rendering bounding box
        // Since all the detectable vehicles in this scene are along the same axis as our car, the simple already-set-up bounding box function should yield good results.
        Box box = pointCloudXYZIProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}


int main(int argc, char** argv)
{
    std::cout << "Starting ......." << std::endl;
    double grounddisthreshold;
    int clusterminpoints;
    double clusterdisthreshold;

    grounddisthreshold = std::stof(argv[1]);
    clusterminpoints = std::stoi(argv[2]);
    clusterdisthreshold = std::stof(argv[3]);

    /*
    std::stringstream gds;
    gds<<argv[1];
    gds>>grounddisthreshold;

    std::stringstream cminp;
    cminp<<argv[2];
    cminp>>clusterminpoints;

    std::stringstream cds;
    cds<<argv[3];
    cds>>clusterdisthreshold;
    */

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = Fps;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI> pointCloudXYZIProcessor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcdCloud;
    // Passing path to directory containing sequentially-ordered PCD files. Returns a chronologically0ordered vector of all those file names
   // std::vector<boost::filesystem::path> stream = pointCloudXYZIProcessor.streamPcd("../src/sensors/data/pcd/data_1/");
    
    // make iterator start from the beginning  
    // Using an iterator to go through the stream vector
    // auto streamIterator = stream.begin(); 
    
    // 双层目录
    std::vector<boost::filesystem::path> dirpaths(boost::filesystem::directory_iterator{"../20201208_car_scene"}, boost::filesystem::directory_iterator{});

    for(auto & dp: dirpaths)
    {
        std::cout<<dp.string()<<std::endl;
        std::vector<boost::filesystem::path> stream = pointCloudXYZIProcessor.streamPcd(dp.string());
        auto streamIterator = stream.begin();

        while (streamIterator != stream.end())
        {   
            if ((*streamIterator).filename().extension().string() != ".pcd")
            {
                streamIterator++;
                continue;
            }

            else
            {
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();
                std::cout<<streamIterator->filename().string()<<std::endl;
                pcdCloud = pointCloudXYZIProcessor.loadPcd((*streamIterator).string());  // dereferencing streamIterator and converting output path to string
                objectDetectionXYZI(viewer, pointCloudXYZIProcessor, pcdCloud, streamIterator->filename().stem().string(), grounddisthreshold, clusterminpoints, clusterdisthreshold);
        
                streamIterator++;
                viewer->spinOnce(1000);  // controls the frame rate. By default it waits 1 time step, which would make it run as fast as possible. Frame rate depends on time efficiency of obstacle-detection functions   
            }
            
        }

    }

    /*
    while (!viewer->wasStopped ())  // PC Viewer run cycle  // frame update loop
    {
        std::cout<<streamIterator->filename()<<std::endl;
        // Clearing previously-rendered pointclouds and shapes from viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        // std::stringstream filepath;
        // filepath<<"/home/lfq/Point_Cloud_Detection/src/sensors/data/pcd/data_1/"<<i<<".pcd"; 
        
        // Loading PCD
        if(streamIterator->filename().extension()!=".pcd")
        {
            streamIterator++;  // TODO: check if same as ++streamIterator;
            continue;
        }
        pcdCloud = pointCloudXYZIProcessor.loadPcd((*streamIterator).string());  // dereferencing streamIterator and converting output path to string

        //pcdCloud = pointCloudXYZIProcessor.loadPcd(filepath.str()); 
        // Running obstacle-detection pipeline
        //std::cout<<clusterminpoints<<std::endl;
        objectDetectionXYZI(viewer, pointCloudXYZIProcessor, pcdCloud, streamIterator->filename().stem().string(), grounddisthreshold, clusterminpoints, clusterdisthreshold);
        
        streamIterator++;  // TODO: check if same as ++streamIterator;
        if (streamIterator == stream.end())  // when it reaches the end
            streamIterator = stream.begin();  // reset back to the beginning

        viewer->spinOnce(2000);  // controls the frame rate. By default it waits 1 time step, which would make it run as fast as possible. Frame rate depends on time efficiency of obstacle-detection functions  
    }
    */
}
