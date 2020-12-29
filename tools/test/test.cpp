
#include <iostream>
#include <ctime>
#include <chrono>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>


struct Color
{
    float r_, g_, b_;
    
    Color(float setr, float setg, float setb):r_(setr), g_(setg), b_(setb)
    {}
};
enum CameraAngle
{
    XY,TopDown, Side, Fps
};
struct  Box
{
float x_min_;
float y_min_;
float z_min_;
float x_max_;
float y_max_;
float z_max_;
};

Box BoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min_ = minPoint.x;
    box.y_min_ = minPoint.y;
    box.z_min_ = minPoint.z;
    box.x_max_ = maxPoint.x;
    box.y_max_ = maxPoint.y;
    box.z_max_ = maxPoint.z;

    return box;
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr &viewer, Box box, int id , Color color=Color(1,0,0), float opacity=1)
{

    if(opacity>1.0)
    {
         opacity=1.0;
    }
    if(opacity<0.0)
    {
        opacity=0.0;
    }

    std::string cube="box"+std::to_string(id);

    viewer->addCube(box.x_min_, box.x_max_, box.y_min_, box.y_max_, box.z_min_, box.z_max_, color.r_, color.g_, color.b_, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r_, color.g_, color.b_, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

}


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

pcl::PointCloud<pcl::PointXYZ>::Ptr FilterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector2f minPoint, Eigen::Vector2f maxPoint)
{

    auto startTime=std::chrono::steady_clock::now();
    std::cout<<"  Filter :"<<std::endl;
    std::cout<<"   - PointCloud before filtering: "<<cloud->width*cloud->height<<" data points ("<<pcl::getFieldsList(*cloud)<<")."<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloudX(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloudY(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(minPoint[0], maxPoint[0]);
    pass.filter(*filteredCloudX);

    pass.setInputCloud(filteredCloudX);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(minPoint[1], maxPoint[1]);
    pass.filter(*filteredCloudY);


    auto endTime=std::chrono::steady_clock::now();
    auto elapsedTime=std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);

    std::cout<< "   - PointCloud after filtering "<<filteredCloudY->width*filteredCloudY->height<<" data points("<<pcl::getFieldsList(*filteredCloudY)<<")."<<std::endl;
    std::cout<<"   - Point cloud filtering took "<<elapsedTime.count()<<" milliseconds."<<std::endl;
    //std::cout<<"   -"<<sumtime_.count()<<std::endl;

    return filteredCloudY;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SegmentPlane( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceThreshold, int file)
{
    auto startTime=std::chrono::steady_clock::now();
    std::cout<<"  Ground segment : "<<std::endl;

     pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PointCloud<pcl::PointXYZ>::Ptr rem_cloud(new pcl::PointCloud<pcl::PointXYZ>);

     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
     pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
     pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
     pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size()==0)
    {
        std::cerr<<"Could not estimate a plane model for the given dataset"<<std::endl;
        return nullptr;
    }
    //Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);  
    extract.filter(*seg_cloud);

    extract.setNegative(true);  //将索引对应的的点丢掉，保留剩余的点
    extract.filter(*rem_cloud);

    /*
    pcl::PCDWriter writer;
    std::stringstream segfile;
    segfile<<"../grou_out/"<<file<<"_ground.pcd";
    writer.write<pcl::PointXYZ>(segfile.str(),*seg_cloud,false);
    std::stringstream remfile;
    remfile<<"../rem_out/"<<file<<"_rem.pcd";
    writer.write<pcl::PointXYZ>(remfile.str(),*rem_cloud,false);
    */

    auto endTime=std::chrono::steady_clock::now();
    auto elapsedTime=std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);


    std::cout<< "   - PointCloud after segment "<<rem_cloud->width*rem_cloud->height<<" data points("<<pcl::getFieldsList(*rem_cloud)<<")."<<std::endl;
    std::cout<<"   - Point cloud ground segmentation took "<<elapsedTime.count()<<" milliseconds."<<std::endl;
    //std::cout<<"   -"<<sumtime_.count()<<std::endl;

    return rem_cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float distanceTolerance, int minSize=10, int maxSize=800)
{
    std::cout<<"  Obstacle :"<<std::endl;

    double dcos=std::cos(0.138);
    double dsin=std::sin(0.138);

    auto startTime = std::chrono::steady_clock::now();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;  // vector of point clouds

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  // KdTree object for optimised search during extraction
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(distanceTolerance);  // 2cm  //  PCL example uses 0.02
    ec.setMinClusterSize(minSize);  // PCL example uses 100
    ec.setMaxClusterSize(maxSize);  // PCL example uses 25000
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (pcl::PointIndices getIndices : clusterIndices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (int index : getIndices.indices)     //getIndices.indices聚类后一簇点云的索引
            cloudCluster->points.push_back(cloud->points[index]);

        pcl::PointXYZ maxPoint;
        pcl::PointXYZ minPoint;
        pcl::getMinMax3D(*cloudCluster, minPoint, maxPoint);

        std::cout<<"   - "<<maxPoint<<" "<<minPoint<<std::endl;
        //std::cout<<std::fabs(maxPoint.z-minPoint.z)<<std::endl;
        if(std::fabs(maxPoint.z-minPoint.z)<0.5)
        {
            continue;
        }
            if(std::fabs(maxPoint.z-minPoint.z)>6)
        {
            continue;
        }
        if(std::fabs(maxPoint.x-minPoint.x)>10)
        {
            continue;
        }
            if(std::fabs(maxPoint.y-minPoint.y)>6)
        {
            continue;
        }

        pcl::PointXYZ centerPoint;
        pcl::PointXYZ rcenterPoint;

        centerPoint.x=(maxPoint.x+minPoint.x)/2;
        centerPoint.y=(maxPoint.y+minPoint.y)/2;
        centerPoint.z=(maxPoint.z+minPoint.z)/2;
        std::cout<<"   - Center point: "<<centerPoint<<std::endl;

        rcenterPoint.x = dcos*centerPoint.x - dsin*centerPoint.z;
        rcenterPoint.y = centerPoint.y;
        rcenterPoint.z = dsin*centerPoint.x + dcos*centerPoint.z;  
        std::cout<<"   - Rotation Center point: "<<rcenterPoint<<std::endl;

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);

        std::cout << "   - PointCloud representing the cluster: " << cloudCluster->points.size() << " data points." << std::endl;
    }  

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "   - Obstacle took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    //std::cout<<"   -"<<sumtime_.count()<<std::endl;
    return clusters;
}


std::vector<boost::filesystem::path> streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    sort(paths.begin(), paths.end());  // sorting files in ascending order so playback is chronological

    return paths;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPcd(std::string file)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    /*
    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
    pcl::PCDReader reader;
    reader.read(file,*cloud_blob);
    pcl::fromPCLPointCloud2(*cloud_blob,*cloud);

    */
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1)
        PCL_ERROR("Couldn't read file \n");

    std::cout << "Loaded " << cloud->points.size () << " data points from " + file << std::endl;

    return cloud;
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string name, Color color)
{
    if(color.r_==-1)
    {
        //Select color based on cloud intensity
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> intensity_distribution(cloud, "intensity");
        viewer->addPointCloud<pcl::PointXYZ>(cloud, intensity_distribution, name);
    }
    else
    {
        //Select color based on input value
        viewer->addPointCloud<pcl::PointXYZ>(cloud,name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r_, color.g_, color.b_, name);
    }

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name); 
}


int main(int argc, char **argv)
{
    double grounddisthreshold;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcdcloud(new pcl::PointCloud<pcl::PointXYZ>);

    /*
    std::stringstream gds;
    gds<<argv[1];
    gds>>grounddisthreshold;
    */
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = Fps;
    initCamera(setAngle, viewer);
    std::vector<boost::filesystem::path> stream = streamPcd("../pcd/");
    //auto streamIterator = stream.begin(); 
    //pcdcloud = loadPcd((*streamIterator).string());  // dereferencing streamIterator and converting output path to string

    int i=0;
    for ( auto streamIterator = stream.begin() ;  streamIterator != stream.end() ; streamIterator++) 
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        pcdcloud->clear();
        pcdcloud = loadPcd((*streamIterator).string());   

       renderPointCloud(viewer, pcdcloud, "Cloud", Color(1, 1, 1));

        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud=FilterCloud(pcdcloud,Eigen::Vector2f(-40, 0), Eigen::Vector2f(40, 55));
        pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedCloud = SegmentPlane(filteredCloud, 1000, 0.191, i); 
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacleClusters = Clustering(segmentedCloud, 1.5, 10, 800);  
         
        i++;


        std::vector<Color> colours = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};  
        int clusterId = 0;


        for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : obstacleClusters)
        {
            // std::cout << "Cluster size: " << std::endl; pointCloudXYZIProcessor.numPoints(cluster);

            // Rendering cluster
            renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colours[2]);
        
            // Rendering bounding box
            // Since all the detectable vehicles in this scene are along the same axis as our car, the simple already-set-up bounding box function should yield good results.
            Box box = BoundingBox(cluster);
            renderBox(viewer, box, clusterId);

            viewer->spinOnce(2000000);

            ++clusterId;
        }

    }
}