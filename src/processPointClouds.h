#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <ctime>
#include <cmath>
#include <vector>
#include <chrono>
#include <string>  
#include <iostream> 
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
//#include <experimental/filesystem>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "kdtree.h"
#include "render/box.h"

template<typename PointT>
class ProcessPointClouds
{

private:
    //std::chrono::milliseconds  sumtime_;
    std::chrono::duration<double, std::milli>  sumtime_ ;
public:
    ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector2f minPoint, Eigen::Vector2f maxPoint);

    typename pcl::PointCloud<PointT>::Ptr SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold,  std::string filename);

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, std::string filename, float distanceTolerance,int minSize=10, int maxSize=2000,  bool usePCLClustering=true);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file , bool  scene_test=false);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    void showalltime();

    pcl::PointXYZ cusrotationpoint( pcl::PointXYZ  rawpoint)
    {   
    pcl::PointXYZ  rotpoint;
    double dcos=std::cos(0.138);
    double dsin=std::sin(0.138);
    rotpoint.x = dcos*rawpoint.x - dsin*rawpoint.z;
    rotpoint.y = rawpoint.y;
    rotpoint.z = dsin*rawpoint.x + dcos*rawpoint.z; 

    return rotpoint;
    };

    bool Trickwork(pcl::PointXYZ  p8)
    {
        // if( (8.67675<p8.y && p8.y<11.58155) && (48.0815<p8.x && p8.x<51.7687))
        if( (5.0<p8.y && p8.y<12.88155) && (43.5815<p8.x && p8.x<53.7687))
        {
            return true;
        }
        else
        {
            return false;
        }  
    };

    ~ProcessPointClouds();

};

struct sort_functor
{
    bool operator ()(const boost::filesystem::path & a,const boost::filesystem::path & b)
    {
        return std::stoi(boost::filesystem::basename(a)) < std::stoi(boost::filesystem::basename(b));  
    }
};

#endif /* PROCESSPOINTCLOUDS_H_ */
