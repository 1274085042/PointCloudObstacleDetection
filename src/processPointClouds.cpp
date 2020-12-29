#include "processPointClouds.h"

// Constructor
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds():sumtime_(0)
{

}


template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout<<cloud->points.size()<<std::endl;
}


template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector2f minPoint, Eigen::Vector2f maxPoint)
{

    auto startTime=std::chrono::steady_clock::now();
    std::cout<<"  Filtered :"<<std::endl;
    std::cout<<"   - PointCloud before filtering: "<<cloud->width*cloud->height<<" data points ("<<pcl::getFieldsList(*cloud)<<")."<<std::endl;

    typename pcl::PointCloud<PointT>::Ptr filteredCloudX(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr filteredCloudY(new pcl::PointCloud<PointT>);

    pcl::PassThrough<PointT> pass;
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

    sumtime_+=elapsedTime;

    std::cout<< "   - PointCloud after filtering "<<filteredCloudY->width*filteredCloudY->height<<" data points("<<pcl::getFieldsList(*filteredCloudY)<<")."<<std::endl;
    std::cout<<"   - Point cloud filtering took "<<elapsedTime.count()<<" milliseconds."<<std::endl;
    //std::cout<<"   -"<<sumtime_.count()<<std::endl;

    //pcl::PCDWriter writer;
    //writer.write<PointT>("../exhibition_out/filtered.pcd",*filteredCloudY,false);

    return filteredCloudY;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, std::string filename)
{
    auto startTime=std::chrono::steady_clock::now();
    std::cout<<"  Ground segment : "<<std::endl;

    typename pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr rem_cloud(new pcl::PointCloud<PointT>);

    typename pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    typename pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    typename pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    typename pcl::ExtractIndices<PointT> extract;
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
    segfile<<"../ground_cloud/"<<filename<<"_ground.pcd";
    writer.write<PointT>(segfile.str(),*seg_cloud,false);
    std::stringstream remfile;
    remfile<<"../rem_cloud/"<<filename<<"_rem.pcd";
    writer.write<PointT>(remfile.str(),*rem_cloud,false);
    */

    auto endTime=std::chrono::steady_clock::now();
    auto elapsedTime=std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);

    sumtime_+=elapsedTime;
    std::cout<< "   - PointCloud after segment "<<rem_cloud->width*rem_cloud->height<<" data points("<<pcl::getFieldsList(*rem_cloud)<<")."<<std::endl;
    std::cout<<"   - Point cloud ground segmentation took "<<elapsedTime.count()<<" milliseconds."<<std::endl;
    //std::cout<<"   -"<<sumtime_.count()<<std::endl;

    return rem_cloud;
}


template<typename PointT>
void euclideanClusteringHelper(int index, const typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, kdTree<PointT>* tree, float distanceTolerance)  // & cluster (works), & tree (doesn't)
{
    // If we have never processed this point before
    std::cout << "\tI" << index << " : " << processed[index] << std::endl;
    processed[index] = true;

    cluster.push_back(index);
    std::vector<int> nearby = tree->search(index, distanceTolerance);

    int unprocessed_points = 0;
    for (int id : nearby) {
        if (processed[id] != true) {
            unprocessed_points += 1; 
        }
    }  
    std::cout << "\t\t" << unprocessed_points << "/" << nearby.size() << " : " << " left to process." << std::endl;

    for (int id : nearby) {
        if (processed[id] != true) {
            euclideanClusteringHelper(id, cloud, cluster, processed, tree, distanceTolerance);
        }
    }
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, kdTree<PointT>* tree, float distanceTol)  // TODO: should tree be passed by reference
{
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool> processed(cloud->points.size(), false);  // same size as points, each of which starts as false

	int i = 0;
	while (i < cloud->points.size())
	{
		if (processed[i] == false)
		{
            // If the point has not been processed, create a new cluster
            std::vector<int> newIdCluster;
            std::cout << "\nCreating a new cluster - Index " << i << std::endl;
            euclideanClusteringHelper(i, cloud, newIdCluster, processed, tree, distanceTol);  // i: point id, cluster passed by reference

            typename pcl::PointCloud<PointT>::Ptr newPointCluster(new typename pcl::PointCloud<PointT>());
            std::cout << "\nCreating Cluster\n" << std::endl;
            for (int id : newIdCluster)
            {
                std::cout << "\tAdding " << "(" << cloud->points[id].x << ", " << cloud->points[id].y << ", " << cloud->points[id].z << ")" << std::endl;
                newPointCluster->points.push_back((cloud->points[id]));
            }
            clusters.push_back(newPointCluster);  // Assertion failed: (px != 0), function operator->, file /usr/local/include/boost/smart_ptr/shared_ptr.hpp, line 734. // Abort trap: 6
		}
		i++;
	}

	return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, std::string filename, float distanceTolerance,  int minSize, int maxSize, bool usePCLClustering)
{
    std::cout<<"  Obstacle :"<<std::endl;
    //std::cout<<minSize<<std::endl;
    std::stringstream sfilename;
    sfilename<<"../obstacle_out/"<<filename<<".txt";

    //std::ofstream file(sfilename.str(), std::ios::out | std::ios::app);

    //double dcos=std::cos(0.138);
    //double dsin=std::sin(0.138);

    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;  // vector of point clouds

    if (usePCLClustering == true)                                 // use built-in PCL euclidean-clustering functions
    {
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);  // KdTree object for optimised search during extraction
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(distanceTolerance);  // 2cm  //  PCL example uses 0.02
        ec.setMinClusterSize(minSize);  // PCL example uses 100
        ec.setMaxClusterSize(maxSize);  // PCL example uses 25000
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(clusterIndices);

        size_t i=0;

        for (pcl::PointIndices getIndices : clusterIndices)
        {
            typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

            for (int index : getIndices.indices)     //getIndices.indices聚类后一簇点云的索引
                cloudCluster->points.push_back(cloud->points[index]);

            PointT maxPoint;
            PointT minPoint;
            pcl::getMinMax3D(*cloudCluster, minPoint, maxPoint);

            //std::cout<<"   - "<<maxPoint<<" "<<minPoint<<std::endl;
            //std::cout<<std::fabs(maxPoint.z-minPoint.z)<<std::endl;

            //根据障碍物的大小过滤一波
            if(std::fabs(maxPoint.z-minPoint.z)<0.5)
            {
                continue;
            }
             if(std::fabs(maxPoint.z-minPoint.z)>6)
            {
                continue;
            }
            
            if(std::fabs(maxPoint.x-minPoint.x)>15)
            {
                continue;
            }
             if(std::fabs(maxPoint.y-minPoint.y)>15)
            {
                continue;
            }

            typename pcl::PointXYZ centerPoint;
            typename pcl::PointXYZ rcenterPoint;

            centerPoint.x=(maxPoint.x+minPoint.x)/2;
            centerPoint.y=(maxPoint.y+minPoint.y)/2;
            centerPoint.z=(maxPoint.z+minPoint.z)/2;

            rcenterPoint = cusrotationpoint(centerPoint);

            pcl::PointXYZ  point1(minPoint.x, minPoint.y, minPoint.z);   
            pcl::PointXYZ  point2(maxPoint.x, minPoint.y, minPoint.z);
            pcl::PointXYZ  point3(minPoint.x, maxPoint.y, minPoint.z);
            pcl::PointXYZ  point4(maxPoint.x, maxPoint.y, minPoint.z);
            pcl::PointXYZ  point5(minPoint.x, minPoint.y, maxPoint.z);
            pcl::PointXYZ  point6(maxPoint.x, minPoint.y, maxPoint.z);
            pcl::PointXYZ  point7(minPoint.x, maxPoint.y, maxPoint.z);
            pcl::PointXYZ  point8(maxPoint.x, maxPoint.y, maxPoint.z);
            pcl::PointXYZ  rpoint1=cusrotationpoint(point1);
            pcl::PointXYZ  rpoint2=cusrotationpoint(point2);
            pcl::PointXYZ  rpoint3=cusrotationpoint(point3);

            double l= maxPoint.x - minPoint.x;
            double w= maxPoint.y - minPoint.y;
            double h= maxPoint.z - minPoint.z;
            
            if(Trickwork(point8))     //右上角的误检
            {
                continue;
            }
            
            if( centerPoint.x<9 && centerPoint.y<-11 )  //左下角的误检
            {
                continue;
            }
            std::cout<<"   ["<<i<<"] "<<maxPoint<<" "<<minPoint<<std::endl;
            std::cout<<"   ["<<i<<"] Center point: "<<centerPoint<<std::endl;         
            //rcenterPoint.x = dcos*centerPoint.x - dsin*centerPoint.z;
            //rcenterPoint.y = centerPoint.y;
            //rcenterPoint.z = dsin*centerPoint.x + dcos*centerPoint.z;  
            std::cout<<"   ["<<i<<"] Rotation Center point: "<<rcenterPoint<<std::endl;

            std::cout<<point1<<" "<<point2<<" "<<point3<<" "<<point4<<std::endl;
            std::cout<<point5<<" "<<point6<<" "<<point7<<" "<<point8<<std::endl;
            std::cout<<l<<" "<<w<<" "<<h<<std::endl;

            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;
            clusters.push_back(cloudCluster);

            std::cout << "   ["<<i<<"] PointCloud representing the cluster: " << cloudCluster->points.size() << " data points." << std::endl;

            /*
            file<<centerPoint.x<<" "<<centerPoint.y<<" "<<centerPoint.z<<" "<<l<<" "<<w<<" "<<h<<" "<<rcenterPoint.x<<" "<<rcenterPoint.y<<" "<<rcenterPoint.z<<" "
            <<rpoint8.x<<" "<<rpoint8.y<<" "<<rpoint8.z<<" "
            <<rpoint6.x<<" "<<rpoint6.y<<" "<<rpoint6.z<<" "
            <<rpoint5.x<<" "<<rpoint5.y<<" "<<rpoint5.z<<" "
            <<rpoint7.x<<" "<<rpoint7.y<<" "<<rpoint7.z<<" "
            <<rpoint4.x<<" "<<rpoint4.y<<" "<<rpoint4.z<<" "
            <<rpoint2.x<<" "<<rpoint2.y<<" "<<rpoint2.z<<" "
            <<rpoint1.x<<" "<<rpoint1.y<<" "<<rpoint1.z<<" "
            <<rpoint3.x<<" "<<rpoint3.y<<" "<<rpoint3.z<<std::endl;
            */

            i+=1;
        }   
    }
    else  // use custom clustering algorithm
    {
        kdTree<PointT>* tree3D = new kdTree<PointT>(cloud);  // on stack: KdTree<PointT> tree3D;  // TOCHECK: difference between () and no ()

        for (int i = 0; i < cloud->points.size(); i++) 
            tree3D->insertPointIndex(i);
        std::cout << "\nKD Tree Built - Size : " << cloud->points.size() << "\n" << std::endl;

        clusters = euclideanClustering(cloud, tree3D, distanceTolerance);
        std::cout << "\nClusters Found : " << clusters.size() << "\n" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    sumtime_+=elapsedTime;
    std::cout << "   - Obstacle took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    //std::cout<<"   -"<<sumtime_.count()<<std::endl;
    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    PointT minPoint, maxPoint;
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


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to " + file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file, bool scene_test)
{
    sumtime_=std::chrono::duration<double, std::milli> (0);

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    
    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1)
        PCL_ERROR("Couldn't read file \n");

    std::cout << "Loaded " << cloud->points.size () << " data points from " + file << std::endl;

    if (scene_test==true)
    {
        for (size_t i = 0; i < cloud->points.size (); i++)
        {
            PointT tmp=cloud->points[i];
            //std::cout<<tmp<<"   "<<cloud->points[i].intensity<<std::endl;
            cloud->points[i].x=tmp.y;
            cloud->points[i].y=-tmp.x;
        }

            return cloud;
    }
    else
    {
        /*
        pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
        pcl::PCDReader reader;
        reader.read(file,*cloud_blob);
        pcl::fromPCLPointCloud2(*cloud_blob,*cloud);

        */
        return cloud;
    }
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath )
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    //sort(paths.begin(), paths.end(), sort_functor());  // sorting files in ascending order so playback is chronological
    sort(paths.begin(), paths.end());  // sorting files in ascending order so playback is chronological

    //sort(paths.begin(), paths.end());  
    return paths;
}


template<typename PointT>
void ProcessPointClouds<PointT>::showalltime()
{
    std::cout<<"---------------------- sum time: "<<sumtime_.count()<<" ---------------------- "<<std::endl;
}


// Destructor
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() 
{
}