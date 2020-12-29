#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
   //pcl::PointCloud<pcl::PointXYZI>::Ptr cloudxyzi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCLPointCloud2::Ptr cloudxyzi(new  pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>);

    /*
    if(pcl::io::loadPCDFile("../data/1.pcd", *cloudxyzi)==-1)
    {
        return -1;
    }
    */
   pcl::PCDReader reader;
   reader.read("../../data/1.pcd", *cloudxyzi);
    pcl::fromPCLPointCloud2(*cloudxyzi, *cloudxyz);

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("1.pcd", *cloudxyz , false);
}