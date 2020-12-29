#include "render.h"

void renderHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    double roadLength=50.0; //meters
    double roadWidth=12.0;  //meters
    double roadHeight=0.2;  //meters

    viewer->addCube(-roadLength/2,roadLength/2,-roadWidth/2,roadWidth/2,-roadHeight,0, .2, .2, .2, "highwayPavement");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,"highwayPavement");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, .2, .2, .2, "highwayPavement");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "highwayPavement");
    viewer->addLine(pcl::PointXYZ(-roadLength/2, -roadWidth/6, 0.01), pcl::PointXYZ(roadLength/2, -roadWidth/6, 0.01), 1, 1, 0, "line1");
    viewer->addLine(pcl::PointXYZ(-roadLength/2, roadWidth/6, 0.01), pcl::PointXYZ(roadLength/2, roadWidth/6, 0.01), 1, 1, 0, "line2");
}


int countRays=0;
void renderRays(pcl::visualization::PCLVisualizer::Ptr &viewer, const Vec3&origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    for(pcl::PointXYZ point : cloud->points)
    {
        viewer->addLine(pcl::PointXYZ(origin.x_, origin.y_,origin.z_), point, 1, 0, 0, "ray"+std::to_string(countRays));
        countRays++;
    }
}


void clearRays(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    while (countRays)
    {
        countRays--;
        viewer->removeShape("ray"+std::to_string(countRays));
    }
    
}


void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string name, Color color)
{
    viewer->addPointCloud<pcl::PointXYZ>(cloud,name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,4,name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r_, color.g_, color.b_, name);
}


void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string name, Color color)
{
    if(color.r_==-1)
    {
        //Select color based on cloud intensity
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
        viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
    }
    else
    {
        //Select color based on input value
        viewer->addPointCloud<pcl::PointXYZI>(cloud,name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r_, color.g_, color.b_, name);
    }

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name); 
}


void renderBox(pcl::visualization::PCLVisualizer::Ptr &viewer, Box box, int id , Color color, float opacity)
{
    //Draws wireframe box with filled transparent colour

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

    /*
   std::string cubeFill="boxFill" +std::to_string(id);

   viewer->addCube(box.x_min_, box.x_max_, box.y_min_, box.y_max_, box.z_min_, box.z_max_, color.r_, color.g_, color.b_, cubeFill);
   viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
   viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r_, color.g_, color.b_, cubeFill);
   viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
   */
}


void renderBox(pcl::visualization::PCLVisualizer::Ptr &viewer, BoxQ box, int id , Color color, float opacity)
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
    viewer->addCube(box.bboxTransform_, box.bboxQuaternion_, box.cube_length_, box.cube_width_, box.cube_height_, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r_, color.g_, color.b_, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill="boxFill"+std::to_string(id);
    viewer->addCube(box.bboxTransform_, box.bboxQuaternion_, box.cube_length_, box.cube_width_, box.cube_height_, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r_, color.g_, color.b_, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}