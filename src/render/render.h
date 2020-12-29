#ifndef __RENDER_H__
#define __RENDER_H__
#include "box.h"
#include <vector>
#include <string>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>

struct Color
{
    float r_, g_, b_;
    
    Color(float setr, float setg, float setb):r_(setr), g_(setg), b_(setb)
    {}
};


struct Vec3
{
    double x_, y_, z_;

    Vec3(double setx, double sety, double setz):x_(setx), y_(sety), z_(setz)
    {}

    Vec3 operator+(const Vec3& vec)
    {
        Vec3 result(x_+vec.x_, y_+vec.y_, z_+vec.z_);
        return result;
    }
};


enum CameraAngle
{
    XY,TopDown, Side, Fps
};


struct Car
{
    Vec3 position_, dimensions_;
    std::string name_;
    Color color_;

    Car(const Vec3 &setposition, const Vec3 &setdimension, const Color &setcolor, const std::string setname)
    :position_(setposition), dimensions_(setdimension), color_(setcolor), name_(setname)
    {}

    void render(pcl::visualization::PCLVisualizer::Ptr &viewer)
    {
        //rendering bottom of car
        viewer->addCube(position_.x_-dimensions_.x_/2, position_.x_+dimensions_.x_/2, position_.y_-dimensions_.y_/2, position_.y_+dimensions_.y_/2, position_.z_, position_.z_+dimensions_.z_*2/3, color_.r_, color_.g_, color_.b_, name_);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name_);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_.r_, color_.g_, color_.b_, name_);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name_+"Top");
    }

    bool inbetween(double point, double center, double range)
    {
        // checkCollision helper function
        return (center-range<=point) &&(center+range>=point);
    }

    bool checkCollision(Vec3 &point)
    {
        return(inbetween(point.x_, position_.x_, dimensions_.x_/2) && inbetween(point.y_,position_.y_,position_.y_/2) && inbetween(point.z_, position_.z_+dimensions_.z_/3, dimensions_.z_/3 ))
        ||(inbetween(point.x_, position_.x_, dimensions_.x_/4) && inbetween(point.y_,position_.y_,position_.y_/2) && inbetween(point.z_, position_.z_+dimensions_.z_*5/6, dimensions_.z_/6 ));   
    }
};


void renderHighway(pcl::visualization::PCLVisualizer::Ptr &viewer);
void renderRays(pcl::visualization::PCLVisualizer::Ptr &viewer, const Vec3&origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
void clearRays(pcl::visualization::PCLVisualizer::Ptr &viewer);
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string name, Color color=Color(1,1,1));
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string name, Color color=Color(-1,-1,-1));
void renderBox(pcl::visualization::PCLVisualizer::Ptr &viewer, Box box, int id , Color color=Color(1,0,0), float opacity=1);
void renderBox(pcl::visualization::PCLVisualizer::Ptr &viewer, BoxQ box, int id , Color color=Color(1,0,0), float opacity=1);

#endif