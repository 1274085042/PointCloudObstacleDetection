#ifndef __LIDAR_H__
#define __LIDAR_H__
#include <ctime>
#include <chrono>
#include "../render/render.h"

const double PI=3.1415;

struct  Ray
{
    Vec3 origin_;
    double resolution_;
    Vec3 direction_;
    Vec3 castPosition_;
    double castDistance_;

/*
setOrigin: the starting position from where the ray is cast
horizontalAngle: the angle of direction the ray travels on the xy plane
verticalAngle: the angle of direction between xy plane and ray. E.g.0 radians is along xy plane and PI/2 radians is straight up
resolution: the magnitude of the ray's step, used for ray casting (the smaller the more accurate but the more expensive)
*/

    Ray(Vec3 setOrigin, double horizontalAngle, double verticalAngle, double setResolution)
    :origin_(setOrigin),resolution_(setResolution),direction_(resolution_*cos(verticalAngle)*cos(horizontalAngle),resolution_*cos(verticalAngle)*sin(horizontalAngle),resolution_*sin(verticalAngle)),castPosition_(origin_),castDistance_(0)
    {}

    void rayCast(const std::vector<Car>& cars, double minDistance, double maxDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double slopeAngle, double sderr)
    {
        //reset ray
         castPosition_=origin_;
         castDistance_=0;

         bool collision=false;

         while(!collision && castDistance_<maxDistance)
         {
             castPosition_=castPosition_+direction_;
             castDistance_+=resolution_;

             //check if there are any collisions with ground slope
             collision=(castPosition_.z_<=castPosition_.x_*tan(slopeAngle));

             //check if there are any collisions with cars
             if(!collision && castDistance_<maxDistance)
             {
                 for(Car car:cars)
                 {
                     collision |=car.checkCollision(castPosition_);
                     if(collision)
                     break;
                 }
             }
         }

         if((castDistance_>=minDistance)&&(castDistance_<=maxDistance))
         {
             //add noise based on standard deviation error
             double rx=((double) rand()/(RAND_MAX));
             double ry=((double) rand()/(RAND_MAX));
             double rz=((double) rand()/(RAND_MAX));

             cloud->points.push_back(pcl::PointXYZ(castPosition_.x_+rx*sderr,castPosition_.y_+ry*sderr,castPosition_.z_+rz*sderr));
         }

    }

};


struct Lidar
{
    std::vector<Ray> rays_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    std::vector<Car> cars_;
    
    Vec3 position_;

    double groundSlope_;
    double minDistance_;
    double maxDistance_;
    double resolution_;
    double sderr_;

    Lidar(std::vector<Car> setCars, double setGroundSlope): cloud_(new pcl::PointCloud<pcl::PointXYZ>()),position_(0,0,2.6)
    {
        minDistance_=5;   //to remove points from roof of ego car
        maxDistance_=50;
        resolution_=0.2;
        sderr_=0.2;       //meters //to get more interesting PCD files
        cars_=setCars;
        groundSlope_=setGroundSlope;
        
        int numLayers=8;   //correlated with resolution of PCD
        double steepestAngle=30.0*(-PI/180);   //vertical
        double angleRange=26.0*(PI/180);
        double horizontalAngleInc=PI/64;       //correlated with resolution of PCD
        double angleIncrement=angleRange/numLayers;

        for(double angleVertical=steepestAngle; angleVertical<steepestAngle+angleRange;angleVertical+=angleIncrement)
        {
            for(double angle=0; angle<=2*PI;angle+=horizontalAngleInc)
            {
                Ray ray(position_,angle,angleVertical,resolution_);
                rays_.push_back(ray);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan()
    {
        cloud_->points.clear();
        auto startTime=std::chrono::steady_clock::now();

        for(Ray ray:rays_)
        {
            ray.rayCast(cars_, minDistance_, maxDistance_, cloud_, groundSlope_,sderr_);
        }

        auto endTime=std::chrono::steady_clock::now();
        auto elapsedTime=std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
        cout<<"Ray casting took "<<elapsedTime.count()<<" milliseconds"<<endl;

        cloud_->width=cloud_->points.size();
        cloud_->height=1;

        return cloud_;
    }

    ~Lidar()
    {

    }

};

#endif