#pragma once

#include<iostream>
#include<math.h>
#include<vector>
#include<stdio.h>

struct Node
{
    int id_;
    Node *left_;
    Node *right_;

    Node(int setid):id_(setid),left_(nullptr),right_(nullptr)
    {}
};


template <typename PointT>
struct  kdTree
{
    Node *root;
    
    typename pcl::PointCloud<PointT>::Ptr cloud;

    kdTree(typename pcl::PointCloud<PointT>::Ptr pointcloud):root(nullptr),cloud(pointcloud)
    {}

    void insertHelper(Node **node, unsigned int depth, int id)
    //double pointer (Node **) because root was defined as Node* (node pointer) originally, and then we pass the memory address
    //node：当前节点     id:要插入的点云中的点
    {
        /*Create a  new ndoe and places it in the right positon int the k-d tree*/

        if(*node =nullptr)
        //dereferencing to get value 
        //ternimate when it hits a null node
        {
            *node=new Node(id);             //pointing root pointer to new data
            std::cout<<"Addd id to tree "<<std::endl;
            std::cout<<"Depth: "<<depth<<std::endl;
        }
        else //traverse
        {
            unsigned int variable=depth%3;   //3D, always 0, 1 or 2

            if (variable==0)     //point[0]: x value
            {
                if (cloud->points[id].x<cloud->points[(*node)->id_].x)
                insertHelper(&((*node)->left_),depth+1,id);
                else
                insertHelper(&((*node)->right_),depth+1,id);
                
            }
            else if(variable==1)
            {
                 if(cloud->points[id].y<cloud->points[(*node)->id_].y)
                 insertHelper(&((*node)->left_),depth+1,id);
                 else
                 insertHelper(&((*node)->right_),depth+1,id);
            }
            else if(variable==2)
            {
                if(cloud->points[id].z<cloud->points[(*node)->id_].z)
                insertHelper(&((*node)->left_),depth+1,id);
                else
                insertHelper(&((*node)->right_),depth+1,id);
            }
            
        }
    }


    void insertPointIndex(int id)
    {
        insertHelper(&root,0,id);     //passint the address of root
    }

    void searchHelper(int id, Node **node, unsigned int depth, float distanceTol, std::vector<int> &nearbyPointIds)
    {
        //*node代表要检查的点
        //id簇中心的点
        if(*node!=nullptr)
        {
            //Checking if point in current node is inside the target box
            if((cloud->points[(*node)->id_].x>=(cloud->points[id].x-distanceTol) && cloud->points[(*node)->id_].x<=(cloud->points[id].x+distanceTol))
            && (cloud->points[(*node)->id_].y>=(cloud->points[id].y-distanceTol) && cloud->points[(*node)->id_].y<=(cloud->points[id].y+distanceTol))
            && (cloud->points[(*node)->id_].z>=(cloud->points[id].z-distanceTol) && cloud->points[(*node)->id_].z<=(cloud->points[id].z+distanceTol)) )
            {

                //Finding distance between 3D points (node x,y,z and target x,y,z)

                float distance=sqrt(pow((cloud->points[(*node)->id_].x-cloud->points[id].x),2.0)+pow((cloud->points[(*node)->id_].y-cloud->points[id].y),2.0)+pow((cloud->points[(*node)->id_].z-cloud->points[id].z),2.0));

                if (distance<=distanceTol)
                {
                    std::cout<<"Distance within distance tolerance threshold. Adding index to cluster ..."<<std::endl;
                    std::cout<<"\tDist: "<<distance;
                    std::cout<<"\tTole: "<<distanceTol<<std::endl;
                    nearbyPointIds.push_back((*node)->id_);
                }
            }

            //Checking box boundary to see where to move down next in the tree (left or right)
            //找box的边界点
            unsigned int varToCompare=depth%3;
            if(varToCompare==0)   //x dimension
            {
                if((cloud->points[id].x-distanceTol) < cloud->points[(*node)->id_].x)  //if left boundary of box is < node's x|y|z value, that box is in the left region
                searchHelper(id,&((*node)->left_),depth+1,distanceTol,nearbyPointIds);
                if((cloud->points[id].x+distanceTol) > cloud->points[(*node)->id_].x)
                searchHelper(id,&((*node)->right_),depth+1,distanceTol,nearbyPointIds);
            }
            else if(varToCompare==1)   //y dimension
            {
                if((cloud->points[id].y-distanceTol) < cloud->points[(*node)->id_].y)  //if left boundary of box is < node's x|y|z value, that box is in the left region
                searchHelper(id,&((*node)->left_),depth+1,distanceTol,nearbyPointIds);
                if((cloud->points[id].y+distanceTol) > cloud->points[(*node)->id_].y)
                searchHelper(id,&((*node)->right_),depth+1,distanceTol,nearbyPointIds);
            }
            else if(varToCompare==2)   //z dimension
            {
                if((cloud->points[id].z-distanceTol) < cloud->points[(*node)->id_].z)  //if left boundary of box is < node's x|y|z value, that box is in the left region
                searchHelper(id,&((*node)->left_),depth+1,distanceTol,nearbyPointIds);
                if((cloud->points[id].z+distanceTol) > cloud->points[(*node)->id_].z)
                searchHelper(id,&((*node)->right_),depth+1,distanceTol,nearbyPointIds);
            }
        }
    }

    std::vector<int> search(int id, float distanceTol)
    {
        std::vector<int> nearbyPointIds;
        searchHelper(id, &root, 0, distanceTol, nearbyPointIds);
        return nearbyPointIds;
    }

};


