![][image1]
# 算法设计  
## 障碍物检测流程  

![][image2]    

## 直通滤波模块  
**参数设置**  
过滤轴为x轴和y轴  
x轴过滤范围[0, 50]  
y轴过滤范围[-20, 10]

![][image3]
*<font size=3>右图为原始点云 &ensp; 左图为直通滤波后的点云</font>*


## 地面分割模块    

**算法步骤**    
1 在数据中随机选择三个点设定为内群  
2 计算拟合内群的模型  
3 把其它刚才没选到的点带入刚才建立的模型中，根据距离阈值参数（DistanceThreshold）判断是否为内群  
4 记下内群点数量  
5 重复以上步骤，直至达到最大迭代次数（MaxIterations）  
6 比较哪次计算中内群点数量最多，内群点最多的那次所建的模型就是地面模型
  
**参数设置**  
距离阈值参数（DistanceThreshold）设置为0.191  
迭代次数（MaxIterations）设置为1000  

![][image4]
*<font size=3>RANSAC 地面分割示意图</font>*  
     
![][image5]
*<font size=3>分割前的点云</font>*  

![][image6]
*<font size=3>地面点云</font>*  

![][image7]  
*<font size=3>去除地面点云后的点云</font>*  

## 欧式聚类模块  
在聚类之前，先使用Kd-tree存储点云  
Kd-tree数据结构如图所示，构建Kd-tree时，随着树的深度轮流选择轴当作分割面，从点云中取出的点如果小于当前结点的值，则将它放在左子树，如果大于当前结点的值，则将它放在右子树。  
![][image8]

**算法步骤**  
![][image9]

**参数设置**  
距离阈值（ClusterToleranc) 设置为1.0  
MinClusterSize 设置为10  
MaxClusterSize 设置为2000    

![][image10]  
*<font size=3>欧式聚类效果</font>*  

# 编译（Ubuntu16.04）  
## 依赖
* CMake >=3.12
* GCC
* boost 1.58.0
* pcl (sudo apt install libpcl-dev)  
## 使用  
```
$> mkdir build && cd build
$> cmake ..
$> make
$> bash command.sh
```  

[//]:#(reference)
[image1]:./exhibition/PointCloudDetection.gif
[image2]:./exhibition/flow_chat.png
[image3]:./exhibition/pass_through.png
[image4]:./exhibition/RANSAC.png
[image5]:./exhibition/ground_segmentation_before.png  
[image6]:./exhibition/ground_sgmentation.png  
[image7]:./exhibition/ground_segmentation_after.png  
[image8]:./exhibition/Kd-tree.png
[image9]:./exhibition/euclideancluster.png  
[image10]:./exhibition/clustering.png  
