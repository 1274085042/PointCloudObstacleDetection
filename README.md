使用Pybind11将障碍物检测算法封装成Python可以调用的接口    

# How to compile on Ubuntu16.04
## Requirement   
* CMake >=3.12  
* GCC  
* boost 1.58.0
* pcl (sudo apt install libpcl-dev)
* pybind11

## How to run?
```
$> mkdir build && cd build  
$> cmake ..  
$> make  
$> python3.6  
$> import traditionalpointcloud
$> traditionalpointcloud.obstacledetection(filepath)
```
注意：**python3.6是编译pybind11使用的版本**
  

[//]:#(reference)