#author : nan dong
import os
import numpy as np
import open3d as o3d

pcd_root = './data_1/'

for _,_,file in os.walk(pcd_root):
    f = file
    
out_root = './pcd_out/'

c = 0
for i in f:
    print(c)
    pcd =o3d.io.read_point_cloud(pcd_root+i)
    c += 1
    o3d.io.write_point_cloud(out_root+str(c)+'.pcd',pcd,write_ascii= True)