import numpy

# dis1_raw=numpy.sqrt(5.96727**2+17.2654**2)
# dis1_rota=numpy.sqrt(5.54426**2+17.2654**2)
#
# dis2_raw=numpy.sqrt(2.48309**2 + 23.6847**2)
# dis2_rota=numpy.sqrt(2.20864**2 + 23.6847**2)

pcd1_raw_point = [5.96727,17.2654]
pcd1_rota_point= [5.54426,17.2654]
pcd1_raw_dis=numpy.sqrt(pcd1_raw_point[0]**2 + pcd1_raw_point[1]**2)
pcd1_rota_dis=numpy.sqrt(pcd1_rota_point[0]**2 + pcd1_rota_point[1]**2)
print("1_raw_distanc:",pcd1_raw_dis)
print("1_rota_distanc:",pcd1_rota_dis)

pcd2_raw_point = [2.48309,23.6847]
pcd2_rota_point= [2.20864,23.6847]
pcd2_raw_dis=numpy.sqrt(pcd2_raw_point[0]**2 + pcd2_raw_point[1]**2)
pcd2_rota_dis=numpy.sqrt(pcd2_rota_point[0]**2 + pcd2_rota_point[1]**2)
print("2_raw_distanc:",pcd2_raw_dis)
print("2_rota_distanc:",pcd2_rota_dis)

pcd3_raw_point = [-6.19183,31.8843]
pcd3_rota_point= [-6.02965,31.8843]
pcd3_raw_dis=numpy.sqrt(pcd3_raw_point[0]**2 + pcd3_raw_point[1]**2)
pcd3_rota_dis=numpy.sqrt(pcd3_rota_point[0]**2 + pcd3_rota_point[1]**2)
print("3_raw_distanc:",pcd3_raw_dis)
print("3_rota_distanc:",pcd3_rota_dis)

pcd4_raw_point = [-6.46596,39.2257]
pcd4_rota_point= [-6.4116,39.2257]
pcd4_raw_dis=numpy.sqrt(pcd4_raw_point[0]**2 + pcd4_raw_point[1]**2)
pcd4_rota_dis=numpy.sqrt(pcd4_rota_point[0]**2 + pcd4_rota_point[1]**2)
print("4_raw_distanc:",pcd4_raw_dis)
print("4_rota_distanc:",pcd4_rota_dis)