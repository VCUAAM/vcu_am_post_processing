import open3d as o3d
import numpy as np

cloud = o3d.geometry.PointCloud()

filename = "C:\\Users\\schorrl\\Documents\\GitHub\\vcu_am_post_processing\\am_vision\\save\\point_filtered.xyz"
cloud = o3d.io.read_point_cloud(filename,format='xyz')

origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=200, origin=[0,0,0])
pose1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[-220,-146,583])
R = pose1.get_rotation_matrix_from_xyz((np.deg2rad(-147.8),np.deg2rad(-.5),np.deg2rad(93.3)))
pose1.rotate(R, center=(-220,-146,583))
pose1.paint_uniform_color([1,.706,0])

pose2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[-28.9,55,668])
R = pose2.get_rotation_matrix_from_xyz((np.deg2rad(-129.8),np.deg2rad(.2),np.deg2rad(107)))
pose2.rotate(R, center=(-28.9,55,668))

pose3 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[-453,139,765])
R = pose3.get_rotation_matrix_from_xyz((np.deg2rad(-162.9),np.deg2rad(24.4),np.deg2rad(93.9)))
pose3.rotate(R, center=(-453,139,765))

pose4 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[-414,-386,724])
R = pose4.get_rotation_matrix_from_xyz((np.deg2rad(-164.7),np.deg2rad(-11.2),np.deg2rad(101.6)))
pose4.rotate(R, center=(-414,-386,724))

pose5 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[-95,-98.4,464])
R = pose5.get_rotation_matrix_from_xyz((np.deg2rad(-126.5),np.deg2rad(4.9),np.deg2rad(96.9)))
pose5.rotate(R, center=(-95,-98.4,464))

pose6 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[-236,-118,890])
R = pose6.get_rotation_matrix_from_xyz((np.deg2rad(-153.8),np.deg2rad(-2.3),np.deg2rad(93.2)))
pose6.rotate(R, center=(-220,-146,583))

o3d.visualization.draw_geometries([origin,cloud,pose1,pose2,pose3,pose4,pose5,pose6])