import sys

sys.path.append("C:/Users/schorrl/Documents/GitHub/DCAM560/DCAM560-API/DCAM560")
from math import *

import matplotlib.pyplot as plt
import open3d as o3d

from src.API.Vzense_api_560 import *

camera = VzenseTofCam()
filename = "save/point.txt"
filename_tf = "save/point_tf.txt"
filename_filtered = "save/point_filtered.xyz"
rx,ry,rz = -2.6423,0.0944,1.60133 #rad
d = np.array([[-232.9371],[158.59636],[851.17828]]) #mm

new = False
if new:
    camera.init()
    camera.set_data_mode(DataMode.Depth_RGB)
    camera.set_mapper(Sensor.RGB,True)
    camera.set_depth_range(Range.Mid)
    camera.set_depth_distortion_correction(True)
    camera.set_depth_frame(Range.Mid)
    camera.set_compute_depth_correction(True)
    camera.set_threshold(80)

    while True:
        frameready = camera.read_frame()
        if frameready and frameready.mappedRGB:      
            frame = camera.get_frame(Frame.MappedRGB)
            mappedrgb = camera.gen_image(frame,Frame.RGB)
            break

    img_gray = cv2.cvtColor(mappedrgb,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(img_gray,160,255,cv2.THRESH_BINARY)
    polcont_m = np.zeros(thresh.shape[:2], dtype="uint8") * 255
    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
    max_cont = max(contours, key=cv2.contourArea)
    eps = .01*cv2.arcLength(max_cont,True)
    approx = cv2.approxPolyDP(max_cont,eps,True)
    cv2.drawContours(polcont_m,[approx],0,(255,255,255),-1)
    polcont_m = (polcont_m/255*65535).astype(np.uint16)
    cv2.destroyAllWindows()

    camera.set_threshold()
    frameready = camera.read_frame()
    if frameready and frameready.depth:      
        frame = camera.get_frame(Frame.Depth)
        framec = POINTER(c_uint8)
        fw,fh = frame.width,frame.height
        frametmp = np.ctypeslib.as_array(frame.pFrameData, (1, 2*fw*fh))
        frametmp.dtype = np.uint16
        frametmp.shape = (fh, fw)
        frame_msked = np.bitwise_and(frametmp,polcont_m)
        frame.pFrameData = frame_msked.ctypes.data_as(framec)
        pointlist = camera.convert_to_world_vector(frame)
        filename = "save/point.txt"

        fw = open(filename,"w")

        for i in range(frame.width*frame.height):
            if pointlist[i].z!=0 and pointlist[i].z!=65535 and pointlist[i].z < 1500:
                fw.write("{0} {1} {2}\n".format(pointlist[i].x,pointlist[i].y,pointlist[i].z))

        fw.close()
        print("Successfully saved")
    camera.stop_stream() 
    camera.close() 

    cX,sX,cY,sY,cZ,sZ = float(cos(rx)),float(sin(rx)),float(cos(ry)),float(sin(ry)),float(cos(rz)),float(sin(rz))

    rX = np.array([[1,0,0],[0,cX,-sX],[0,sX,cX]])
    rY = np.array([[cY,0,sY],[0,1,0],[-sY,0,cY]])
    rZ = np.array([[cZ,-sZ,0],[sZ,cZ,0],[0,0,1]])

    r_rot = np.matmul(rZ,np.matmul(rY,rX))#.transpose()

    with open(filename,"r") as fr:
        get_all = fr.readlines()
        fr.close()

    with open(filename_tf,"w") as fw:
        for line in get_all:
            st = line.strip("\n").split(" ")
            f = np.array([[float(st[0])],[float(st[1])],[float(st[2])]])
            tf = np.matmul(r_rot,f) + d
            tfl = tf.flatten()
            fw.write("{0} {1} {2}\n".format(tfl[0],tfl[1],tfl[2]))


cloud = o3d.io.read_point_cloud(filename_tf,format='xyz')
origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0,0,0])
voxel_down_pcd = cloud.voxel_down_sample(voxel_size=0.01)

cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=5, radius=7)
filteredr = voxel_down_pcd.select_by_index(ind)
cl, ind = filteredr.remove_statistical_outlier(nb_neighbors=45,std_ratio=3)
filtereds = filteredr.select_by_index(ind)

o3d.io.write_point_cloud(filename_filtered,filtereds,write_ascii=True)

x,y,z = np.loadtxt(filename_filtered,skiprows=1, delimiter=' ', unpack=True)
pcd = np.column_stack((x,y,z))
mask = z>np.mean(z)
pcd_mean = pcd[z>(np.mean(z) - 20)]
np.savetxt(filename_filtered,pcd_mean,delimiter = " ",newline = "\n")

cloud = o3d.io.read_point_cloud(filename_filtered,format='xyz')
origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0,0,0])


#print(np.asarray(points[0][0]))
plane_model, inliers = cloud.segment_plane(distance_threshold=0.2,
                                         ransac_n=3,
                                         num_iterations=10000)

inlier_cloud = cloud.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = cloud.select_by_index(inliers, invert=True)
obb = inlier_cloud.get_minimal_oriented_bounding_box(robust=True)
obb.color = (0, 0, 1)
points = obb.get_box_points()
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud,origin,obb])

