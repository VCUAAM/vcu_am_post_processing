import sys

import cv2
import matplotlib.pyplot as plt
import open3d as o3d

sys.path.append("C:/Users/schorrl/Documents/GitHub/DCAM560/DCAM560-API/DCAM560")
from src.API.Vzense_api_560 import *

camera = VzenseTofCam()

camera.init()
camera.set_data_mode(DataMode.Depth_RGB)
camera.set_mapper(Sensor.RGB,True)
camera.set_depth_range(Range.Mid)
camera.set_RGB_distortion_correction(True)
camera.set_depth_distortion_correction(True)
camera.set_threshold(80)

while True:
    frameready = camera.read_frame()
    if frameready and frameready.mappedRGB:      
        frame = camera.get_frame(Frame.MappedRGB)
        mappedrgb = camera.gen_image(frame,Frame.RGB)
        break

img_gray = cv2.cvtColor(mappedrgb,cv2.COLOR_BGR2GRAY)
ret,thresh = cv2.threshold(img_gray,160,255,cv2.THRESH_BINARY)
polcont_c = np.ones(thresh.shape[:2], dtype="uint8") * 255
polcont_m = np.zeros(thresh.shape[:2], dtype="uint8") * 255
contours,hierarchy = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
max_cont = max(contours, key=cv2.contourArea)
eps = .01*cv2.arcLength(max_cont,True)
approx = cv2.approxPolyDP(max_cont,eps,True)
polcont_c = cv2.cvtColor(polcont_c,cv2.COLOR_GRAY2BGR)
polcont_m = cv2.cvtColor(polcont_m,cv2.COLOR_GRAY2BGR)

for i in range(len(approx)):
    cv2.putText(polcont_c,str(tuple(approx[i][0])),approx[i][0],cv2.FONT_HERSHEY_COMPLEX,.5,(0,0,255))
    
cv2.drawContours(polcont_c,[approx],-1,(0,0,0),2)
cv2.drawContours(polcont_m,[approx],0,(255,255,255),-1)
cv2.imwrite('save/polcont.png',polcont_c)
cv2.destroyAllWindows()

camera.set_threshold()
frameready = camera.read_frame()


if frameready and frameready.depth:      
    frame_d = camera.get_frame(Frame.Depth)
    depth_max,value_min,max_range = camera.get_measuring_range(False)
    polcont_m = cv2.cvtColor(polcont_m,cv2.COLOR_BGR2GRAY)
    polcont_m = (polcont_m/255*65535).astype(np.uint16)
    framec = POINTER(c_uint8)
    fw,fh = frame_d.width,frame_d.height
    frametmp = np.ctypeslib.as_array(frame_d.pFrameData, (1, 2*fw*fh))
    frametmp.dtype = np.uint16
    frametmp.shape = (fh, fw)
    frame_msked = np.bitwise_and(frametmp,polcont_m)
    frame_d.pFrameData = frame_msked.ctypes.data_as(framec)
    #print(set(frame_msked.flatten()))
    cv2.imwrite("save/maskdep.png",frame_msked)
    pointlist = camera.convert_to_world_vector(frame_d)
    folder = os.getcwd() + "/save"
    filename = folder + "/point_filtered.txt"

    if not os.path.exists(folder):
        print("Creating folder")
        os.makedirs(folder)

    file = open(filename,"w")

    for i in range(frame.width*frame.height):
        if pointlist[i].z!=0 and pointlist[i].z!=65535 and pointlist[i].z < 1500:
            file.write("{0} {1} {2}\n".format(pointlist[i].x,pointlist[i].y,pointlist[i].z))

    file.close()
    print("Successfully saved")

cloud = o3d.io.read_point_cloud(filename,format='xyz')
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0,0,0])
voxel_down_pcd = cloud.voxel_down_sample(voxel_size=0.02)
#o3d.visualization.draw_geometries([voxel_down_pcd])

cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=6, radius=5)
filteredr = voxel_down_pcd.select_by_index(ind)
cl, ind = filteredr.remove_statistical_outlier(nb_neighbors=20,std_ratio=2)
filtereds = filteredr.select_by_index(ind)

obb = filtereds.get_oriented_bounding_box()
obb.color = (0, 0, 1)
o3d.visualization.draw_geometries([filtereds,mesh_frame,obb])

\
camera.stop_stream() 
camera.close()