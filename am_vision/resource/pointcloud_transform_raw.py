from math import *

import matplotlib.pyplot as plt
import open3d as o3d
from std_msgs.msg import Float64MultiArray
from dcam560.Vzense_api_560 import *

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return [roll_x, pitch_y, yaw_z]

def main():
    camera = VzenseTofCam()
    filename_tf = "/home/rosnuc/workspace/vcu_am_post_processing/src/am_vision/save/point_tf.txt"
    filename_filtered = "/home/rosnuc/workspace/vcu_am_post_processing/src/am_vision/save/point_filtered.xyz"
    #rx,ry,rz = -1.96 #rad
    x,y,z,w = 0.6985299535101926,0.6881874909198978,-0.05711937237981356,-0.18759333327535896
    d = np.array([[-296.19],
                  [-96.55],
                  [815.40]]) #mm
    rx,ry,rz = euler_from_quaternion(x,y,z,w)

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
        camera.stop_stream() 
        camera.close() 
        cX,sX,cY,sY,cZ,sZ = float(cos(rx)),float(sin(rx)),float(cos(ry)),float(sin(ry)),float(cos(rz)),float(sin(rz))

        rX = np.array([[1,0,0],[0,cX,-sX],[0,sX,cX]])
        rY = np.array([[cY,0,sY],[0,1,0],[-sY,0,cY]])
        rZ = np.array([[cZ,-sZ,0],[sZ,cZ,0],[0,0,1]])

        r_rot = np.matmul(rZ,np.matmul(rY,rX))#.transpose()

        points = []
        
        for i in pointlist:
            if i.z != 0 and i.z != 65535 and i.z < 1500:
                point = [[i.x],[i.y],[i.z]]
                tf = np.matmul(r_rot,point) + d
                tfl = tf.flatten()
                points.append([tfl[0],tfl[1],tfl[2]])

        np.savetxt(filename_tf,points,delimiter = " ",newline = "\n")
    cloud = o3d.io.read_point_cloud(filename_tf,format='xyz')
    #cloud = o3d.geometry.PointCloud()
    #cloud.points = o3d.utility.Vector3dVector(np.asarray(points))
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0,0,0])
    voxel_down_pcd = cloud.voxel_down_sample(voxel_size=0.01)

    cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=5, radius=7)
    filteredr = voxel_down_pcd.select_by_index(ind)
    cl, ind = filteredr.remove_statistical_outlier(nb_neighbors=45,std_ratio=3)
    filtereds = filteredr.select_by_index(ind)

    pcd = np.asarray(filtereds.points)
    z = pcd[:,2]
    pcd_mean = pcd[z>(np.mean(z) - 20)]
    np.savetxt(filename_filtered,pcd_mean,delimiter = " ",newline = "\n")

    #cloud = o3d.io.read_point_cloud(filename_filtered,format='xyz')
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(pcd_mean)
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0,0,0])


    plane_model, inliers = cloud.segment_plane(distance_threshold=0.2,
                                            ransac_n=3,
                                            num_iterations=10000)

    inlier_cloud = cloud.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = cloud.select_by_index(inliers, invert=True)
    obb = inlier_cloud.get_minimal_oriented_bounding_box(robust=True)
    obb.color = (0, 0, 1)
    table_coords = np.asarray(obb.get_box_points())
    table_coords = (table_coords.flatten()).tolist()
    #o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud,origin,obb])

if __name__ == "__main__":
    main()