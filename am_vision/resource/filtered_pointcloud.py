from math import *
import open3d as o3d

from dcam560.Vzense_api_560 import *

class PointcloudCapture:
    
    def __init__(self):
        self.camera = VzenseTofCam()
        self.rotation = np.array([-2.64847,0.02875,1.52433])
        self.position = np.array([[-383.27179],[-105.2708],[727.15753]]) #mm
    
    def find_table_coords(self):
        rpy = self.rotation
        p1 = time.time()
        mask_cont = self.capture_image()
        points = self.pointcloud_transform(mask_cont,rpy)
        table_coords = self.pointcloud_filtering(points,p1)

        return table_coords

    def capture_image(self):
        self.camera.init()
        self.camera.set_data_mode(DataMode.Depth_IR_RGB)
        self.camera.set_depth_range(Range.Mid)
        #self.camera.set_depth_distortion_correction(True)
        #self.camera.set_compute_depth_correction(True)

        while True:
            frameready = self.camera.read_frame()
            if frameready and frameready.ir:      
                frame = self.camera.get_frame(Frame.IR)
                ir = self.camera.gen_image(frame,Frame.IR)
                break
        while True:
            frameready = self.camera.read_frame()
            if frameready and frameready.rgb:      
                frame = self.camera.get_frame(Frame.RGB)
                rgb = self.camera.gen_image(frame,Frame.RGB)
                break
        ret,thresh = cv2.threshold(ir,67,255,cv2.THRESH_BINARY)
        cv2.imwrite('am_vision/save/ir.png',ir)
        cv2.imwrite('am_vision/save/rgb_pap.png',rgb)
        mask_cont = np.zeros(thresh.shape[:2], dtype="uint8") * 255
        contours,hierarchy = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        max_cont = max(contours, key=cv2.contourArea)
        eps = .01*cv2.arcLength(max_cont,True)
        approx = cv2.approxPolyDP(max_cont,eps,True)

        cv2.drawContours(mask_cont,[approx],0,(255,255,255),-1)
        mask_cont = (mask_cont/255*65535).astype(np.uint16)
        cv2.imwrite('am_vision/save/maskcont.png',mask_cont)
        cv2.destroyAllWindows()

        return mask_cont

    def pointcloud_transform(self,mask_cont,rpy):
        while True:
            frameready = self.camera.read_frame()
            if frameready and frameready.depth:      
                frame = self.camera.get_frame(Frame.Depth)
                depth = self.camera.gen_image(frame,Frame.Depth)
                pointlistraw = self.camera.convert_to_world_vector(frame)
                cv2.imwrite('am_vision/save/depth.png',depth)
                framec = POINTER(c_uint8)
                fw,fh = frame.width,frame.height
                frametmp = np.ctypeslib.as_array(frame.pFrameData, (1, 2*fw*fh))
                frametmp.dtype = np.uint16
                frametmp.shape = (fh, fw)
                frame_msked = np.bitwise_and(frametmp,mask_cont)
                frame.pFrameData = frame_msked.ctypes.data_as(framec)
                maskeddepth = self.camera.gen_image(frame,Frame.Depth)
                cv2.imwrite('am_vision/save/maskeddepth.png',maskeddepth)
                pointlist = self.camera.convert_to_world_vector(frame)
            self.camera.stop_stream() 
            self.camera.close() 

            rx,ry,rz = rpy
            cX,sX,cY,sY,cZ,sZ = float(cos(rx)),float(sin(rx)),float(cos(ry)),float(sin(ry)),float(cos(rz)),float(sin(rz))

            rX = np.array([[1,0,0],[0,cX,-sX],[0,sX,cX]])
            rY = np.array([[cY,0,sY],[0,1,0],[-sY,0,cY]])
            rZ = np.array([[cZ,-sZ,0],[sZ,cZ,0],[0,0,1]])

            r_rot = np.matmul(rZ,np.matmul(rY,rX))

            points = []
            #filename = "am_vision\save\pointraw.txt"

            #file = open(filename,"w")
            #for i in pointlistraw:
            #    if i.z != 0 and i.z != 65535 and i.z < 1500:
            #        point = [[i.x],[i.y],[i.z]]
            #        tf = np.matmul(r_rot,point) + self.position
            #@        tfl = tf.flatten()
            #        file.write("{0} {1} {2}\n".format(tfl[0],tfl[1],tfl[2]))
            #file.close()

            for i in pointlist:
                if i.z != 0 and i.z != 65535 and i.z < 1500:
                    point = [[i.x],[i.y],[i.z]]
                    tf = np.matmul(r_rot,point) + self.position
                    tfl = tf.flatten()
                    points.append([tfl[0],tfl[1],tfl[2]])
            
            
            break
        
        return points

    def pointcloud_filtering(self,points,p1):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.asarray(points))
        origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0,0,0])
        #o3d.visualization.draw_geometries([cloud,origin],
			#front = [ 0.6390142761762605, 0.36699851382219117, 0.67599766693031793 ],
			#lookat = [ -681.46768279149137, -297.47970701878046, 93.09798926511661 ],
			#up = [-0.68593177749272549, -0.12581045754473924, 0.7167072801346821 ],
			#zoom = 0.33999999999999964)
        cl, ind = cloud.remove_radius_outlier(nb_points=5, radius=7)
        filteredr = cloud.select_by_index(ind)
        cl, ind = filteredr.remove_statistical_outlier(nb_neighbors=45,std_ratio=3)
        filtereds = filteredr.select_by_index(ind)

        pcd = np.asarray(filtereds.points)
        z = pcd[:,2]
        pcd_mean = pcd[z>(np.mean(z) - 50)]

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pcd_mean)
        downpcd = cloud.voxel_down_sample(voxel_size=5)
        #plane_model, inliers = cloud.segment_plane(distance_threshold=0.15,ransac_n=3,num_iterations=10000)

        #inlier_cloud = cloud.select_by_index(inliers)
        #inlier_cloud.paint_uniform_color([5/255, 195/255, 221/255])

        obb = cloud.get_minimal_oriented_bounding_box(robust=True)
        obb.color = (0, 0, 0)
        table_coords = np.asarray(obb.get_box_points()).tolist()
        p2 = time.time()
        print(p2-p1,"ms")
        o3d.visualization.draw_geometries([downpcd,obb,origin],
			front = [ 0.6390142761762605, 0.36699851382219117, 0.67599766693031793 ],
			lookat = [ -681.46768279149137, -297.47970701878046, 93.09798926511661 ],
			up = [-0.68593177749272549, -0.12581045754473924, 0.7167072801346821 ],
			zoom = 0.33999999999999964)
        
        return table_coords
    
if __name__ == "__main__":
    node = PointcloudCapture()
    points = node.find_table_coords()
    print(points)