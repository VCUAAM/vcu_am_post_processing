import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from am_pp_interfaces.action import PointcloudCapture
from std_msgs.msg import Float64MultiArray
from math import *
import open3d as o3d

from dcam560.Vzense_api_560 import *

class PointcloudCaptureServer(Node):
    
    def __init__(self):
        self.camera = VzenseTofCam()
        self.rotation = np.array([0.6985299535101926,0.6881874909198978,-0.05711937237981356,-0.18759333327535896])
        self.translation = np.array([[-232.9371],[158.59636],[851.17828]]) #mm
        
        super().__init__('pointcloud_action_server')
        self._action_server = ActionServer(
            self,
            PointcloudCapture,
            'pointcloudcapture',
            self.execute_callback)

    def execute_callback(self,goal_handle):
        self.get_logger().info('Executing goal...')

        rpy = self.euler_from_quaternion(self.rotation)
        mask_cont = self.capture_image()
        points = self.pointcloud_transform(mask_cont,rpy)
        table_coords = self.pointcloud_filtering(points)
        goal_handle.succeed()
        result = PointcloudCapture.Result()
        result.table_coords = table_coords

        return result
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    def euler_from_quaternion(self,quaternions):
        x,y,z,w = quaternions
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

    def capture_image(self):
        self.camera.init()
        self.camera.set_data_mode(DataMode.Depth_IR_RGB)
        self.camera.set_depth_range(Range.Mid)
        self.camera.set_depth_distortion_correction(True)
        self.camera.set_compute_depth_correction(True)

        while True:
            frameready = self.camera.read_frame()
            if frameready and frameready.ir:      
                frame = self.camera.get_frame(Frame.IR)
                ir = self.camera.gen_image(frame,Frame.IR)
                break
        
        ret,thresh = cv2.threshold(ir,45,255,cv2.THRESH_BINARY)
        mask_cont = np.zeros(thresh.shape[:2], dtype="uint8") * 255
        contours,hierarchy = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        max_cont = max(contours, key=cv2.contourArea)
        eps = .01*cv2.arcLength(max_cont,True)
        approx = cv2.approxPolyDP(max_cont,eps,True)

        cv2.drawContours(mask_cont,[approx],0,(255,255,255),-1)
        mask_cont = (mask_cont/255*65535).astype(np.uint16)
        cv2.destroyAllWindows()

        return mask_cont

    def pointcloud_transform(self,mask_cont,rpy):
        while True:
            frameready = self.camera.read_frame()
            if frameready and frameready.depth:      
                frame = self.camera.get_frame(Frame.Depth)
                framec = POINTER(c_uint8)
                fw,fh = frame.width,frame.height
                frametmp = np.ctypeslib.as_array(frame.pFrameData, (1, 2*fw*fh))
                frametmp.dtype = np.uint16
                frametmp.shape = (fh, fw)
                frame_msked = np.bitwise_and(frametmp,mask_cont)
                frame.pFrameData = frame_msked.ctypes.data_as(framec)
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
            
            for i in pointlist:
                if i.z != 0 and i.z != 65535 and i.z < 1500:
                    point = [[i.x],[i.y],[i.z]]
                    tf = np.matmul(r_rot,point) + self.translation
                    tfl = tf.flatten()
                    points.append([tfl[0],tfl[1],tfl[2]])
            break
        
        return points

    def pointcloud_filtering(self,points):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.asarray(points))
        origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0,0,0])

        cl, ind = cloud.remove_radius_outlier(nb_points=5, radius=7)
        filteredr = cloud.select_by_index(ind)
        cl, ind = filteredr.remove_statistical_outlier(nb_neighbors=45,std_ratio=3)
        filtereds = filteredr.select_by_index(ind)

        pcd = np.asarray(filtereds.points)
        z = pcd[:,2]
        pcd_mean = pcd[z>(np.mean(z) - 20)]

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pcd_mean)
        plane_model, inliers = cloud.segment_plane(distance_threshold=0.2,
                                                ransac_n=3,
                                                num_iterations=10000)

        inlier_cloud = cloud.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        obb = inlier_cloud.get_minimal_oriented_bounding_box(robust=True)
        obb.color = (0, 0, 1)
        table_coords = np.asarray(obb.get_box_points()).tolist()

        return table_coords

def main():
    rclpy.init()

    node = PointcloudCaptureServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()