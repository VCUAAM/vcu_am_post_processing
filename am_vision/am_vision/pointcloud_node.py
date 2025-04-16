import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from am_pp_interfaces.msg import FloatArrayAM,FloatListAM
from am_pp_interfaces.action import PointcloudCapture
from geometry_msgs.msg import Pose
from math import *
import open3d as o3d

from dcam560.Vzense_api_560 import *

class PointcloudCaptureNode(Node):
    
    def __init__(self):
        # Initializing the camera
        self.camera = VzenseTofCam()
        #self.rotation = np.array([0.6985299535101926,0.6881874909198978,-0.05711937237981356,-0.18759333327535896])
        #self.position = np.array([[-232.9371],[158.59636],[851.17828]]) #mm

        # Defining the orientation and position arrays to be used for camera->robot frame conversion
        self.orientation = np.array([])
        self.position = np.array([])

        # Starting the action server for the pointcloud to grab the pose of the robot
        super().__init__('pointcloud_action_server')
        self.subscription = self.create_subscription(
            Pose,
            'current_pose',
            self.get_current_pose,
            10)

        self._action_server = ActionServer(
            self,
            PointcloudCapture,
            'pointcloudcapture',
            self.action_callback)

    """
    Function that will collect the orientation and position from a given odometry input
    """
    def get_current_pose(self,odom):
        self.orientation = np.array([odom.orientation.x,
                                  odom.orientation.y,
                                  odom.orientation.z,
                                  odom.orientation.z])
        self.position = np.array([[odom.position.x],
                                  [odom.position.y],
                                  [odom.position.z]])

    """ 
    Setting the sequence of functions for the action server to execute
    """
    def action_callback(self,goal_handle):
        print(self.position)
        self.get_logger().info('Calculating table vertices...')

        # Converting the quaternion orientation into Euler
        rpy = self.euler_from_quaternion(self.orientation)

        # Capture an IR image to create a mask
        mask_cont = self.capture_image()

        # Applying the mask to a depth image, converting the depth image to a 3D pointcloud, and transforming the camera frame to robot frame
        points = self.pointcloud_transform(mask_cont,rpy)

        # Pointcloud post-processing and finding the vertices
        table_coords = self.pointcloud_filtering(points)

        # Converting the table coordinates into usable data to transmit over a topic
        tc = FloatArrayAM()
        for point in table_coords:
            fl = FloatListAM()
            fl.x,fl.y,fl.z = point
            tc.points.append(fl)
        goal_handle.succeed()
        result = PointcloudCapture.Result()
        result.table_coords = tc
        self.get_logger().info('Table vertices calculated')

        return result
    
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    def euler_from_quaternion(self,orientation):
        x,y,z,w = orientation
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
    
    """
    Function to capture the image and identify the largest contour
    """
    def capture_image(self):
        # Initializing the camera settings
        self.camera.init()
        self.camera.set_data_mode(DataMode.Depth_IR_RGB)
        self.camera.set_depth_range(Range.Mid)
        self.camera.set_depth_distortion_correction(True)
        self.camera.set_compute_depth_correction(True)

        # Capture an IR image if there is a frame to read
        while True:
            frameready = self.camera.read_frame()
            if frameready and frameready.ir:      
                frame = self.camera.get_frame(Frame.IR)
                ir = self.camera.gen_image(frame,Frame.IR)
                break
            
        # Setting the threshold for the IR image
        ret,thresh = cv2.threshold(ir,45,255,cv2.THRESH_BINARY)

        # Initializing an empty matrix 
        mask_cont = np.zeros(thresh.shape[:2], dtype="uint8") * 255

        # Finding the contours in the image
        contours,hierarchy = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)

        # Selecting the contour with the largest area
        max_cont = max(contours, key=cv2.contourArea)

        # Turning the contour into a polygonal approximation and setting how fine the lines are
        eps = .01*cv2.arcLength(max_cont,True)
        approx = cv2.approxPolyDP(max_cont,eps,True)

        # Drawing the polygon into the empty matrix
        cv2.drawContours(mask_cont,[approx],0,(255,255,255),-1)

        #Changing the matrix to work well with the range of the depth parameter
        mask_cont = (mask_cont/255*65535).astype(np.uint16)

        # Closing out opencv
        cv2.destroyAllWindows()

        return mask_cont

    """"
    Applying the mask to a depth image, converting the depth image to a 3D pointcloud, and then transforming from camera frame to robot frame
    'mask_cont' is the mask generated in 'capture_image', and is a matrix with all points not in the foreground set to 0
    'rpy' is the current orientation of the camera in roll-pitch-yaw format
    """
    def pointcloud_transform(self,mask_cont,rpy):
        while True:
            # Reads the frame of the camera
            frameready = self.camera.read_frame()
            
            # Captures a depth image if the frame exists
            if frameready and frameready.depth:
                frame = self.camera.get_frame(Frame.Depth)

                # Setting a pointer to inject the masked image back into the original object
                framec = POINTER(c_uint8)

                # Obtaining the width and height of the depth frame
                fw,fh = frame.width,frame.height

                # Turning the depth image into an array
                frametmp = np.ctypeslib.as_array(frame.pFrameData, (1, 2*fw*fh))

                # Changing the datatype and size for the temporary image to what is required to inject back into the frame object
                frametmp.dtype = np.uint16
                frametmp.shape = (fh, fw)

                # Applying the mask to the generated frame
                frame_msked = np.bitwise_and(frametmp,mask_cont)

                # Injecting the altered (masked) depth image back into the frame
                frame.pFrameData = frame_msked.ctypes.data_as(framec)

                # Converts the depth image to a 3D pointcloud in the camera frame
                pointlist = self.camera.convert_to_world_vector(frame)

            # Stopping communication with the camera
            self.camera.stop_stream() 
            self.camera.close() 

            # Defining roll, pitch, and yaw into variables
            rx,ry,rz = rpy

            # Doing preliminary computations to make syntax easier for transformation matrix
            cX,sX,cY,sY,cZ,sZ = float(cos(rx)),float(sin(rx)),float(cos(ry)),float(sin(ry)),float(cos(rz)),float(sin(rz))

            # Constructing transformation matrices to convert from camera frame to robot frame
            rX = np.array([[1,0,0],[0,cX,-sX],[0,sX,cX]])
            rY = np.array([[cY,0,sY],[0,1,0],[-sY,0,cY]])
            rZ = np.array([[cZ,-sZ,0],[sZ,cZ,0],[0,0,1]])

            # Multipling the individual transformation matrices to get a total transformation matrix
            r_rot = np.matmul(rZ,np.matmul(rY,rX))

            # Creating an empty list to populate with the points in the cloud
            points = []
            
            for i in pointlist:
                # Automatically chopping off empty data points or data points under a certain threshold
                if i.z != 0 and i.z != 65535 and i.z < 1500:
                    point = [[i.x],[i.y],[i.z]]

                    # Applying transformation matrix and positional offset
                    tf = np.matmul(r_rot,point) + self.position
                    
                    # Flattening matrix to single array
                    tfl = tf.flatten()

                    # Appending point to the pointcloud
                    points.append([tfl[0],tfl[1],tfl[2]])
            break
        
        return points

    """
    Function for post-processing a given point cloud
    'points' is the list of (x,y,z) coordinates that make up the pointcloud
    """
    def pointcloud_filtering(self,points):
        # Creating an instance of a cloud within the open3d library
        cloud = o3d.geometry.PointCloud()

        # Setting the points in the cloud equal to the points captured from the camera
        cloud.points = o3d.utility.Vector3dVector(np.asarray(points))

        # Defining the origin for the pointcloud
        origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0,0,0])

        # Applying a radial outlier filter to the pointcloud to remove points outside of a certain radius of other points
        cl, ind = cloud.remove_radius_outlier(nb_points=5, radius=7)

        # Creating a new cloud based on the filtered points
        filteredr = cloud.select_by_index(ind)

        # Applying a statistical outlier filter to remove points outside of three standard deviations
        cl, ind = filteredr.remove_statistical_outlier(nb_neighbors=45,std_ratio=3)
        filtereds = filteredr.select_by_index(ind)

        # Creating a numpy array based on the filtered points selected
        pcd = np.asarray(filtereds.points)

        # Isolating the height of the pointcloud
        z = pcd[:,2]

        # Calculating the average of the height of the pointcloud and eliminating any points less than 20 below the average
        pcd_mean = pcd[z>(np.mean(z) - 20)]

        # Creating an empty point cloud and filling them with the points selected after the height thresholding
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pcd_mean)

        # Finding the best-fit plane for the pointcloud and selecting the points that fall within a certain distance threshold
        plane_model, inliers = cloud.segment_plane(distance_threshold=0.2,
                                                ransac_n=3,
                                                num_iterations=10000)

        # Creating a cloud with only the points near the best-fit plane
        inlier_cloud = cloud.select_by_index(inliers)

        # Setting the color of the point cloud
        inlier_cloud.paint_uniform_color([5/255, 195/255, 221/255])
        
        # Setting a bounding box around the point cloud
        obb = inlier_cloud.get_minimal_oriented_bounding_box(robust=True)
        obb.color = (0, 0, 0)

        # Visualizing the cloud, the bounding box, and the coordinate origin
        #o3d.visualization.draw_geometries([inlier_cloud,obb,origin])

        # Printing out the coordinates for the vertices of the table
        table_coords = np.asarray(obb.get_box_points()).tolist()
        #table_coords = (table_coords.flatten()).tolist()

        return table_coords

# Spinning up the node as the main function
def main():
    rclpy.init()

    node = PointcloudCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

# If this program is executed by itself (python 3) it will automatically execute the 'main' function
if __name__ == "__main__":
    main()