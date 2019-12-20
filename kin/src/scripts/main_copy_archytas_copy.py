#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image, CameraInfo
import sys
import tf2_ros
import tf2_geometry_msgs


from moveit_msgs.msg import OrientationConstraint
from path_planner import PathPlanner
from controller import Controller
import numpy as np
import time
import traceback

#Adi: Image related imports
#from __future__ import print_function
from collections import deque
import imutils

import sys
import rospy
import cv2
from image_geometry import PinholeCameraModel
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb

#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys
import numpy as np
from geometry_msgs.msg import Twist, Vector3, TransformStamped


class Mover:
    def __init__(self, box_in):
        self.box = box_in
        self.conveyor_z = -0.038
        self.mover_setup()

    def mover_setup(self):
        limb = Limb("right")
        # Right arm planner
        self.planner = PathPlanner("right_arm")
        # Left arm planner
        self.planner_left = PathPlanner("left_arm")

        place_holder = {'images': [], 'camera_infos': []}

        #Create the function used to call the service
        compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        count = 0
        print(limb.endpoint_pose())
        calibration_points = []

        if ROBOT == "sawyer":
            Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
            Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
            Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
            Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        else:
            Kp = 0.45 * np.array([0.8, 2.5, 1.7, 2.2, 2.4, 3, 4])
            Kd = 0.015 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
            Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
            Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

        # Controller for right arm
        self.c = Controller(Kp, Ki, Kd, Kw, Limb('right'))
        self.orien_const = OrientationConstraint()
        self.orien_const.link_name = "right_gripper";
        self.orien_const.header.frame_id = "base";
        self.orien_const.orientation.y = -1.0;
        self.orien_const.absolute_x_axis_tolerance = 0.1;
        self.orien_const.absolute_y_axis_tolerance = 0.1;
        self.orien_const.absolute_z_axis_tolerance = 0.1;
        self.orien_const.weight = 1.0;
        box = PoseStamped()
        box.header.frame_id = "base"
        box.pose.position.x = self.box.pose.position.x
        box.pose.position.y = self.box.pose.position.y
        # box.pose.position.z = self.box.pose.position.z
        box.pose.position.z = self.conveyor_z
        # box.pose.position.x = 0.5
        # box.pose.position.y = 0.0
        # box.pose.position.z = 0.0
        box.pose.orientation.x = 0.0
        box.pose.orientation.y = 0.0
        box.pose.orientation.z = 0.0
        box.pose.orientation.w = 1.0
        self.planner.add_box_obstacle((100, 100, 0.00001), "box", box)

        # Controller for left arm
        self.c_left = Controller(Kp, Ki, Kd, Kw, Limb('left'))

    def move(self, x, y, z):
        try:
            print("Moving\n\n\n")
            print(x, y, z)
            goal = PoseStamped()
            goal.header.frame_id = "base"
            #x, y, and z position
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = z

            #Orientation as a quaternion
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = -1.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 0.0

            plan = self.planner.plan_to_pose(goal, [self.orien_const])
            # TODO: Remove the raw_input for the demo.
            # x = raw_input("Move?")
            if not self.c.execute_path(plan):
                raise Exception("Execution failed")    
        except Exception as e:
            print(e)
            traceback.print_exc()

    def delayed_move(self, x, y, z, delay, time1):
        try:
            print("Delayed Move\n\n")
            print(x, y, z)
            goal = PoseStamped()
            goal.header.frame_id = "base"
            #x, y, and z position
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = z

            #Orientation as a quaternion
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = -1.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 0.0

            plan = self.planner.plan_to_pose(goal, [self.orien_const])

            # Wait till delay has elapsed.
            while rospy.Time.now().to_sec() < time1 + delay:
                pass

            if not self.c.execute_path(plan):
                raise Exception("Execution failed")    
        except Exception as e:
            print(e)
            traceback.print_exc()

    # Move camera to set position (optional if cannot find a good intermediary pose)
    def move_camera(self):
        try:
            goal = PoseStamped()
            goal.header.frame_id = "base"
            # TODO: Update position and orientation values
            #x, y, and z position
            goal.pose.position.x = .655
            goal.pose.position.y = .21
            goal.pose.position.z = .25

            #Orientation as a quaternion
            goal.pose.orientation.x = -.7
            goal.pose.orientation.y = .71
            goal.pose.orientation.z = -.03
            goal.pose.orientation.w = -.02

            plan = self.planner.plan_to_pose(goal)
            # TODO: Remove the raw_input for the demo.
            x = raw_input("Move?")
            if not self.c.execute_path(plan):
                raise Exception("Execution failed")    
        except Exception as e:
            print(e)
            traceback.print_exc()

'''
230 conveyor speed
time

self.time_delta = 2
# Delay (seconds) to stop object after calculating velocity.
self.time_to_stop = 15
# TODO: Time (seconds) to drop from hover position.
self.time_to_drop = 3.1417

# Amount to move gripper during sweeping.
self.sweep_dist = 0.1

self.detected_class = 'blue'

self.hover_z = 0.2386
self.drop_z = self.conveyor_z + 0.0748
self.table_delta = 0
self.conveyor_height = 0
self.cup_height = 0.125

self.ar_marker = "ar_marker_" + str(ar_tag_num)
self.ic_setup()
self.conveyor_z = -0.038
'''


class image_converter:

    def __init__(self, ar_tag_num):
        #self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.debug = False
        self.bridge = CvBridge()
        self.counter = 0
        self.prevPoint = (None, 0)

        #Adi: Use this topic for Sawyer right hand camera feed
        #self.image_sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw",Image,self.callback)

        #Adi: Use this topic for realsense color
        #self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

        #Adi: Use this topic for realsense depth
        #self.image_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image,self.callback)
        self.desiredPixel = (None, 0)
        #Adi: Use this topic for Baxter left_hand_camera
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image,self.callback)

        #Adi: Define the camera model
        self.cam_info = rospy.wait_for_message("/cameras/left_hand_camera/camera_info", CameraInfo, timeout=5)
        self.cam_model = PinholeCameraModel()
        self.cam_model.fromCameraInfo(self.cam_info)

            #Create a publisher and a tf buffer, which is primed with a tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.table_delta = 0
        self.conveyor_height = 0
        self.cup_height = 0.125

        self.ar_marker = "ar_marker_" + str(ar_tag_num)
        self.ic_setup()
        self.conveyor_z = -0.038
        self.Mover = Mover(self.box)
        # Move camera to set position
        #self.Mover.move_camera()

        # Query rate (seconds) for 2nd pixel in camToWorld()
        self.time_delta = 3
        # Delay (seconds) to stop object after calculating velocity.
        self.time_to_stop = 15
        # TODO: Time (seconds) to drop from hover position.
        self.time_to_drop = 1

        # Amount to move gripper during sweeping.
        self.sweep_dist = 0.1

        self.detected_class = 'blue'

        self.hover_z = 0.2386
        self.drop_z = self.conveyor_z + 0.04
        # self.table_z
        # self.plane_z
        # self.box_z


    # def callback(self,data):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #         #cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
    #         #print(cv_image)
    #         #cv2.imshow("image", cv_image)
    #     except CvBridgeError as e:
    #         print(e)

    #     #print(cv_image.shape)
    #     #print("NOT ON BALL")
    #     #print(cv_image[10][10])
    #     #print("ON BALL")
    #     #print(cv_image[320][240])
    #     (rows,cols, channel) = cv_image.shape
    #     if cols > 60 and rows > 60 :
    #         cv2.circle(cv_image, self.getDesiredPixel(), 10, 255)

    #     cv2.imshow("Image window", cv_image)
    #     cv2.waitKey(3)

    #     #try:
    #     #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #     #except CvBridgeError as e:
    #     #  print(e)

    def resize(self, image, width=None, height=None, inter=cv2.INTER_AREA):
      # initialize the dimensions of the image to be resized and
      # grab the image size
        dim = None
        (h, w) = image.shape[:2]

        # if both the width and height are None, then return the
        # original image
        if width is None and height is None:
            return image

        # check to see if the width is None
        if width is None:
            # calculate the ratio of the height and construct the
            # dimensions
            r = height / float(h)
            dim = (int(w * r), height)

        # otherwise, the height is None
        else:
            # calculate the ratio of the width and construct the
            # dimensions
            r = width / float(w)
            dim = (width, int(h * r))

        # resize the image
        resized = cv2.resize(image, dim, interpolation=inter)

        # return the resized image
        return resized
    
    def grab_contours(self, cnts):
      # if the length the contours tuple returned by cv2.findContours
      # is '2' then we are using either OpenCV v2.4, v4-beta, or
      # v4-official
        if len(cnts) == 2:
            cnts = cnts[0]

        # if the length of the contours tuple is '3' then we are using
        # either OpenCV v3, v4-pre, or v4-alpha
        elif len(cnts) == 3:
            cnts = cnts[1]

        # otherwise OpenCV has changed their cv2.findContours return
        # signature yet again and I have no idea WTH is going on
        else:
            raise Exception(("Contours tuple must have length 2 or 3, "
              "otherwise OpenCV changed their cv2.findContours return "
              "signature yet again. Refer to OpenCV's documentation "
              "in that case"))

        # return the actual contours array
        return cnts


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
          #cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
          #print(cv_image)
          #cv2.imshow("image", cv_image)
        except CvBridgeError as e:
            print(e)

        
        #(rows,cols, channel) = cv_image.shape
        #if cols > 60 and rows > 60 :
        #  cv2.circle(cv_image, self.getDesiredPixel(), 10, 255)

        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)

        # Red puck: TODO fix hsv range
        # redLower = (0, 0, 250)
        # redUpper = (0, 0, 255)
        # Blue ball:
        #100, 115
        #60, 190  200, 280
        blueLower = (60, 130, 100)
        blueUpper = (260, 180, 280)
        
        pts = deque(maxlen=64)

        # resize the frame, blur it, and convert it to the HSV
        # color space
        #cv_image = self.resize(cv_image, width=600)
        blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
        #cv2.imshow("blurred", blurred)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        #hsv = hsv[465:796, 238:400]
        #hsv = hsv[200:400, 400:600]
        #print(hsv.shape)
        if self.debug:
            cv2.imshow("hsv", hsv)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        #mask = [:, 400:600]
        blueMask = cv2.inRange(hsv, blueLower, blueUpper)
        if self.debug:
            cv2.imshow("mask", blueMask)
        blueMask = cv2.erode(blueMask, None, iterations=2)
        blueMask = cv2.dilate(blueMask, None, iterations=2)

        # redMask = cv2.inRange(hsv, redLower, redUpper)
        # cv2.imshow("mask", redMask)
        # redMask = cv2.erode(redMask, None, iterations=2)
        # redMask = cv2.dilate(redMask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        blue_cnts = cv2.findContours(blueMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_cnts = self.grab_contours(blue_cnts)
        # red_cnts = cv2.findContours(redMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # red_cnts = self.grab_contours(red_cnts)
        center = None
 
    # only proceed if at least one contour was found
        if len(blue_cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            self.detected_class = 'blue'
            c = max(blue_cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
     
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                self.desiredPixel = (center, rospy.Time.now())
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
            else:
                self.desiredPixel = (None, 0)
        # elif len(red_cnts) > 0:
        #     # find the largest contour in the mask, then use
        #     # it to compute the minimum enclosing circle and
        #     # centroid
        #     self.detected_class = 'red'
        #     c = max(red_cnts, key=cv2.contourArea)
        #     ((x, y), radius) = cv2.minEnclosingCircle(c)
        #     M = cv2.mcenteroments(c)
        #     center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
     
        #     # only proceed if the radius meets a minimum size
        #     if radius > 10:
        #         # draw the circle and centroid on the frame,
        #         # then update the list of tracked points
        #         self.desiredPixel = (center, rospy.Time.now())
        #         cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        #         cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
        #     else:
        #         self.desiredPixel = (None, 0)
     
        # update the points queue
        pts.appendleft(center)
        # loop over the set of tracked points
        for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
            if pts[i - 1] is None or pts[i] is None:
                continue
     
            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(cv_image, pts[i - 1], pts[i], (0, 0, 255), thickness)

        if self.debug:
            cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


    def pixToCam(self, u, v):
        cam_coord = self.cam_model.projectPixelTo3dRay((u,v))
        return cam_coord

    def getDesiredPixel(self):
        # return (844, 328)
        pixel = None
        while pixel is None:
            #u, v = self.getDesiredPixel()
            pixel, time = self.desiredPixel
        #print(pixel)
        return pixel, time.to_sec()

    def LinePlaneCollision(self, planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):
        ndotu = planeNormal.dot(rayDirection)
        if abs(ndotu) < epsilon:
          raise RuntimeError("no intersection or line is within plane")

        w = rayPoint - planePoint
        si = -planeNormal.dot(w) / ndotu
        Psi = w + si * rayDirection + planePoint
        return Psi

    def rayToWorld(self, transform, ray, ar_marker):
        # Transform is ar to camera
        # Transform ar_marker's normal z vector, origin into camera frame to get planeNormal, planePoint
        # rayDirection = pixToCam, rayPoint = 0,0,0

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = ar_marker 
        ps.pose.position.x = 0
        ps.pose.position.y = 0
        ps.pose.position.z = self.conveyor_z
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        pose = tf2_geometry_msgs.do_transform_pose(ps, self.trans4)
        # pose = tf2_geometry_msgs.do_transform_pose(ps, transform)



        ps1 = PoseStamped()
        ps1.header.stamp = rospy.Time.now()
        ps1.header.frame_id = ar_marker 
        ps1.pose.position.x = 0
        ps1.pose.position.y = 0
        ps1.pose.position.z = 1
        ps1.pose.orientation.x = 0.0
        ps1.pose.orientation.y = 0.0
        ps1.pose.orientation.z = 0.0
        ps1.pose.orientation.w = 1.0
        pose1 = tf2_geometry_msgs.do_transform_pose(ps1, transform)

        planeNormal = np.array([0,0,1])
        # planeNormal = np.array([pose1.pose.position.x - pose.pose.position.x, pose1.pose.position.y - pose.pose.position.y, pose1.pose.position.z - pose.pose.position.z])
        # TODO: Adjust planepoint + z
        # planePoint = np.array([0, 0, 0.048])
        planePoint = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        rayDirection = np.array([ray[0], ray[1], ray[2]])
        rayPoint = np.array([0,0,0])

        pt = self.LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint)
        ps2 = PoseStamped()
        ps2.header.stamp = rospy.Time.now()
        ps2.header.frame_id = ar_marker 
        ps2.pose.position.x = pt[0]
        ps2.pose.position.y = pt[1]
        ps2.pose.position.z = pt[2]
        ps2.pose.orientation.x = 0.0
        ps2.pose.orientation.y = 0.0
        ps2.pose.orientation.z = 0.0
        ps2.pose.orientation.w = 1.0
        # pt is in camera coordinates.
        return ps2


    def ic_setup(self):
        run_while = True
        # Loop until the node is killed with Ctrl-C
        while run_while:
            try:
                rospy.sleep(1)
                run_while = False
                self.trans0 = self.tfBuffer.lookup_transform(self.ar_marker, "reference/left_hand_camera", rospy.Time())
                self.trans1 = self.tfBuffer.lookup_transform("base", self.ar_marker, rospy.Time())
                self.trans2 = self.tfBuffer.lookup_transform("reference/base", "base", rospy.Time())
                self.trans3 = self.tfBuffer.lookup_transform("reference/left_hand_camera", self.ar_marker, rospy.Time())
                self.trans4 = self.tfBuffer.lookup_transform("reference/left_hand_camera", "reference/base", rospy.Time())

                box = PoseStamped()
                box.header.frame_id =self.ar_marker
                box.pose.position.x = 0.0
                box.pose.position.y = 0.0
                box.pose.position.z = 0.0
                box.pose.orientation.x = 0.0
                box.pose.orientation.y = 1.0
                box.pose.orientation.z = 0.0
                box.pose.orientation.w = 1.0

                pose_base = tf2_geometry_msgs.do_transform_pose(box, self.trans1)
                self.box = pose_ref = tf2_geometry_msgs.do_transform_pose(pose_base, self.trans2)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                run_while = True
                print(e)
                pass

    def camToWorld(self, pixel):
        run_while = True
        # Loop until the node is killed with Ctrl-C
        while not rospy.is_shutdown() and run_while:
            try:
                # Do we want this sleep?
                # rospy.sleep(1)
                run_while = False

                u, v = pixel[0], pixel[1]
                # u, v = self.getDesiredPixel()
                ray = self.pixToCam(u, v)

                pose_pix = self.rayToWorld(self.trans3, ray, self.ar_marker)
                pose_yay = tf2_geometry_msgs.do_transform_pose(pose_pix, self.trans0)
                pose_lmao = tf2_geometry_msgs.do_transform_pose(pose_yay, self.trans1)
                print("RIP", pose_lmao)
                pt_lmao = [pose_lmao.pose.position.x,  pose_lmao.pose.position.y,  pose_lmao.pose.position.z]
                return pt_lmao
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                run_while = True
                print(e)
                pass


    # def avgVelocity(self):
    #     x_velocities = []
    #     y_velocities = []
    #     self.desiredPixel = (None, 0)
    #     self.counter = 1

    #     self.prevPix = self.getDesiredPixel()
        
    #     initial_time = self.prevPix[1]
    #     while rospy.Time.now().to_sec() < initial_time + (self.counter * 0.5):
    #         pass
    #     self.currPix = self.getDesiredPixel()
    #     prev_pt = self.camToWorld(self.prevPix[0])
    #     initial_pt = prev_pt

    #     while self.currPix[1] - initial_time < self.time_delta:
    #         cur_pt = self.camToWorld(self.currPix[0])
    #         print('TIME', self.currPix[1] - self.prevPix[1])
    #         x_vel = (cur_pt[0] - prev_pt[0])/(self.currPix[1] - self.prevPix[1])
    #         y_vel = (cur_pt[1] - prev_pt[1])/(self.currPix[1] - self.prevPix[1])
    #         self.counter += 1
    #         x_velocities.append(x_vel)
    #         y_velocities.append(y_vel)
    #         prev_pt = cur_pt
    #         self.prevPix = (self.currPix[0], self.currPix[1])
    #         while rospy.Time.now().to_sec() < initial_time + (self.counter * 0.5):
    #             pass
    #         self.currPix = self.getDesiredPixel()

    #     velocity = (sum(x_velocities) / len(x_velocities), sum(y_velocities) / len(y_velocities))
    #     x_displacement = velocity[0] * self.time_to_stop
    #     y_displacement = velocity[1] * self.time_to_stop
    #     final_world = [initial_pt[0] + x_displacement, initial_pt[1] + y_displacement, initial_pt[2]]
    #     return final_world, initial_time

    def avgVelocity(self):
        x_velocities = []
        y_velocities = []
        self.desiredPixel = (None, 0)
        self.counter = 1

        self.prevPix = self.getDesiredPixel()
        
        initial_time = self.prevPix[1]
        while rospy.Time.now().to_sec() < initial_time + (self.counter * 0.5):
            pass
        self.currPix = self.getDesiredPixel()
        prev_pt = self.prevPix[0]
        initial_pt = prev_pt

        while self.currPix[1] - initial_time < self.time_delta:
            cur_pt = self.currPix[0]
            print('TIME', self.currPix[1] - self.prevPix[1])
            x_vel = (cur_pt[0] - prev_pt[0])/(self.currPix[1] - self.prevPix[1])
            y_vel = (cur_pt[1] - prev_pt[1])/(self.currPix[1] - self.prevPix[1])
            self.counter += 1
            x_velocities.append(x_vel)
            y_velocities.append(y_vel)
            prev_pt = cur_pt
            self.prevPix = (self.currPix[0], self.currPix[1])
            while rospy.Time.now().to_sec() < initial_time + (self.counter * 0.5):
                pass
            self.currPix = self.getDesiredPixel()

        velocity = ((sum(x_velocities)+self.time_delta) / len(x_velocities), (sum(y_velocities)+self.time_delta) / len(y_velocities))
        x_displacement = velocity[0] * self.time_to_stop
        y_displacement = velocity[1] * self.time_to_stop
        final_world = self.camToWorld([initial_pt[0] + x_displacement, initial_pt[1] + y_displacement])
        return final_world, initial_time


    def run(self):
        # run_while = True
        # Loop until the node is killed with Ctrl-C
        while not rospy.is_shutdown():
            raw_input("tracking")
            self.Mover.move(.67, -.2, .3)
            hover_point, time1 = self.avgVelocity()
            # hover_point, time1 = self.getStopPosition()
            # Hover over stop position
            # self.Mover.move(hover_point[0], hover_point[1], self.hover_z)
            # self.Mover.move(stop_point[0], stop_point[1], stop_point[2] + self.table_delta + self.conveyor_height + self.cup_height)
            # TODO: set drop height diff to be the amount of the cup that is not grasped.
            drop_point = [hover_point[0], hover_point[1], self.drop_z]
            # drop_point = [stop_point[0], stop_point[1], stop_point[2] + self.table_delta + self.conveyor_height + (0.5) * self.cup_height]
            self.Mover.delayed_move(drop_point[0], drop_point[1], drop_point[2], self.time_to_stop - self.time_to_drop, time1)

            # TODO: Do other things.  For now, just sleep.
            # rospy.sleep(2)
            
            if self.detected_class == 'blue':
                self.Mover.move(drop_point[0] + self.sweep_dist, drop_point[1], drop_point[2]) 
            elif cself.detected_class == 'red':
                self.Mover.move(drop_point[0] - self.sweep_dist, drop_point[1], drop_point[2])
            

if __name__ == '__main__':
    # Check if the node has received a signal to shut down
    # If not, run the talker method

    #Run this program as a new node in the ROS computation graph 
    #called /turtlebot_controller.
    rospy.init_node('our_controller', anonymous=True)

    #Static Transforms
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "/reference/left_hand_camera"
    static_transformStamped.child_frame_id = "/left_hand_camera" 

    static_transformStamped.transform.translation.x = 0 
    static_transformStamped.transform.translation.y = 0
    static_transformStamped.transform.translation.z = 0

    static_transformStamped.transform.rotation.x = 0 
    static_transformStamped.transform.rotation.y = 0
    static_transformStamped.transform.rotation.z = 0
    static_transformStamped.transform.rotation.w = 1

    broadcaster.sendTransform(static_transformStamped)

    ic = image_converter(sys.argv[2])
    ic.run()
