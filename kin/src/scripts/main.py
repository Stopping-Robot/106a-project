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

#Define the method which contains the main functionality of the node.
def controller(ar_tag_num):
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - turtlebot_frame: the tf frame of the AR tag on your turtlebot
  - target_frame: the tf frame of the target AR tag
  """

  ar_marker = "ar_marker_" + str(ar_tag_num)

  #Create a publisher and a tf buffer, which is primed with a tf listener
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      rospy.sleep(1)

      trans1 = tfBuffer.lookup_transform("base", ar_marker, rospy.Time())
      #could be source of error
      trans2 = tfBuffer.lookup_transform("reference/base", "base", rospy.Time())

      #This is a point in the ar_marker frame (i.e. the table), so it should the transformation should return the origin of the ar tag in base frame of reference.  
      ps = PoseStamped()
      ps.header.stamp = rospy.Time.now()
      ps.header.frame_id = ar_marker 
      ps.pose.position.x = 0.0
      ps.pose.position.y = 0.0
      ps.pose.position.z = 0.0
      ps.pose.orientation.x = 0.0
      ps.pose.orientation.y = 0.0
      ps.pose.orientation.z = 0.0
      ps.pose.orientation.w = 1.0

      pose_ref_left_gripper = tf2_geometry_msgs.do_transform_pose(ps, trans1)
      pose_ref_base = tf2_geometry_msgs.do_transform_pose(pose_ref_left_gripper, trans2)
      #pose_left_hand_camera = tf2_geometry_msgs.do_transform_pose(ps, trans1)
      #pose_right_gripper = tf2_geometry_msgs.do_transform_pose(pose_left_hand_camera, trans2)
      print(pose_ref_base)

      pos = pose_ref_base.pose.position 
      move(pos.x, pos.y, pos.z + 0.1)


      #translation = trans1.transform.translation

      #rot = trans.transform.rotation
      #print("AR TAG FRAME TRANS")
      #print(translation)
      #print(rot)
      
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      print(e)
      pass
    # Use our rate object to sleep until it is time to publish again
    r.sleep()

def move(x, y, z, box_in):

    planner = PathPlanner("right_arm")


    #TODO: Add constraint restricting z > 0.
    #Wait for the IK service to become available
    #rospy.wait_for_service('compute_ik')
    #rospy.init_node('service_query')
    place_holder = {'images': [], 'camera_infos': []}
    #rospy.Subscriber("/cameras/head_camera/image", Image, lambda x: place_holder['images'].append(x))
    #rospy.Subscriber("/cameras/head_camera/camera_info", CameraInfo, lambda x: place_holder['camera_infos'].append(x))
    limb = Limb("right")
    #print(limb)
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    count = 0
    print(limb.endpoint_pose())
    calibration_points = []
    # while not rospy.is_shutdown() and count < 3:
    #     #calibration points 1-3, closest to robot right = origin, closest to robot left, id_card 3 3/8 inches forward from first point
    #     '''
    #     /cameras/head_camera/camera_info
    #     /cameras/head_camera/camera_info_std
    #     /cameras/head_camera/image

    #     '''
    #     count += 1
    #     raw_input("calibrate point " + str(count))
    #     pos = limb.endpoint_pose()
    #     calibration_points.append({})
    #     calibration_points[-1]['world_coordinates'] = (pos['position'].x, pos['position'].y, pos['position'].z)
    #     #calibration_points[-1]['camera_info'] = place_holder['camera_infos'][-1]
    #     #calibration_points[-1]['image'] = place_holder['images'][-1]

    # for i in calibration_points:
    #     print(i['world_coordinates'])


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

    ## Add the controller here! IF you uncomment below, you should get the checkoff!
    c = Controller(Kp, Ki, Kd, Kw, Limb('right'))
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;
    box = PoseStamped()
    box.header.frame_id = "base"
    box.pose.position.x = box_in.pose.position.x
    box.pose.position.y = box_in.pose.position.y
    box.pose.position.z = box_in.pose.position.z
    # box.pose.position.x = 0.5
    # box.pose.position.y = 0.0
    # box.pose.position.z = 0.0
    box.pose.orientation.x = 0.0
    box.pose.orientation.y = 0.0
    box.pose.orientation.z = 0.0
    box.pose.orientation.w = 1.0
    planner.add_box_obstacle((3, 4, 0.10), "box", box)
    # planner.add_box_obstacle((box_in.pose.position.x, box_in.pose.position.y, box_in.pose.position.z), "box", box)
    # ree = input("some text")
    while not rospy.is_shutdown():
        try:
            if ROBOT == "baxter":
                _x, _y, _z = .67, -.2, .3
                #x, y, z = .95, .11, .02
                #x, y, z = .7, -.4, .2
                print(0)
            else:
                raise Exception("not configured for sawyer yet.")
            goal_1 = PoseStamped()
            goal_1.header.frame_id = "base"

            #x, y, and z position
            goal_1.pose.position.x = _x
            goal_1.pose.position.y = _y
            goal_1.pose.position.z = _z

            #Orientation as a quaternion
            goal_1.pose.orientation.x = 0.0
            goal_1.pose.orientation.y = -1.0
            goal_1.pose.orientation.z = 0.0
            goal_1.pose.orientation.w = 0.0

            # Might have to edit this . . . 
            plan = planner.plan_to_pose(goal_1, [orien_const])

            raw_input("Press <Enter> to move the right arm to goal pose 1: ")
            start = time.time()
            if not c.execute_path(plan):
                raise Exception("Execution failed")
            runtime = time.time() - start
            with open('/home/cc/ee106a/fa19/class/ee106a-aby/ros_workspaces/final_project_kin/src/kin/src/scripts/timing.txt', 'a') as f:
                f.write(str(runtime) + ' move above point \n')
        except Exception as e:
            print(e)
            traceback.print_exc()
        else:
            break

    while not rospy.is_shutdown():
        try:
            if ROBOT == "baxter":
                #x, y, z = .67, -.2, 0
                # x, y, z = .79, .04, -.04 
                pass
            else:
                raise Exception("not configured for sawyer yet.")
            goal_2 = PoseStamped()
            goal_2.header.frame_id = "base"
            #x, y, and z position
            goal_2.pose.position.x = x
            goal_2.pose.position.y = y
            goal_2.pose.position.z = z

            #Orientation as a quaternion
            goal_2.pose.orientation.x = 0.0
            goal_2.pose.orientation.y = -1.0
            goal_2.pose.orientation.z = 0.0
            goal_2.pose.orientation.w = 0.0

            # Might have to edit this . . . 
            plan = planner.plan_to_pose(goal_2, [orien_const])

            raw_input("Press <Enter> to move the right arm to goal pose 2: ")
            start = time.time()
            if not c.execute_path(plan):
                raise Exception("Execution failed")
            runtime = time.time() - start
            with open('/home/cc/ee106a/fa19/class/ee106a-aby/ros_workspaces/final_project_kin/src/kin/src/scripts/timing.txt', 'a') as f:
                f.write(str(runtime) + ' move to point \n')
        except Exception as e:
            print(e)
            traceback.print_exc()
        else:
            break

    # while not rospy.is_shutdown():
    #     raw_input('Press enter to compute an IK solution:')
        
    #     #Construct the request
    #     request = GetPositionIKRequest()
    #     request.ik_request.group_name = "right_arm"
    #     request.ik_request.ik_link_name = "right_gripper"
    #     request.ik_request.attempts = 20
    #     request.ik_request.pose_stamped.header.frame_id = "base"
        
    #     #Set the desired orientation for the end effector HERE
    #     request.ik_request.pose_stamped.pose.position.x = 0.5
    #     request.ik_request.pose_stamped.pose.position.y = 0.5
    #     request.ik_request.pose_stamped.pose.position.z = 0.0
        
    #     request.ik_request.pose_stamped.pose.orientation.x = 0.0
    #     request.ik_request.pose_stamped.pose.orientation.y = 1.0
    #     request.ik_request.pose_stamped.pose.orientation.z = 0.0
    #     request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
    #     try:
    #         #Send the request to the service
    #         response = compute_ik(request)
            
    #         #Print the response HERE
    #         print(response)
            
    #     except rospy.ServiceException, e:
    #         print "Service call failed: %s"%e

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.desiredPixel = (None, 0)

    #Adi: Use this topic for Sawyer right hand camera feed
    #self.image_sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw",Image,self.callback)

    #Adi: Use this topic for realsense color
    #self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

    #Adi: Use this topic for realsense depth
    #self.image_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image,self.callback)

    #Adi: Use this topic for Baxter left_hand_camera
    self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image,self.callback)

    #Adi: Define the camera model
    self.cam_info = rospy.wait_for_message("/cameras/left_hand_camera/camera_info", CameraInfo, timeout=5)
    self.cam_model = PinholeCameraModel()
    self.cam_model.fromCameraInfo(self.cam_info)

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

    #Tennis Ball:
    #greenLower = (0, 0, 250)
    #greenUpper = (0, 0, 255)
    #Blue Cup:
    greenLower = (100, 130, 200)
    greenUpper = (109, 180, 280)
    
    pts = deque(maxlen=64)

    # resize the frame, blur it, and convert it to the HSV
    # color space
    #cv_image = self.resize(cv_image, width=600)
    blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
    #cv2.imshow("blurred", blurred)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv", hsv)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    cv2.imshow("mask", mask)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = self.grab_contours(cnts)
    center = None
 
    # only proceed if at least one contour was found
    if len(cnts) > 0:
	    # find the largest contour in the mask, then use
	    # it to compute the minimum enclosing circle and
	    # centroid
	    c = max(cnts, key=cv2.contourArea)
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


    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)



  def pixToCam(self, u, v):
    cam_coord = self.cam_model.projectPixelTo3dRay((u,v))
    return cam_coord

  def getDesiredPixel(self):
    #return (754, 509)
    return self.desiredPixel


  def LinePlaneCollision(self, planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):
   
    # ndotu = np.dot(planeNormal, rayDirection)
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
    ps.pose.position.z = 0
    ps.pose.orientation.x = 0.0
    ps.pose.orientation.y = 0.0
    ps.pose.orientation.z = 0.0
    ps.pose.orientation.w = 1.0
    pose = tf2_geometry_msgs.do_transform_pose(ps, transform)



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

    planeNormal = np.array([pose1.pose.position.x - pose.pose.position.x, pose1.pose.position.y - pose.pose.position.y, pose1.pose.position.z - pose.pose.position.z])
    planePoint = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z + 0.125])
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


  def camToWorld(self, ar_tag_num):
    ar_marker = "ar_marker_" + str(ar_tag_num)
  
    #Create a publisher and a tf buffer, which is primed with a tf listener
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
  
    run_while = True
    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown() and run_while:
      try:
        rospy.sleep(1)
        run_while = False
        trans0 = tfBuffer.lookup_transform(ar_marker, "reference/left_hand_camera", rospy.Time())
        trans1 = tfBuffer.lookup_transform("base", ar_marker, rospy.Time())
        #could be source of error
        trans2 = tfBuffer.lookup_transform("reference/base", "base", rospy.Time())
        box = PoseStamped()
        box.header.frame_id = ar_marker
        box.pose.position.x = 0.0
        box.pose.position.y = 0.0
        box.pose.position.z = 0.0
        box.pose.orientation.x = 0.0
        box.pose.orientation.y = 1.0
        box.pose.orientation.z = 0.0
        box.pose.orientation.w = 1.0
        #This is a point in the /reference/left_hand_camera frame 
        # ps = PoseStamped()
        # ps.header.stamp = rospy.Time.now()
        # ps.header.frame_id = "reference/left_hand_camera" 
        # ps.pose.position.x = cam_coord[0]
        # ps.pose.position.y = cam_coord[1]
        # ps.pose.position.z = cam_coord[2]
        # ps.pose.orientation.x = 0.0
        # ps.pose.orientation.y = 0.0
        # ps.pose.orientation.z = 0.0
        # ps.pose.orientation.w = 1.0
  
        pose_base = tf2_geometry_msgs.do_transform_pose(box, trans1)
        pose_ref = tf2_geometry_msgs.do_transform_pose(pose_base, trans2)
        # pose_ref_base = tf2_geometry_msgs.do_transform_pose(pose_base, trans2)
        
  
        #translation = trans1.transform.translation
  
        #rot = trans.transform.rotation
        #print("AR TAG FRAME TRANS")
        #print(translation)
        #print(rot)
        
        
        # TODO finish calling rayToWorld
        # ic = image_converter()
        pixel = None
        while pixel is None:
            #u, v = self.getDesiredPixel()
            pixel, time = self.getDesiredPixel()
        print(pixel)
        u, v = pixel[0], pixel[1]
        
        ray = self.pixToCam(u, v)

        trans3 = tfBuffer.lookup_transform("reference/left_hand_camera", ar_marker, rospy.Time())
        pose_pix = self.rayToWorld(trans3, ray, ar_marker)
        pose_yay = tf2_geometry_msgs.do_transform_pose(pose_pix, trans0)
        pose_lmao = tf2_geometry_msgs.do_transform_pose(pose_yay, trans1)
        print("RIP", pose_lmao)
        pt_lmao = [pose_lmao.pose.position.x,  pose_lmao.pose.position.y,  pose_lmao.pose.position.z]
        # move(pose_lmao.pose.position.x,  pose_lmao.pose.position.y,  pose_lmao.pose.position.z)
        return pt_lmao, pose_ref
        

        
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        run_while = True
        print(e)
        pass

    

      
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

  ic = image_converter()
  pt_lmao, box = ic.camToWorld(sys.argv[2])
  table_delta = 0.3
  #TODO FIND THESE VALUES
  conveyor_height = 0
  cup_height = 0
  move(pt_lmao[0], pt_lmao[1],pt_lmao[2] + table_delta + conveyor_height + cup_height, box)
  # u, v = ic.getDesiredPixel()
  # ray = ic.pixToCam(u, v)

  # trans3 = tfBuffer.lookup_transform("reference/left_hand_camera", ar_marker, rospy.Time())
  # pt = rayToWorld(trans3, ray, ar_marker)
  '''
  #Adi: Computer Vision
  ic = image_converter()
  u, v = ic.getDesiredPixel()
  cam_coord = ic.pixToCam(u, v)
  try:
    ic.camToWorld(0, cam_coord)
    #move()
  except rospy.ROSInterruptException:
    pass
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
  '''
  # try:
  #   controller(sys.argv[2])
  #   #move(0, 0, 0)
  # except rospy.ROSInterruptException:
  #   pass
