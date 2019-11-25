#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
import sys


from path_planner import PathPlanner
from controller import Controller
import numpy as np
import time
import traceback

assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb

def main():

    planner = PathPlanner("right_arm")

    #Wait for the IK service to become available
    #rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    place_holder = {'images': [], 'camera_infos': []}
    rospy.Subscriber("/cameras/head_camera/image", Image, lambda x: place_holder['images'].append(x))
    rospy.Subscriber("/cameras/head_camera/camera_info", CameraInfo, lambda x: place_holder['camera_infos'].append(x))
    limb = Limb("right")
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    count = 0
    print(limb.endpoint_pose())
    calibration_points = []
    while not rospy.is_shutdown() and count < 3:
        #calibration points 1-3, closest to robot right = origin, closest to robot left, id_card 3 3/8 inches forward from first point
        '''
        /cameras/head_camera/camera_info
        /cameras/head_camera/camera_info_std
        /cameras/head_camera/image

        '''
        count += 1
        raw_input("calibrate point " + str(count))
        pos = limb.endpoint_pose()
        calibration_points.append({})
        calibration_points[-1]['world_coordinates'] = (pos['position'].x, pos['position'].y, pos['position'].z)
        calibration_points[-1]['camera_info'] = place_holder['camera_infos'][-1]
        calibration_points[-1]['image'] = place_holder['images'][-1]

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

    for i in range(10):
        while not rospy.is_shutdown():
            try:
                if ROBOT == "baxter":
                    x, y, z = .67, -.2, .108
                else:
                    raise Exception("not configured for sawyer yet.")
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z

                #Orientation as a quaternion
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                # Might have to edit this . . . 
                plan = planner.plan_to_pose(goal_1, [])

                raw_input("Press <Enter> to move the right arm to goal pose 1: ")
                start = time.time()
                if not c.execute_path(plan):
                    raise Exception("Execution failed")
                runtime = time.time() - start
                with open('/home/cc/ee106a/fa19/class/ee106a-aby/ros_workspaces/final_project_kin/src/kin/src/scripts/timing.txt', 'a') as f:
                    f.write(str(runtime) + ' move above point \n')
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

        while not rospy.is_shutdown():
            try:
                if ROBOT == "baxter":
                    x, y, z = .67, -.2, 0
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
                plan = planner.plan_to_pose(goal_2, [])

                raw_input("Press <Enter> to move the right arm to goal pose 2: ")
                start = time.time()
                if not c.execute_path(plan):
                    raise Exception("Execution failed")
                runtime = time.time() - start
                with open('/home/cc/ee106a/fa19/class/ee106a-aby/ros_workspaces/final_project_kin/src/kin/src/scripts/timing.txt', 'a') as f:
                    f.write(str(runtime) + ' move to point \n')
            except Exception as e:
                print e
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

#Python's syntax for a main() method
if __name__ == '__main__':
    main()

