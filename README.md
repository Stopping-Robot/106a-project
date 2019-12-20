# 106a-project
1. .\/baxter.sh [robot\_name].local
2. Enable robot: rosrun baxter\_tools enable\_robot.py -e for baxter
   or: rosrun intera\_interface enable\_robot.py -e for sawyer
3. roslaunch kin baxter\_moveit\_gui\_noexec.launch
5. Start trajectory controller: rosrun baxter\_interface joint\_trajectory\_action\_server.py for baxter
   or: rosrun intera\_interface joint\_trajectory\_action\_server.py for sawyer
6. roslaunch baxter\_moveit\_config baxter\_grippers.launch for baxter
   or: roslaunch sawyer\_moveit\_config sawyer\_moveit.launch electric\_gripper:=true for sawyer
  7. roslaunch kin ar_track.launch
7. rosrun kin main.py [baxter or sawyer]

tf static transform, identity transform where reference is the parent
