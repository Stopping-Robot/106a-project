# 106A Project: Stopping Robot
## Aditya Ganapathi, Steven Lu, Aditya Nair, Mrunal Puram, William Wong

There are two packages to look at: `ar_track_alvar` and `kin`.
We modified `/kin/launch/ar_track.launch` from the course labs to work with the Baxter's left arm camera. Most of the project code we wrote is in `/kin/src/scripts/final_code.py`.

1. SSH into baxter: `./baxter.sh [robot_name].local`
2. Enable robot: `rosrun baxter_tools enable_robot.py -e` for baxter
3. `roslaunch kin baxter_moveit_gui_noexec.launch`
4. Start trajectory controller: `rosrun baxter_interface joint_trajectory_action_server.py`
5. `roslaunch baxter_moveit_config baxter_grippers.launch`
6. `roslaunch kin ar_track.launch`
7. `rosrun kin final_code.py [ar_tag_num]`
