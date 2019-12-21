# 106A Final Project: Stopping Robot
## 1. Introduction

##### a. Describe the end goal of your project. b. Why is this an interesting project? What interesting problems do you need to solve to make your solution work? c. In what real-world robotics applications could the work from your project be useful?


The end goal of our project was to be able to stop a moving object using Baxter as before it reached the end of its trajectory. Our project consistently stopped objects moving on a conveyor belt at variable speeds with a cup. In order to accomplish such reliability, we had to solve three interesting problems: how to translate from a pixel captured by a camera to world frame; getting accurate velocity predictions; and being able to always find motion plans. Upon accomplishing this, we would be able to move on to stopping rolling objects, stopping one out of many objects, or even grasping and picking up the object. The problems that we need to solve in order to do all of this include being able to identify an object and distinguish it from its background or other objects with computer vision, determine its velocity and an intercept point along its trajectory to send the Baxter arm to, and then actually move the Baxter arm to intercept the object without missing. This project has many real-world applications, from assembly line robots that have to interact with objects moving along a conveyor belt to robots that could intercept and catch falling or thrown objects.

## 2. Design

## 3. Implementation

## 4. Results
##### How well did your project work? What tasks did it perform?

Based on our project proposal, and the task we outlined, our project worked as well as we could expect. Our program was able to:
- detect a blue ball
- isolate it’s velocity
- predict where it would be in _n_ seconds, and
- properly move the right gripper to cover the ball with a cup

Please check out the images and videos we recorded at the bottom of this page.

## 5. Conclusion

##### Discuss your results. How well did your finished solution meet your design criteria?

Our resulting product accomplished what we set out to accomplish. In terms of individual components, motion planning and actual robot motion was a bit difficult to maintain because it seemed small errors would compound (may reflect issues with encoders or us moving the arm improperly). The motion tracking component was a bit buggy, but it was robust enough to withstand 7-10 trials before needing to be rebooted. The integration of the two was robust - the majority of the latency was limited to motion planning.

##### Did you encounter any particular difficulties?
Yes. AR tags were very buggy to the point of stationary AR tags flickering on Rviz. Asimov had a topic and feed swapped, where the head camera topic would give us the left hand camera feed, the right hand camera topic would give us the  head camera feed. The TF tree had no connection between /reference vs /. Halfway through building the motion tracking, we realized that the lighting was quite variable, which led to variable functionality. This was fixed by tuning values and playing around with the algorithm used.The biggest issue we faced was inconsistencies across worksessions - working conditions were not stable. We found that moving from one robot to another led to massive changes in performance and calibration

##### Does your solution have any flaws or hacks? What improvements would you make if you had additional time?
No, the closest thing we had to a hack was the AR tags being used to determine the plane of operability. If we had the additional time, we would most likely implement our own controller, create our own motion planning system, and improve upon the computer vision side so it could handle bottles (for extensions) and other objects.
## 6. Team

### Aditya Nair
Aditya is a third year EECS  major.  He is passionate about the intersection of multiple different fields with software engineering, including medical technology and finance.  When he gets the chance,  he plays  video  games  and does his  best to exercise (maybe after one more episode of brooklyn nine nine)
#### Contributions: 
worked on motion planning, testing and integration

## 7. Additional Materials
<iframe src="https://drive.google.com/file/d/1BQAm_RjPR9sWDSu1Nwgtj4vPDHOydzAK/preview" width="640" height="480"></iframe>

<iframe src="https://drive.google.com/file/d/1suGYGNwerla3IVOqMhIisVCky3tWUqx2/preview" width="640" height="480"></iframe>

<iframe src="https://drive.google.com/file/d/1fNjhpO_wv8IXBtZxPNynihbfqFGpi8sk/preview" width="640" height="480"></iframe>

<iframe src="https://drive.google.com/file/d/1OA8bqGylb1t3J94DRzgeqRTQAbJtsRG4/preview" width="640" height="480"></iframe>
