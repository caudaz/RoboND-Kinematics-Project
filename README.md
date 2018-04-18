**Robotics Nanodegree**

**Term1 – Project2: Robotic arm - Pick & Place project**

![](./media/image1.png)

**SETUP AND RUN**

1-Gazebo check. It should be version 7.7.0+:

gazebo --version

sudo sh -c 'echo "deb
http://packages.osrfoundation.org/gazebo/ubuntu-stable \`lsb\_release
-cs\` main" &gt; /etc/apt/sources.list.d/gazebo-stable.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key
add -

sudo apt-get update

sudo apt-get install gazebo7

gazebo –version

2-On TERMINAL:

'''
mkdir -p \~/catkin\_ws/src

cd \~/catkin\_ws/

catkin\_make

cd \~/catkin\_ws/src

git clone <https://github.com/udacity/RoboND-Kinematics-Project.git>

cd \~/catkin\_ws

rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

cd \~/catkin\_ws/src/RoboND-Kinematics-Project/kuka\_arm/scripts

sudo chmod +x target\_spawn.py

sudo chmod +x IK\_server.py

sudo chmod +x safe\_spawner.sh

cd \~/catkin\_ws

catkin\_make

export
GAZEBO\_MODEL\_PATH=\~/catkin\_ws/src/RoboND-Kinematics-Project/kuka\_arm/models

source \~/catkin\_ws/devel/setup.bash

cd \~/catkin\_ws/src/RoboND-Kinematics-Project/kuka\_arm/scripts

./safe\_spawner.sh
'''

3-On TERMINAL:

source \~/catkin\_ws/devel/setup.bash

cd \~/catkin\_ws/src/RoboND-Kinematics-Project/kuka\_arm/scripts

rosrun kuka\_arm IK\_server.py

4-On RVIZ window:

Click on “Next” to move thru actions.

**INTRODUCTION**

This project is modeled after KUKA KR210 Robotic Arm. The purpose is to
implement the inverse kinematics calculations of the system, after being
given the end-effector pose. All the arm angles will be calculated. The
tools used for the project are:

-ROS

-GAZEBO for world physics simulator

-MoveIt for motion planning

-RViz for visualization

The goal of the project is to track the planned trajectory and
successfully complete pick and place operation.

**kinematics THEORY**

Your writeup should contain a DH parameter table with proper notations
and description about how you obtained the table. Make sure to use the
modified DH parameters discussed in this lesson. Please add an annotated
figure of the robot with proper link assignments and joint rotations
(Example figure provided in the writeup template). It is strongly
recommended that you use pen and paper to create this figure to get a
better understanding of the robot kinematics.

Your writeup should contain a DH parameter table with proper notations
and description about how you obtained the table. Make sure to use the
modified DH parameters discussed in this lesson. Please add an annotated
figure of the robot with proper link assignments and joint rotations
(Example figure provided in the writeup template). It is strongly
recommended that you use pen and paper to create this figure to get a
better understanding of the robot kinematics.

Based on the geometric Inverse Kinematics method described here,
breakdown the IK problem into Position and Orientation problems. Derive
the equations for individual joint angles. Your writeup must contain
details about the steps you took to arrive at those equations. Add
figures where necessary. If any given joint has multiple solutions,
select the best solution and provide explanation about your choice
(Hint: Observe the active robot workspace in this project and the fact
that some joints have physical limits).

**kinematics implementation**

IK\_server.py must contain properly commented code. The robot must track
the planned trajectory and successfully complete pick and place
operation. Your writeup must include explanation for the code and a
discussion on the results, and a screenshot of the completed pick and
place process..

Function handle\_calculate\_IK(req) under IK\_serverpy performs the
inverse kinematics calculations:

\# Define DH param symbols

d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') \# link offset

a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') \#link length

alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 =
symbols('alpha0:7') \#twist angle

\# Joint angle symbols

q1, q2, q3, q4 ,q5, q6, q7 = symbols('q1:8')

\# Create Modified DH parameters

DH\_Table = { alpha0: 0, a0: 0, d1: 0.75, q1: q1,

alpha1: -pi/2., a1: 0.35, d2: 0, q2: -pi/2. + q2,

alpha2: 0, a2: 1.25, d3: 0, q3: q3,

alpha3: -pi/2., a3: -.054, d4: 1.5, q4: q4,

alpha4: pi/2., a4: 0, d5: 0, q5: q5,

alpha5: -pi/2., a5: 0, d6: 0, q6: q6,

alpha6: 0, a6: 0, d7: 0.303, q7: 0}

\# Define Modified DH Transformation matrix

def TF\_Matrix(alpha, a, d, q):

TF = Matrix(\[\[ cos(q), -sin(q), 0, a\],

\[ sin(q)\*cos(alpha), cos(q)\*cos(alpha), -sin(alpha),
-sin(alpha)\*d\],

\[ sin(q)\*sin(alpha), cos(q)\*sin(alpha), cos(alpha), cos(alpha)\*d\],

\[ 0, 0, 0, 1\]\])

return TF

\# Create individual transformation matrices

T0\_1 = TF\_Matrix(alpha0, a0, d1, q1).subs(DH\_Table)

T1\_2 = TF\_Matrix(alpha1, a1, d2, q2).subs(DH\_Table)

T2\_3 = TF\_Matrix(alpha2, a2, d3, q3).subs(DH\_Table)

T3\_4 = TF\_Matrix(alpha3, a3, d4, q4).subs(DH\_Table)

T4\_5 = TF\_Matrix(alpha4, a4, d5, q5).subs(DH\_Table)

T5\_6 = TF\_Matrix(alpha5, a5, d6, q6).subs(DH\_Table)

T6\_EE = TF\_Matrix(alpha6, a6, d7, q7).subs(DH\_Table)

T0\_EE = T0\_1 \* T1\_2 \* T2\_3 \* T3\_4 \* T4\_5 \* T5\_6 \* T6\_EE

\# Extract rotation matrices from the transformation matrices

r, p, y = symbols('r p y')

ROT\_x = Matrix(\[\[1, 0, 0\],

\[0, cos(r), -sin(r)\],

\[0, sin(r), cos(r)\]\]) \# ROLL

ROT\_y = Matrix(\[\[ cos(p), 0, sin(p)\],

\[ 0, 1, 0\],

\[-sin(p), 0, cos(p)\]\]) \# PITCH

ROT\_z = Matrix(\[\[cos(y), -sin(y), 0\],

\[sin(y), cos(y), 0\],

\[ 0, 0, 1\]\]) \# YAW

ROT\_EE = ROT\_z \* ROT\_y \* ROT\_x

\# Initialize service response

joint\_trajectory\_list = \[\]

for x in xrange(0, len(req.poses)):

\# IK code starts here

joint\_trajectory\_point = JointTrajectoryPoint()

\# Extract end-effector position and orientation from request

\# px,py,pz = end-effector position

\# roll, pitch, yaw = end-effector orientation

px = req.poses\[x\].position.x

py = req.poses\[x\].position.y

pz = req.poses\[x\].position.z

(roll, pitch, yaw) = tf.transformations.euler\_from\_quaternion(

\[req.poses\[x\].orientation.x, req.poses\[x\].orientation.y,

req.poses\[x\].orientation.z, req.poses\[x\].orientation.w\])

\# Compensate for rotation discrepancy between DH parameters and Gazebo

Rot\_Error = ROT\_z.subs(y, radians(180)) \* ROT\_y.subs(p,
radians(-90))

ROT\_EE = ROT\_EE \* Rot\_Error

ROT\_EE = ROT\_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

EE = Matrix(\[\[px\],

\[py\],

\[pz\]\])

WC = EE - (0.303) \* ROT\_EE\[:,2\]

\# Calculate joint angles using Geometric IK method

theta1 = atan2(WC\[1\], WC\[0\])

\# SS triangle for theta2 and theta3

side\_a = 1.501

side\_b = sqrt(pow((sqrt(WC\[0\] \* WC\[0\] + WC\[1\] \* WC\[1\]) -
0.35), 2) + pow((WC\[2\] - 0.75), 2))

side\_c = 1.25

angle\_a = acos((side\_b \* side\_b + side\_c \* side\_c - side\_a \*
side\_a) / (2 \* side\_b \* side\_c))

angle\_b = acos((side\_a \* side\_a + side\_c \* side\_c - side\_b \*
side\_b) / (2 \* side\_a \* side\_c))

angle\_c = acos((side\_a \* side\_a + side\_b \* side\_b - side\_c \*
side\_c) / (2 \* side\_a \* side\_b))

theta2 = pi / 2 - angle\_a - atan2(WC\[2\] - 0.75, sqrt(WC\[0\] \*
WC\[0\] + WC\[1\] \* WC\[1\]) - 0.35)

theta3 = pi / 2 - (angle\_b + 0.036) \# 0.036 accounts for sag in link4
of -0.054m

R0\_3 = T0\_1\[0:3, 0:3\] \* T1\_2\[0:3, 0:3\] \* T2\_3\[0:3, 0:3\]

R0\_3 = R0\_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

R3\_6 = R0\_3.inv("LU") \* ROT\_EE

\# Euler angles from rotation matrix

theta4 = atan2(R3\_6\[2,2\], -R3\_6\[0,2\])

theta5 = atan2(sqrt(R3\_6\[0,2\] \* R3\_6\[0,2\] + R3\_6\[2,2\] \*
R3\_6\[2,2\]), R3\_6\[1,2\])

theta6 = atan2(-R3\_6\[1,1\], R3\_6\[1,0\])

\# Populate response for the IK request

joint\_trajectory\_point.positions = \[theta1, theta2, theta3, theta4,
theta5, theta6\]

joint\_trajectory\_list.append(joint\_trajectory\_point)

rospy.loginfo("length of Joint Trajectory List: %s" %
len(joint\_trajectory\_list))

return CalculateIKResponse(joint\_trajectory\_list)

**ROS system model**

Rock pickup is WIP. To enable it uncomment lines 59-84 on decision.py.

**CONCLUSIONS**

Code\\ Rover\_Project\_Test\_Notebook01.py -&gt; Simple version of the
Jupyter Notebook
