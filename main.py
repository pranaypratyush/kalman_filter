import roslib;roslib.load_manifest('pose_server_python')
import rospy
import time
from resources import topicHeader

import kraken_msgs
import kraken_msgs
from kraken_msgs.msg._absoluteRPY import absoluteRPY
from kraken_msgs.msg._dvlData import dvlData
from kraken_msgs.msg import imuData
from kraken_msgs.msg._positionData import positionData
from kraken_msgs.msg._stateData import stateData
from matrix import matrix

#state = [[pos x,vel x],[pos y,vel y], [pos z,vel z]]
state=[[0,0],[0,0],[0,0]] # initial state (location and velocity)
measurements = [[[0,0,0]],[[0,0,0]]] 	#[[[vx,vy,vz]],[[ax,ay,az]]
P_3dim = [[[1., 0.], [0., 1]],[[1., 0.], [0., 1]],[[1., 0.], [0., 1]]]# initial uncertainty
#pos_history for x in all three dimensions
pos_history=[[0],[0],[0]]
dvlfilled=False
imufilled=False
CONVERTED_TO_WORLD=False
'''
The "state" variable stores the current filtered position and velocity for all
three dimensions.
The "measurements" variable stores a history of all measurements made
although it's sliced after a certain number is reached to improve performance.
"pos_history" stores the history of "filtered" positions for all 3 dimensions
this too is sliced routinely to improve performance.
'''
def transformCallback(abrpy):

	'''
	This funtion transforms the absolute roll,pitch,yaw for body to world.
	At the end of processing, this function toggles the variable "CONVERTED_TO_WORLD".
	This is used later to check if processing is done.
	'''

	global state
	global CONVERTED_TO_WORLD
	global FIRST_ITERATION
	global base_roll
	global base_pitch
	global base_yaw

	vx = state[0][1]
	vy = state[1][1]
	vz = state[2][1]

	# yaw, pitch, roll

	roll = abrpy.roll
	pitch = abrpy.pitch
	yaw = abrpy.yaw

	if FIRST_ITERATION:

		base_roll = roll
		base_pitch = pitch
		base_yaw = yaw
		FIRST_ITERATION = False

	print "IMU (Un-Corrected): ",
	print round(roll, 2),
	print round(pitch, 2),
	print round(yaw, 2)

	roll = roll - base_roll
	pitch = pitch - base_pitch
	yaw = yaw - base_yaw

	print "IMU (Corrected): ",
	print round(roll, 2),
	print round(pitch, 2),
	print round(yaw, 2)

	## Convert the roll, pitch and yaw to radians.

	yaw = yaw * 3.14 / 180
	roll = roll * 3.14 / 180
	pitch = pitch * 3.14 / 180

	## Refer: http://www.chrobotics.com/library/understanding-euler-angles

	bodytoworld = matrix(
		[
		[ # row 1
			cos(yaw) * cos(pitch),
		  sin(yaw) * cos(pitch),
		 	-1 * sin(pitch)
		],

		[ # row 2
			cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll),
		 	sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll),
		 	cos(pitch) * sin(roll)
		],
		[ # row 3
			cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll),
		 	sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll),
		 	cos(pitch) * cos(roll)
		]
	  ]
	)

	bodytoworld = bodytoworld.transpose()

	vel_wrt_body = matrix([[vx],[vy],[0.]])

	vel_wrt_world = bodytoworld * vel_wrt_body

	vel_wrt_world.show()

	CONVERTED_TO_WORLD = True

def dvlCallback(dvl):

	'''
	This function receives data from dvl by subscribing to "topicHeader.SENSOR_DVL".
	It appends velocity data to "measurements" and sets "dvlfilled" to True.
	"dvlfilled" would be used later to check if data from dvl has been received.
	'''
	global state
	global measurements
	global dvfilled

	vx = dvl.data[3]
	vy = -1 * dvl.data[4]
	vz = dvl.data[5]

	roll = dvl.data[0]
	pitch = dvl.data[1]
	yaw = dvl.data[2]

	this_iteration_measurement = [vx, vy,vz]

	#statefilled += 2
	measurements[0]= measurements[0][-100:] + this_iteration_measurement
	dvfilled=True
	return

def imuCallback(imu):

	'''
	This function receives data from imu by subscribing to "topicHeader.SENSOR_IMU".
	It appends acceleration data to "measurements" and sets "imufilled" to True.
	"imufilled" would later be used to check if data from dvl has been received.
	'''
	global measurements
	global imufilled
	ax=imu.data[3]
	ay=imu.data[4]
	az=imu.data[5]

	this_iteration_measurement= [ax,ay,az]

	measurements[1]= measurements[1][-100:] + this_iteration_measurement
	imufilled=True
	return

def publishStateAndPosition(state_matrix):

	'''
	This function is used to publish filtered data to appropriate topics.
	It would be called from the infinite loop.
	'''
	global position_publisher
	global state_publisher

	pos_x = state_matrix[0][0]
	pos_y = state_matrix[1][0]
	pos_z = state_matrix[2][0]
	vx = state_matrix[0][1]
	vy = state_matrix[1][1]
	vz = state_matrix[2][1]
	present_position = positionData()
	present_state = stateData()

	present_position.x_coordinate = present_state.x_coordinate = pos_x
	present_position.y_coordinate = present_state.y_coordinate = pos_y
	present_position.z_coordinate = present_state.z_coordinate = pos_z
	present_state.velocity_x = vx
	present_state.velocity_y = vy
	present_state.velocity_z = vz
	position_publisher.publish(present_position)
	state_publisher.publish(present_state)

	return

absolute_rpy_topic_name = topicHeader.ABSOLUTE_RPY
dvl_topic_name = topicHeader.SENSOR_DVL
publish_state_topic_name = topicHeader.POSE_SERVER_STATE
publish_position_topic_name = topicHeader.PRESENT_POSE

rospy.init_node('pose_server_python_node', anonymous=True)

rospy.Subscriber(name=absolute_rpy_topic_name, data_class=absoluteRPY, callback=transformCallback)
rospy.Subscriber(name=dvl_topic_name, data_class=dvlData, callback=dvlCallback)
rospy.Subscriber(name=topicHeader.SENSOR_IMU, data_class=imuData, callback=imuCallback)

position_publisher = rospy.Publisher(publish_position_topic_name, positionData, queue_size=10)
state_publisher = rospy.Publisher(publish_state_topic_name, stateData, queue_size=10)

r=rospy.Rate(10)
while(1):

	# if all the data has been accumulated in the state variable

#	if(statefilled >= NUM_VARIABLE_IN_STATE and CONVERTED_TO_WORLD):
	if dvlfilled and imufilled and CONVERTED_TO_WORLD :
		#(new_state, new_P) = kalman_estimate(state, P, measurements[-1])
		'''
		In this loop, on each iteration we first check whether the datas are
		available or not. Then we process data for each of the dimensions
		sequentially. The function "kalman_estimate_1D" takes velocity and
		acceleration from previous filter updates in the first argument.
		So state[i] is supplied as this allows us to keep track of all three
		dimensions simulaneously.The second argument is covariance matrix from
		previous filtration. The third argument is the new velocity and accelration
		measurement passed in [vel,acc] form. The fourth argument is position
		history. A filtered history is kept (instead of the current state) for the
		current postion since this would be required in the Unscented Kalman Filter.
		Towards the end of the loop it sets the boolean values to false in order to
		prepare for next iteration. It also publishes the data filtered in the
		current iteration.
		'''

		for i in range(0,2) :
#			x=state[i]
#            P=P_3dim[i]
			va_measurements=[measurements[0][-1][i],measurements[1][-1][i]]
			(state[i],P_3dim[i],pos_history[i]) = kalman_estimate_1D (state[i],P_3dim[i],va_measurements,pos_history[i])
#            state[i]=x
#            P_3dim[i]=P

		#statefilled = 2
		dvlfilled=False
		imufilled=False
		CONVERTED_TO_WORLD = False

		publishStateAndPosition(state)
		r.sleep()


rospy.spin()
