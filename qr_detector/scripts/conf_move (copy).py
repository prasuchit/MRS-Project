#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def qr_callback(qrtype):
    global robot_id
    val = qrtype.data
    robot_id = val[0]
    obj_type = val[1]
    objs = robot_map[robot_id]
    actions = objs[int(obj_type)]
    best_action = max(actions)
    if actions[best_action] < max_value:
	help_publisher.publish(val)
    elif best_action==0:
	move()
    elif best_action==1:
	move_around()
    else:
	avoid()

def comm_callback(command):
    global robot_id
    val = command.data
    cmd_id = val[0]
    print(cmd_id)
    print(val[1])
    print(val[2])
    obj_id = int(val[1])
    act_id = int(val[2])
    robot_map[cmd_id][obj_id][act_id] += 1
    if cmd_id == robot_id:
	if act_id == 0:
	    move()
    	elif act == 1:
	    move_around()
    	else:
	    avoid()
    

def move():
    velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    vel_msg = Twist()
    
    
    #Set movement speed
    if(True):
        vel_msg.linear.x = abs(2)
    else:
        vel_msg.linear.x = -abs(2)
    #Only consider x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    print(vel_msg.linear.x)
    velocity_publisher.publish(vel_msg)
    while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < 1):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            #current_distance= speed*(t1-t0)
            current_distance += 1

        #After the loop, stops the robot
        #vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
	global max_value, help_publisher, robot_map, robot_id
	max_value = 4
	robot_map = {"A": [[0,0,0],[0,0,0],[0,0,0]], "B": [[0,0,0],[0,0,0],[0,0,0]]}
	actions = {0: "M", 1: "A", 2: "V"}

    	# Initialize this node.
    	rospy.init_node('conf_move')
    	

    	# Subscribe to topics
    	rospy.Subscriber('/qr_codes', String, qr_callback)
	rospy.Subscriber('/command', String, comm_callback)

    	help_publisher = rospy.Publisher('/help', String, queue_size=1)

    	rospy.spin()
        	
        
    except rospy.ROSInterruptException: pass
