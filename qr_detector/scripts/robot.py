import rospy
import time
from std_msgs.msg import String


def command_callback(data):
    global obstacle_encountered, maneuver_dict, helped_by_human, helped_by_robot, confidence_matrix, waiting_for_help, at_obstacle, current_obstacle
    [robID,  obstacle, comID, maneuver] = data.data.split(",")
    if not robID == myID:
        # This is not my command. just exit
        print("This isn't meant for me. It's for {0}. exiting...".format(robID))
    else:
        if obstacle == "0":
            # Debugging: set encountered obstacle to true
            obstacle_encountered = True
            at_obstacle = True
            current_obstacle = "obstacle_D"
            print("setting obstacle_encountered to true")
            return
        else:
            if comID == "Human":
                # total trust
                maneuver_dict[obstacle] = maneuver
                confidence_matrix[obstacle] = confidence_matrix[obstacle] + trust_matrix[comID]
                helped_by_human = True
                # waiting_for_help = False
            else:
                if comID not in trust_matrix:
                    # we can't trust this robot at all
                    print("{0} is not a trusted robot".format(comID))
                    return
                else:
                    if obstacle in maneuver_dict:
                        if maneuver_dict[obstacle] == "unitialized":
                            maneuver_dict[obstacle] = maneuver
                            confidence_matrix[obstacle] = confidence_matrix[obstacle] + trust_matrix[comID]
                            helped_by_robot = True
                        elif not maneuver_dict[obstacle] == maneuver:
                            # why is this robot giving us a maneuver that's different from current one? ignore this
                            pass
                        elif maneuver_dict[obstacle] == maneuver:
                            confidence_matrix[obstacle] = confidence_matrix[obstacle] + trust_matrix[comID]
                            helped_by_robot = True

        rospy.loginfo(rospy.get_caller_id() + ' says: %s', data.data)

def help_callback(help_msg):
    [rob_ID, rob_obstacle] = help_msg.data.split(",")
    if rob_ID == myID:
        print("can't help myself. duh!")
        return
    else:
        if check_confidence(rob_obstacle):
            # message format requesting_robotID , obstacle, robot/human_sending_command, maneuvre_corresponding_to_obstacle
            command_msg = rob_ID + "," + rob_obstacle + "," + myID + "," + maneuver_dict[rob_obstacle]
            command_pub.publish(command_msg)
            print("My confidence matrix for obstacle {0} is high enough. I can help robot {1}!".format(rob_obstacle, rob_ID))


def execute_maneuver(maneuver):
    global maneuvering
    global velocity_publisher
    maneuvering = True # in QR code, if maneuvering is true, do not detect obstacles!
    if maneuver == "maneuver_A":
        print("performing maneuver A") #Go straight
	move()
    elif maneuver == "maneuver_B":
        print("performing maneuver B") #Turn right
	vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 1.0
        velocity_publisher.publish(vel_msg)
        move()
    elif maneuver == "maneuver_C":
        print("performing maneuver C") #Turn left
	vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z =  -1.0
        velocity_publisher.publish(vel_msg)
        move()
    elif maneuver == "maneuver_D":
        print("performing maneuver D") #Undetermined
    else:
        print("Unrecognized maneuver")
    maneuvering = False
    return

def move():
    global velocity_publisher
    vel_msg = Twist()
    
    
    #Set movement speed
    vel_msg.linear.x = abs(0.5)
    speed = 0.5
    #Only move in a straight line
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    print(vel_msg.linear.x)
    velocity_publisher.publish(vel_msg)

    #Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    #Loop to move the turtle in an specified distance
    while(current_distance < 4):
        #Publish the velocity
        velocity_publisher.publish(vel_msg)
        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        #Calculates distance traveled
        current_distance= speed*(t1-t0)

    #After the loop, stops the robot
    vel_msg.linear.x = 0
    #Force the robot to stop
    velocity_publisher.publish(vel_msg)


def check_confidence(obstacle):
    global confidence_matrix
    if obstacle not in confidence_matrix:
        # add obstacle to dictionary. 0 confidence
        confidence_matrix[obstacle] = 0
    return confidence_matrix[obstacle] >= confidence_threshold


if __name__ == '__main__':
    global obstacle_encountered, maneuver_dict, helped_by_human, helped_by_robot, confidence_matrix, waiting_for_help, at_obstacle, current_obstacle, velocity_publisher
    myID = "Robot1"
    #initialize globals
    velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    at_obstacle = False
    helped_by_robot = False
    helped_by_human = False
    maneuvering = False # don't detect obstacles if we're maneuvering
    current_obstacle = "obstacle_B" #default obstacle. should change in the main loop, where asynchronous QR is active

    obstacle_encountered = False
    waiting_for_help = False
    timeout = 10 # seconds
    prev_obstacle_time = time.time()

    # initialize confidence matrix with
    confidence_matrix = dict()
    confidence_threshold = 0.5
    initial_confidence = 0.0
    obstacle_list = ['obstacle_A', 'obstacle_B', 'obstacle_C', 'obstacle_D']
    for O in obstacle_list:
        confidence_matrix[O] = initial_confidence

    # initialize maneuver dictionary
    maneuver_list = ['uninitialized', 'uninitialized', 'uninitialized', 'uninitialized']
    maneuver_dict = dict(zip(obstacle_list, maneuver_list))

    # initialize confidence matrix
    trust_list = ["Robot2", "Robot3", "Human"]
    trust_values = [0.1, 0.5, 1.0]
    trust_matrix = dict(zip(trust_list, trust_values))

    # intialize robot node
    rospy.init_node('Robot', anonymous=True)
    help_pub = rospy.Publisher('/help', String, queue_size=10)  # create publisher to help topic
    command_pub = rospy.Publisher('/command', String, queue_size=10)  # create publisher to command topic
    rospy.Subscriber('/command', String, command_callback)  # subscribe to command topic
    rospy.Subscriber('/help', String, help_callback) # subscribe to command topic
    rate = rospy.Rate(100) # hz


    while not rospy.is_shutdown():
        # rospy.loginfo(msg)
        if obstacle_encountered:
            obstacle_encountered = False
            if check_confidence(current_obstacle):
                maneuver = maneuver_dict[current_obstacle]
                execute_maneuver(maneuver)
                print("my confidence is high enough for {}. executing evasive action".format(current_obstacle))
                at_obstacle = False
            else:
                if not waiting_for_help:
                    helped_by_robot = False
                    helped_by_human = False
                    print("new obstacle encountered. publishing to help topic")
                    msg = myID + "," + current_obstacle
                    help_pub.publish(msg)
                    waiting_for_help = True
                    prev_obstacle_time = time.time()
        if waiting_for_help:
            if helped_by_human:
                waiting_for_help = False
            elif helped_by_robot and time.time() - prev_obstacle_time > timeout and check_confidence(current_obstacle):
                waiting_for_help = False
            else:
                print("waiting for clarification")
        if not (maneuvering or waiting_for_help or obstacle_encountered or at_obstacle):
            # keep exploring
            print("I am exploring") # move robot according to algorithm here: SLAM. Random movement. Search and rescue. whatever.
            # check for QR codes here
        if at_obstacle and not waiting_for_help:
            # executing maneuver to move me away from obstacle
            at_obstacle = False
            execute_maneuver(maneuver_dict[current_obstacle])
            print("Executing maneuver to move away from obstacle")
        rate.sleep()
