#! /usr/bin/env python                                                                                 
                                                                                                       
import robot_api                                                                                       
from joint_state_reader import JointStateReader
import rospy                  
                              
                       
def wait_for_time():   
    """Wait for simulated time to begin.
    """                
    while rospy.Time().now().to_sec() == 0:                                                            
        pass                                                                                           
                                                                                                       
                                                                                                       
def main():                                                                                            
    rospy.init_node('joint_reader_demo')                                                               
    wait_for_time()                                                                                    
    argv = rospy.myargv()                                                                              
    reader = JointStateReader()
    rospy.sleep(0.5)
    # Fetch Only
    # names = robot_api.ArmJoints.names()
    # Kuri: Browse joints and initialize your own names list
    # names = []
    names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']

    arm_vals = reader.get_joints(names)
    for k, v in zip(names, arm_vals):
        print('{}\t{}'.format(k, v))
                      
                      
if __name__ == '__main__':
    main()