#!/usr/bin/env python3

import rospy
import rosbag
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from open_manipulator_msgs.msg import JointPosition
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MimicController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('mimic_controller')

        self.joint_position = JointState()
        self.slave_joint_trajectory = JointTrajectory()
        self.slave_joint_trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        self.slave_joint_trajectory_points = JointTrajectoryPoint()
        
        # Initialize ROS publishers and subscribers
        self.joint_pub = rospy.Publisher('/slave/arm_controller/command', JointTrajectory, queue_size=10)
        self.joint_cmd_sub = rospy.Subscriber('/master/joint_states', JointState, self.joint_states_callback)

        print("Mimic Controller")
        print("Options:")
        print("1: Start Mimicking Master Movements")
        print("2: Stop Mimicking Master Movements")
        print("3: Start Recording Mimic Trajectory")
        print("4: Stop Recording Mimic Trajectory")
        print("5: Play Recorded Mimic Trajectory")
        
        self.is_mimicking = False
        self.is_recording = False
        self.mimic_trajectory = []
        self.record_bag = None


    def joint_states_callback(self, msg):
        if self.is_mimicking:
            self.joint_position.position = msg.position [1:]
            self.update_slave_mimic()

        if self.is_recording:
            self.mimic_trajectory.append(msg)


    def update_slave_mimic(self):
        self.slave_joint_trajectory_points.positions = self.joint_position.position
        self.slave_joint_trajectory_points.time_from_start = rospy.Duration(0.1)
        self.slave_joint_trajectory.points = [self.slave_joint_trajectory_points]
        
        self.joint_pub.publish(self.slave_joint_trajectory)


    def record_trajectory(self,filename):
        if not filename.endswith('.bag'):
            filename += '.bag'

        if self.is_recording:
            self.record_bag = rosbag.Bag(filename, 'w')
            
            for point in self.mimic_trajectory:
                self.record_bag.write('/master/joint_states', point)

            self.record_bag.close()
            print('Recorded trajectory to {}'.format(filename))
        

    def playback_trajectory(self,filename):
        if not filename.endswith('.bag'):
            filename += '.bag'
        
        playback_bag = rosbag.Bag(filename, 'r')

        for topic, msg, t in playback_bag.read_messages():
            if topic == '/master/joint_states':

                self.joint_position.position = msg.position [1:]
                self.update_slave_mimic()
                rospy.sleep(0.01)  

        playback_bag.close()


    def run(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            option = input("Enter option: ")

            if option == '1':
                self.is_mimicking = True
                print("Mimicking Master Movements...")

            elif option == '2':
                self.is_mimicking = False
                print("Stop The Mimic ...")

            elif option == '3':
                self.mimic_trajectory = []
                self.is_recording = True
                print("Recording Mimic Trajectory... Press 4 or 5 to stop recording and save the file")   

            elif option == '4':
                filename = input("Enter the filename to record: ")
                self.record_trajectory(filename)
                print("Stopped Recording Mimic Trajectory.")
                self.is_recording = False

            elif option == '5':
                self.is_mimicking = False
                if self.is_recording:
                    filename = input("Enter the filename to record at: ")
                    self.record_trajectory(filename)
                    print("Stopped Recording Mimic Trajectory.")
                    self.is_recording = False

                print("Playing Recorded Mimic Trajectory...")
                filename = input("Enter the filename to play: ")
                self.playback_trajectory(filename)
                print("Finished")

            else:
                print("Invalid option. Please enter a valid option.")

            rate.sleep()


if __name__ == '__main__':
    mimic_controller = MimicController()
    mimic_controller.run()