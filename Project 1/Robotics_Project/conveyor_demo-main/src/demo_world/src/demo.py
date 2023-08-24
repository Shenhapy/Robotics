#!/usr/bin/env python3

import rospy, tf, rospkg, random
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import Quaternion, Pose, Point
from std_msgs.msg import String
cube_names=[]
class CubeSpawner():

    def __init__(self) -> None:
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('demo_world')+"/urdf/"
        self.cubes = []
        self.cubes.append(self.path+"red_cube.urdf")
        self.cubes.append(self.path+"green_cube.urdf")
        self.cubes.append(self.path+"blue_cube.urdf")
        self.col = 0

        self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.name = "cube_" + str(rospy.Time.now().to_sec())  # Generate a unique name for each cube

    def checkModel(self):
        res = self.ms(self.name+str(" "), "world")
        return res.success

    def getPosition(self):
        res = self.ms(self.name, "world")
        x= res.pose.position.x
        y= res.pose.position.y
        z= res.pose.position.z
        return x,y,z

    def spawnModel(self):
        random.shuffle(self.cubes) 
        cube = self.cubes[self.col]
        with open(cube, "r") as f:
            cube_urdf = f.read()

        # Get the position of the conveyor
        conveyor_pose = self.ms("conveyor", "world")

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        self.name = "cube_" + str(rospy.Time.now().to_sec())  # Generate a unique name for each cube

        # Set the spawn position relative to the conveyor
        spawn_x = conveyor_pose.pose.position.x #+ random.uniform(-0.1, 0.1)  # Randomize x position slightly
        spawn_y = conveyor_pose.pose.position.y - 0.55
        spawn_z = conveyor_pose.pose.position.z + 0.75

        pose = Pose(Point(x=spawn_x, y=spawn_y, z=spawn_z), orient)
        self.sm(self.name, cube_urdf, '', pose, 'world')

        if self.col < 2:
            self.col += 1
        else:
            self.col = 0

        rospy.sleep(1)  # Wait for 1 second after each spawn

    def deleteModel(self, model_name):
        self.dm(model_name)
        rospy.sleep(3)

    def spawnCubes(self, num_cubes):
        for _ in range(num_cubes):
            if self.checkModel() == False:
                self.spawnModel()
                x,y,z = self.getPosition()
                pub = rospy.Publisher('chatter', String, queue_size=10)
                rate = rospy.Rate(10) # 10hz
                while z >= 0.7:
                    hello_x = "x position = %s \n" % x
                    hello_y = "y position = %s \n" % y
                    hello_z = "z position = %s \n" % z
                    # rospy.loginfo(hello_x)
                    # rospy.loginfo(hello_y)
                    # rospy.loginfo(hello_z)
                    x,y,z = self.getPosition()
                    # xyz = [hello_x,hello_y,hello_z]
                    # pub.publish(xyz)
                    pub.publish(hello_y)
                    # pub.publish(hello_z)
                    rate.sleep()
		
           # elif self.getPosition() < 0.05:
               # self.deleteModel("cube")

    def shutdown_hook(self):
        self.deleteModel(self.name)
        print("Shutting down")


if __name__ == "__main__":
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_cubes")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    rospy.wait_for_service("/gazebo/get_model_state")
    cs = CubeSpawner()
    rospy.on_shutdown(cs.shutdown_hook)

    num_cubes_to_spawn = 100  # Change this value to control the number of cubes to be spawned
    cs.spawnCubes(num_cubes_to_spawn)

    rospy.spin()  # Keep the node running after spawning all the cubes

