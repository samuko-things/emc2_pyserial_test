import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from pynput.keyboard import Key, Listener







arg_msg = """
enter wheel angular velocity args in format <angVel (between 1.000 to 7.000 rad/sec)>
        """

def process_args_vel():
    angVel = 3.142
    try:
        if len(sys.argv) == 1:
            print(arg_msg)
            print("using default values")
            return angVel
        else:
            print("using entered wheel angular velocity value")
            angVel = float(sys.argv[1])
            return angVel
    except Exception as e:
        print(e)
        print(arg_msg)
        print("using default value")
        return angVel




msg = """
This node takes keypresses from the keyboard 
and publishes them to control the emc2_driver 
via the emc2_test_drive_server_node.
It makes use of the pynput python library

------------------------------------------------
drive around with arrow keys:

                  [forward]
                      |
  [rotate left] -------------- [rotate right]
                      |
                  [reverse] 

stops when no arrow key is pressed

For Holonomic mode (strafing), PRESS ALT key
(press again to return to non holonomic mode).

-------------------------------------------------
"""



class EmcPynputClient(Node):
    def __init__(self):
        super().__init__(node_name="emc2_pynput_client_node") # initialize the node name

        self.defaultAngVel = process_args_vel()
        self.angVelA = 0.000
        self.angVelB = 0.000

        self.send_cmd = self.create_publisher(Float32MultiArray, 'emc2/cmd_vel', 10)
        
        self.status = 0

        self.upPressed = False
        self.downPressed = False
        self.leftPressed = False
        self.rightPressed = False


        # ...or, in a non-blocking fashion:
        listener = Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        # listener.join()

        print(msg)
            


    def publish_cmd_vel(self, angVelA, angVelB):
        cmd_vel = Float32MultiArray()
        cmd_vel.data = [angVelA, angVelB]
        self.send_cmd.publish(cmd_vel)

        self.print_speed()


    def print_speed(self):
        if (self.status == 10):
            print(msg)
        self.status = (self.status + 1) % 11

        print('currently:\tangVelA=%f\tangVelB=%s' % (self.angVelA, self.angVelB))


    # def reset_speed(self):
    #     self.speed = self.default_speed
    #     self.turn = self.default_turn
    #     self.can_print=True


    def on_press(self, key):       
        if key == Key.up:
            if (not self.upPressed) and (not self.downPressed) and (not self.leftPressed) and (not self.rightPressed):
                self.upPressed = True
                self.angVelA = self.defaultAngVel
                self.angVelB = self.defaultAngVel
                self.publish_cmd_vel(self.angVelA, self.angVelB)
                
        if key == Key.down:
            if (not self.upPressed) and (not self.downPressed) and (not self.leftPressed) and (not self.rightPressed):
                self.downPressed = True
                self.angVelA = -self.defaultAngVel
                self.angVelB = -self.defaultAngVel
                self.publish_cmd_vel(self.angVelA, self.angVelB)

        if key == Key.left:
            if (not self.upPressed) and (not self.downPressed) and (not self.leftPressed) and (not self.rightPressed):
                self.leftPressed = True
                self.angVelA = -self.defaultAngVel*0.5
                self.angVelB = self.defaultAngVel*0.5
                self.publish_cmd_vel(self.angVelA, self.angVelB)
                
        if key == Key.right:
            if (not self.upPressed) and (not self.downPressed) and (not self.leftPressed) and (not self.rightPressed):
                self.rightPressed = True
                self.angVelA = self.defaultAngVel*0.5
                self.angVelB = -self.defaultAngVel*0.5
                self.publish_cmd_vel(self.angVelA, self.angVelB)

            
    def on_release(self, key):

        if key == Key.up:
            if (self.upPressed) and (not self.downPressed) and (not self.leftPressed) and (not self.rightPressed):
                self.upPressed = False
                self.angVelA = 0.000
                self.angVelB = 0.000
                self.publish_cmd_vel(self.angVelA, self.angVelB)
                
        if key == Key.down:
            if (not self.upPressed) and (self.downPressed) and (not self.leftPressed) and (not self.rightPressed):
                self.downPressed = False
                self.angVelA = 0.000
                self.angVelB = 0.000
                self.publish_cmd_vel(self.angVelA, self.angVelB)

        if key == Key.left:
            if (not self.upPressed) and (not self.downPressed) and (self.leftPressed) and (not self.rightPressed):
                self.leftPressed = False
                self.angVelA = 0.000
                self.angVelB = 0.000
                self.publish_cmd_vel(self.angVelA, self.angVelB)
     
        if key == Key.right:
            if (not self.upPressed) and (not self.downPressed) and (not self.leftPressed) and (self.rightPressed):
                self.rightPressed = False
                self.angVelA = 0.000
                self.angVelB = 0.000
                self.publish_cmd_vel(self.angVelA, self.angVelB)

        if key == Key.esc:
            # Stop listener
            return False
        






def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    emc_pynput_client = EmcPynputClient()

    # spin the node so the call back function is called
    rclpy.spin(emc_pynput_client)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    emc_pynput_client.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown() 



if __name__=='__main__':
    main()