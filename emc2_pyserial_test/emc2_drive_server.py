
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from emc2_arduino_serial_comm import EMC2ArduinoSerialCommApi
from time import sleep



class EmcDriveServer(Node):

    def __init__(self):
        super().__init__('emc2_drive_server_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/emc2/cmd_vel',
            self.emc_drive_server_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Float32MultiArray, 'emc2/wheeldata', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.emc = EMC2ArduinoSerialCommApi('/dev/ttyUSB0')
        sleep(4.0)

        self.readAngVelA = 0.000
        self.readAngVelB = 0.000
        self.readAngPosA = 0.000
        self.readAngPosB = 0.000

        self.angVelA = 0.000 #rad/sec
        self.angVelB = 0.000 #rad/sec
        self.emc.sendTargetVel(self.angVelA, self.angVelB)



    def emc_drive_server_callback(self, cmd_vel):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        self.angVelA = cmd_vel.data[0]
        self.angVelB = cmd_vel.data[1]
        self.emc.sendTargetVel(self.angVelA, self.angVelB)

    def timer_callback(self):
        try:
          msg = Float32MultiArray()
          angPosA, angPosB = self.emc.getMotorsPos() # returns angPosA, angPosB
          angVelA, angVelB = self.emc.getMotorsVel() # returns angVelA, angVelB
          msg.data = [angPosA, angVelA, angPosB, angVelB]
          self.publisher_.publish(msg)
          self.get_logger().info("motorA_readings: [%f, %f]\nmotorB_readings: [%f, %f]\n" % (msg.data[0], msg.data[1], msg.data[2], msg.data[3]))
        except:
          pass
        


def main(args=None):
    rclpy.init(args=args)

    emc_drive_server = EmcDriveServer()

    rclpy.spin(emc_drive_server)

    emc_drive_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
