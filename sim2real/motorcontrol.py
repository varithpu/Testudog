#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import math
from motor_driver.canmotorlib import CanMotorController
import time

'''
Here are the things to run after connecting the motor:
ip link show
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
'''

class Controller:
    def __init__(self):
        self.knee_angle = None
        self.pitch_angle = None

    def knee_callback(self,data):
        self.knee_angle = math.degrees(data.data) + 90
        rospy.loginfo("knee angle: %s", self.knee_angle)
        if self.knee_angle >= 60:
            self.knee_angle = 60
        if self.knee_angle <= -60:
            self.knee_angle = -60

    def pitch_callback(self,data):
        self.pitch_angle = math.degrees(data.data)-30
        #rospy.loginfo("pitch angle: %s", self.pitch_angle)
        if self.pitch_angle >= 30:
            self.pitch_angle = 30
        if self.pitch_angle <= -30:
            self.pitch_angle = -30
        p_knee,v_knee,c_knee = motor_knee.send_deg_command(int(self.knee_angle), 30, 8, .5, 3)
        p_pitch,v_pitch,c_pitch = motor_pitch.send_deg_command(int(self.pitch_angle), 30, 8, .5, 3)

def motorpos_sub():
    controller = Controller()
    rospy.init_node('motorpos_sub_node')
    rospy.Subscriber("testudog_controller/front_left_knee_controller/command", Float64, controller.knee_callback)
    rospy.Subscriber("testudog_controller/front_left_pitching_controller/command", Float64, controller.pitch_callback)
    rospy.spin()
    motor_knee.disable_motor()
    motor_pitch.disable_motor()

if __name__ == '__main__':
    motor_knee = CanMotorController(can_socket='can0', motor_id=0x03, motor_type='AK80_6_V2', socket_timeout=0.5)
    motor_pitch = CanMotorController(can_socket='can0', motor_id=0x02, motor_type='AK80_6_V2', socket_timeout=0.5)
    motor_knee.set_zero_position()
    motor_pitch.set_zero_position()
    motor_knee.enable_motor()
    motor_pitch.enable_motor()

    start_time = time.time()
    while(time.time() - start_time < 5.0):
        pass
    motorpos_sub()
 
