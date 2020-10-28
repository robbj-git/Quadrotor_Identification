import rospy
from dji_sdk.srv import SDKControlAuthority
from sensor_msgs.msg import Joy #, Imu, NavSatFix
from std_msgs.msg import Header
from math import cos, sin, pi
import time

class Experimenter():

    def __init__(self, freq):
        rospy.init_node('UAV_experimenter')
        self.rate = rospy.Rate(freq)
        self.T = 1/freq;
        self.publisher = rospy.Publisher(\
            '/dji12/dji_sdk/flight_control_setpoint_generic', Joy,\
            queue_size = 10)

        print('Waiting for control authority serivce')
        rospy.wait_for_service('/dji12/dji_sdk/sdk_control_authority')
        print('Finished waiting')

        try:
            authority_server = rospy.ServiceProxy(\
                '/dji12/dji_sdk/sdk_control_authority', SDKControlAuthority)
            control_response = authority_server(1)
        except:
            print "Failed reaching control authority. Sleeping for 3s."
            time.sleep(3)

    # ----------------------------- Experiments --------------------------------

    def run_vert_experiment(self, length):
        period = 2*length
        t0 = rospy.get_time()
        while rospy.get_time() - t0 < length:
            if rospy.is_shutdown():
                break
            w_cmd = self.get_w_cmd(t0, period)
            # 0x02 + 0x08  = 0x0a
            axes = [0.0, 0.0, w_cmd, 0.0, 0x0a]
            msg = Joy(Header(), axes, [])
            self.publisher.publish(msg)
            self.rate.sleep()

    def run_phi_experiment(self, length):
        period = 2*length
        t0 = rospy.get_time()
        while rospy.get_time() - t0 < length:
            if rospy.is_shutdown():
                break
            phi_cmd = self.get_phi_cmd(t0, period)
            # 0x02 + 0x08  = 0x0a
            axes = [phi_cmd, 0.0, 0.0, 0.0, 0x0a]
            msg = Joy(Header(), axes, [])
            self.publisher.publish(msg)
            self.rate.sleep()

    def run_theta_experiment(self, length):
        period = 2*length
        t0 = rospy.get_time()
        while rospy.get_time() - t0 < length:
            if rospy.is_shutdown():
                break
            theta_cmd = self.get_theta_cmd(t0, period)
            # 0x02 + 0x08  = 0x0a
            axes = [0.0, theta_cmd, 0.0, 0.0, 0x0a]
            msg = Joy(Header(), axes, [])
            self.publisher.publish(msg)
            self.rate.sleep()

    def run_psi_dot_experiment(self, length):
        period = 2*length
        t0 = rospy.get_time()
        while rospy.get_time() - t0 < length:
            if rospy.is_shutdown():
                break
            psi_dot_cmd = self.get_psi_dot_cmd(t0, period)
            # 0x02 + 0x08  = 0x0a
            axes = [0.0, 0.0, 0.0, psi_dot_cmd, 0x0a]
            msg = Joy(Header(), axes, [])
            self.publisher.publish(msg)
            self.rate.sleep()

    def run_complete_experiment(self, length):
        period = 2*length
        t0 = rospy.get_time()
        while rospy.get_time() - t0 < length:
            if rospy.is_shutdown():
                break
            w_cmd = self.get_w_cmd(t0, period)
            phi_cmd = self.get_phi_cmd(t0, period)
            theta_cmd = self.get_theta_cmd(t0, period)
            psi_dot_cmd = self.get_psi_dot_cmd(t0, period)
            # 0x02 + 0x08  = 0x0a
            axes = [phi_cmd, theta_cmd, w_cmd, psi_dot_cmd, 0x0a]
            msg = Joy(Header(), axes, [])
            self.publisher.publish(msg)
            self.rate.sleep()

    # -------------------------- Input Generators ------------------------------

    def get_w_cmd(self, t0, period):
        t = rospy.get_time()
        A1 = 2
        A2 = 0.2
        return A1*cos( 2*pi*(t-t0)/period ) + A2*sin(12*2*pi*t/period)\
        + A2*sin(15*2*pi*t/period + 2) + A2*sin(18*2*pi*t/period + 1.3)\
        + A2*sin(21*2*pi*t/period + 0.78) + A2*sin(25*2*pi*t/period + 2.7)\
        + A2*sin(30*2*pi*t/period + 3.2) + A2*sin(33*2*pi*t/period + 0.2)

    def get_phi_cmd(self, t0, period):
        t = rospy.get_time()
        A1 = 0.3
        A2 = 0.02
        return A1*cos( 2*pi*(t-t0)/period ) + A2*sin(12*2*pi*t/period)\
        + A2*sin(15*2*pi*t/period + 2) + A2*sin(18*2*pi*t/period + 1.3)\
        + A2*sin(21*2*pi*t/period + 0.78) + A2*sin(25*2*pi*t/period + 2.7)\
        + A2*sin(30*2*pi*t/period + 3.2) + A2*sin(33*2*pi*t/period + 0.2)

    def get_theta_cmd(self, t0, period):
        t = rospy.get_time()
        A1 = 0.3
        A2 = 0.02
        return A1*cos( 2*pi*(t-t0)/period ) + A2*sin(12*2*pi*t/period)\
        + A2*sin(15*2*pi*t/period + 2) + A2*sin(18*2*pi*t/period + 1.3)\
        + A2*sin(21*2*pi*t/period + 0.78) + A2*sin(25*2*pi*t/period + 2.7)\
        + A2*sin(30*2*pi*t/period + 3.2) + A2*sin(33*2*pi*t/period + 0.2)

    def get_psi_dot_cmd(self, t0, period):
        t = rospy.get_time()
        A1 = 1
        A2 = 0.1
        return A1*cos( 2*pi*(t-t0)/period ) + A2*sin(12*2*pi*t/period)\
        + A2*sin(15*2*pi*t/period + 2) + A2*sin(18*2*pi*t/period + 1.3)\
        + A2*sin(21*2*pi*t/period + 0.78) + A2*sin(25*2*pi*t/period + 2.7)\
        + A2*sin(30*2*pi*t/period + 3.2) + A2*sin(33*2*pi*t/period + 0.2)
