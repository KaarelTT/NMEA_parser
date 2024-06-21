#!/home/kaarel/anaconda3/envs/ros_pyvisa/bin/python

import rospy
import pyvisa
from std_msgs.msg import Float32


def read_oscilloscope():
    rm = pyvisa.ResourceManager()
    oscilloscope = rm.open_resource(
        'ASRL/dev/ttyS4::INSTR')  # Replace with your oscilloscope's resource string

    rospy.init_node('oscilloscope_reader', anonymous=True)
    pub_ch1 = rospy.Publisher('oscilloscope/ch1', Float32, queue_size=10)
    pub_ch2 = rospy.Publisher('oscilloscope/ch2', Float32, queue_size=10)
    pub_ch3 = rospy.Publisher('oscilloscope/ch3', Float32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            ch1_data = float(oscilloscope.query(':MEAS:VAV? CHAN1'))
            ch2_data = float(oscilloscope.query(':MEAS:VAV? CHAN2'))
            ch3_data = float(oscilloscope.query(':MEAS:VAV? CHAN3'))

            rospy.loginfo(f"CH1: {ch1_data}, CH2: {ch2_data}, CH3: {ch3_data}")

            pub_ch1.publish(ch1_data)
            pub_ch2.publish(ch2_data)
            pub_ch3.publish(ch3_data)

        except Exception as e:
            rospy.logerr(f"Error reading oscilloscope: {e}")

        rate.sleep()


if __name__ == '__main__':
    try:
        read_oscilloscope()
    except rospy.ROSInterruptException:
        pass