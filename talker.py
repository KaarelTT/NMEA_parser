#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket


def udp_listener():
    rospy.init_node('udp_listener', anonymous=True)
    raw_pub = rospy.Publisher('chatter', String, queue_size=10)
    yaw_pub = rospy.Publisher('yaw', String, queue_size=10)
    pitch_pub = rospy.Publisher('pitch', String, queue_size=10)
    roll_pub = rospy.Publisher('roll', String, queue_size=10)
    heave_pub = rospy.Publisher('heave', String, queue_size=10)
    heading_pub = rospy.Publisher('heading', String, queue_size=10)
    rate_of_turn_pub = rospy.Publisher('rate_of_turn', String, queue_size=10)
    air_temp_pub = rospy.Publisher('air_temp', String, queue_size=10)
    baro_press_pub = rospy.Publisher('baro_press', String, queue_size=10)

    rate = rospy.Rate(10)  # 10hz

    # Set up the UDP socket
    UDP_IP = "0.0.0.0"  # Listen on all interfaces
    UDP_PORT = 10110
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    rospy.loginfo("UDP Listener started on port %d", UDP_PORT)

    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
        rospy.loginfo("Received packet from %s:%d", addr[0], addr[1])
        rospy.loginfo("Packet data: %s", data)

        # Publish raw data
        rospy.loginfo("Publishing raw data")
        raw_pub.publish(data.decode('utf-8'))

        # Parse and publish specific data
        message = data.decode('utf-8').strip()
        if message.startswith('$YXXDR'):
            parts = message.split(',')
            for i in range(0, len(parts) - 1, 5):
                if parts[i + 1] == 'A':
                    if parts[i + 3] == 'Yaw':
                        rospy.loginfo("Publishing Yaw data")
                        yaw_pub.publish(parts[i + 2])
                    elif parts[i + 3] == 'Pitch':
                        rospy.loginfo("Publishing Pitch data")
                        pitch_pub.publish(parts[i + 2])
                    elif parts[i + 3] == 'Roll':
                        rospy.loginfo("Publishing Roll data")
                        roll_pub.publish(parts[i + 2])
        elif message.startswith('$HEHDT'):
            parts = message.split(',')
            rospy.loginfo("Publishing Heading data")
            heading_pub.publish(parts[1])
        elif message.startswith('$TIROT'):
            parts = message.split(',')
            rospy.loginfo("Publishing Rate of Turn data")
            rate_of_turn_pub.publish(parts[1])
        elif message.startswith('$WIXDR'):
            parts = message.split(',')
            for i in range(0, len(parts) - 1, 5):
                if parts[i + 1] == 'C':
                    rospy.loginfo("Publishing Air Temperature data")
                    air_temp_pub.publish(parts[i + 2])
                elif parts[i + 1] == 'P':
                    rospy.loginfo("Publishing Barometric Pressure data")
                    baro_press_pub.publish(parts[i + 2])
        elif message.startswith('$YXXDR,D'):
            parts = message.split(',')
            rospy.loginfo("Publishing Heave data")
            heave_pub.publish(parts[2])

        rate.sleep()


if __name__ == '__main__':
    try:
        udp_listener()
    except rospy.ROSInterruptException:
        pass
