#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket


def udp_listener():
    rospy.init_node('udp_listener', anonymous=True)

    # Publishers for Furuno SCX20 data
    raw_pub = rospy.Publisher('chatter', String, queue_size=10)
    yaw_pub = rospy.Publisher('yaw', String, queue_size=10)
    pitch_pub = rospy.Publisher('pitch', String, queue_size=10)
    roll_pub = rospy.Publisher('roll', String, queue_size=10)
    heave_pub = rospy.Publisher('heave', String, queue_size=10)
    heading_pub = rospy.Publisher('heading', String, queue_size=10)
    rate_of_turn_pub = rospy.Publisher('rate_of_turn', String, queue_size=10)
    air_temp_pub = rospy.Publisher('air_temp', String, queue_size=10)
    baro_press_pub = rospy.Publisher('baro_press', String, queue_size=10)

    # Publishers for AIS data
    ais_ships_pub = rospy.Publisher('ais_ships', String, queue_size=10)
    ais_status_pub = rospy.Publisher('ais_status', String, queue_size=10)

    # Publishers for additional NMEA data
    vtgs_pub = rospy.Publisher('vtgs', String, queue_size=10)
    gpgga_pub = rospy.Publisher('gpgga', String, queue_size=10)

    rate = rospy.Rate(10)  # 10hz

    # Set up the UDP socket for both Furuno SCX20 and em-trak B921 AIS
    UDP_IP = "0.0.0.0"  # Listen on all interfaces
    UDP_PORT = 10110
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    rospy.loginfo("UDP Listener started on port %d", UDP_PORT)

    while not rospy.is_shutdown():
        try:
            # Read data from UDP socket
            data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
            message = data.decode('utf-8').strip()
            rospy.loginfo("Received packet from %s:%d", addr[0], addr[1])
            rospy.loginfo("Packet data: %s", message)

            # Publish raw data
            raw_pub.publish(message)

            # Parse and publish specific data based on NMEA sentence type
            if message.startswith('$YXXDR'):
                parts = message.split(',')
                for i in range(0, len(parts) - 1, 5):
                    if parts[i + 1] == 'A':
                        if parts[i + 3] == 'Yaw':
                            yaw_pub.publish(parts[i + 2])
                        elif parts[i + 3] == 'Pitch':
                            pitch_pub.publish(parts[i + 2])
                        elif parts[i + 3] == 'Roll':
                            roll_pub.publish(parts[i + 2])
            elif message.startswith('$HEHDT'):
                parts = message.split(',')
                heading_pub.publish(parts[1])
            elif message.startswith('$TIROT'):
                parts = message.split(',')
                rate_of_turn_pub.publish(parts[1])
            elif message.startswith('$WIXDR'):
                parts = message.split(',')
                for i in range(0, len(parts) - 1, 5):
                    if parts[i + 1] == 'C':
                        air_temp_pub.publish(parts[i + 2])
                    elif parts[i + 1] == 'P':
                        baro_press_pub.publish(parts[i + 2])
            elif message.startswith('$YXXDR,D'):
                parts = message.split(',')
                heave_pub.publish(parts[2])
            elif message.startswith('!AIVDM') or message.startswith('!AIVDO'):
                # AIS specific sentences
                ais_ships_pub.publish(message)
                ais_status_pub.publish("AIS module operational")
            elif message.startswith('$GPVTG'):
                # Handle GPVTG sentence here
                vtgs_pub.publish(message)
            elif message.startswith('$GPGGA'):
                # Handle GPGGA sentence here
                gpgga_pub.publish(message)
            elif message.startswith('$VDVBW'):
                # Handle VDVBW sentence here
                rospy.loginfo("VDVBW sentence: %s", message)
            else:
                rospy.logwarn("Unrecognized NMEA sentence: %s", message)

        except Exception as e:
            rospy.logerr("Error: %s", str(e))
            ais_status_pub.publish("AIS module error: " + str(e))

        rate.sleep()


if __name__ == '__main__':
    try:
        udp_listener()
    except rospy.ROSInterruptException:
        pass
