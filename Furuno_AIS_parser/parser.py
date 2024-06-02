#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32,Int32, String
import socket



def udp_listener():
    rospy.init_node('udp_listener', anonymous=True)

    # Publishers for Furuno SCX20 data
    message_pub = rospy.Publisher('message', String, queue_size=10)
    heading_pub = rospy.Publisher('heading', Float32, queue_size=10)
    pitch_pub = rospy.Publisher('pitch', Float32, queue_size=10)
    yaw_pub = rospy.Publisher('yaw', Float32, queue_size=10)
    roll_pub = rospy.Publisher('roll', Float32, queue_size=10)
    latitude_pub = rospy.Publisher('latitude', Float32, queue_size=10)
    longitude_pub = rospy.Publisher('longitude', Float32, queue_size=10)
    speedkmh_pub = rospy.Publisher('speedkmh', Float32, queue_size=10)
    speedknots_pub = rospy.Publisher('speedknots', Float32, queue_size=10)
    truecourse_pub = rospy.Publisher('truecourse', Float32, queue_size=10)
    heave_pub = rospy.Publisher('heave', Float32, queue_size=10)

    rate = rospy.Rate(50)  # 50hz

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
            message_pub.publish(message)


            #rospy.loginfo("Received packet from %s:%d", addr[0], addr[1])
            #rospy.loginfo("Packet data: %s", message)
            # Parse and publish specific data based on NMEA sentence type

            #HEADING
            if message.startswith('$HEHDT'):
                parts = message.split(',')
                heading = float(parts[1])
                heading_pub.publish(heading)
                rospy.loginfo("Heading %f", heading)

            #ROLL,PITCH,YAW
            elif message.startswith('$YXXDR'):
                parts = message.split(',')

                if parts[1] == 'A':
                    yaw = float(parts[2])
                    pitch = float(parts[6])
                    roll = float(parts[10])

                    yaw_pub.publish(yaw)
                    pitch_pub.publish(pitch)
                    roll_pub.publish(roll)
                    rospy.loginfo("Attitude Roll:%f, Pitch:%f, Yaw:%f", roll,pitch,yaw)
                elif parts[1] == 'D':
                    heave = float(parts[2])
                    heave_pub.publish(heave)
                    rospy.loginfo("Heave:%f", heave)

            #LATITUDE, LONGITUDE
            elif message.startswith('$GPGGA'):
                parts = message.split(',')

                # Extract latitude and longitude strings
                latitude_str = parts[2]
                latitude_direction = parts[3]
                longitude_str = parts[4]
                longitude_direction = parts[5]

                # Convert latitude and longitude to float
                # Latitude: DDMM.MMMMM -> DD + MM.MMMMM / 60
                latitude_deg = int(latitude_str[:2])
                latitude_min = float(latitude_str[2:])
                latitude = latitude_deg + (latitude_min / 60.0)

                # If the latitude direction is South, make the latitude negative
                if latitude_direction == 'S':
                    latitude = -latitude

                # Longitude: DDDMM.MMMMM -> DDD + MM.MMMMM / 60
                longitude_deg = int(longitude_str[:3])
                longitude_min = float(longitude_str[3:])
                longitude = longitude_deg + (longitude_min / 60.0)

                # If the longitude direction is West, make the longitude negative
                if longitude_direction == 'W':
                    longitude = -longitude

                latitude_pub.publish(latitude)
                longitude_pub.publish(longitude)
                rospy.loginfo("Latitude:%f, Longitude:%f", latitude, longitude)

            elif message.startswith('$GPVTG'):
                parts = message.split(',')

                # Extract the true course and speeds
                true_course = float(parts[1])
                ground_speed_knots = float(parts[5])
                ground_speed_kmh = float(parts[7])
                speedkmh_pub.publish(ground_speed_kmh)
                speedknots_pub.publish(ground_speed_knots)
                truecourse_pub.publish(true_course)
                rospy.loginfo("Speed (knots):%f, True course:%f", ground_speed_knots, true_course)

        except Exception as e:
            rospy.logerr("Error: %s", str(e))

        rate.sleep()


if __name__ == '__main__':
    try:
        udp_listener()
    except rospy.ROSInterruptException:
        pass