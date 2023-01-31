#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
from math import sin, pi
from std_msgs.msg import Float64, String
from lab1.msg import gps
import utm

if __name__ == "__main__":
    sensor_name = "gps"
    rospy.init_node("gps")
    port_name = "/dev/ttyUSB0"
    baud_rate = 4800

    serial_port = rospy.get_param("~port", port_name)
    serial_baud = rospy.get_param("~baudrate", baud_rate)

    port = serial.Serial(serial_port, serial_baud, timeout=3.)

    rospy.logdebug("Using gps sensor on port " + serial_port + " at " + str(serial_baud))
        
    gps_pub = rospy.Publisher(sensor_name, gps, queue_size=10)
    
    rospy.logdebug("Initialization complete")

    msg = gps()

    try:
        while not rospy.is_shutdown():
            try:
                line = port.readline()
                line = line.decode()
                
                if line == "":
                    continue
                
                data = line.split(",")
            
            except:
                continue
            
            if line.startswith("$GNGGA"):

                if (data[2] == "" or data[4] == "" or data[9] == ""):
                    rospy.logwarn("GPS: No data")
                
                else:
                
                    flag_latitude_direction = 1
                    flag_longitude_direction = 1

                    latitude_raw = str(data[2])
                    latitude_direction = str(data[3])
                    latitude_degree = float(latitude_raw[:2])
                    latitude_minutes = float(latitude_raw[2:])

                    longitude_raw = str(data[4])
                    longitude_direction = str(data[5])
                    longitude_degree = float(longitude_raw[:3])
                    longitude_minutes = float(longitude_raw[3:])

                    if(latitude_direction == "S"):
                        flag_latitude_direction = -1
                    
                    if(longitude_direction == "W"):
                        flag_longitude_direction = -1

                    latitude = float(flag_latitude_direction*((latitude_degree) + (latitude_minutes/60)))
                    longitude = float(flag_longitude_direction*((longitude_degree) + (longitude_minutes/60)))
    
                    altitude = float(data[9])
                    quality = data[6]

                    data_utm = utm.from_latlon(latitude, longitude)

                    utm_easting = data_utm[0]
                    utm_northing = data_utm[1]
                    utm_zone_number = data_utm[2]
                    utm_zone_letter = data_utm[3]

                    msg.header.stamp = rospy.get_rostime()
                    msg.Latitude = latitude
                    msg.Longitude = longitude
                    msg.Altitude = altitude
                    msg.UTM_Easting = utm_easting
                    msg.UTM_Northing = utm_northing
                    msg.Zone = utm_zone_number
                    msg.Letter = utm_zone_letter
                    msg.quality = quality

                    gps_pub.publish(msg)

    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down the gps node...")
