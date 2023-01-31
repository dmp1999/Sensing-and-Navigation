#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import serial
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import quaternion_from_euler


if __name__ == "__main__":
    sensor_name = "imu"
    rospy.init_node("imu")
    port_name = "/dev/ttyUSB0"
    baud_rate = 115200

    serial_port = rospy.get_param("~port", port_name)
    serial_baud = rospy.get_param("~baudrate", baud_rate)

    port = serial.Serial(serial_port, serial_baud, timeout=3.)

    rospy.logdebug("Using imu sensor on port " + serial_port + " at " + str(serial_baud))
        
    imu_pub = rospy.Publisher(sensor_name+'/imu_data', Imu, queue_size=10)
    magnetic_pub = rospy.Publisher(sensor_name+'/magnetic_data', MagneticField, queue_size=10)   

    rospy.logdebug("Initialization complete")
    
    imu_msg = Imu()
    magnetic_msg = MagneticField()

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
            
            if line.startswith("$VNYMR"):
                
                    yaw = np.deg2rad(float(data[1]))
                    pitch = np.deg2rad(float(data[2]))
                    roll = np.deg2rad(float(data[3]))
                    magnetic_x = float(data[4])
                    magnetic_y = float(data[5])
                    magnetic_z = float(data[6])
                    accelration_x = float(data[7])
                    accelration_y = float(data[8])
                    accelration_z = float(data[9])
                    gyro_x = float(data[10])
                    gyro_y = float(data[11])
                    gyro_z = float(data[12][0:10])
                    
                    quaternion = quaternion_from_euler(roll, pitch, yaw)

                    q1 = quaternion[0]
                    q2 = quaternion[1]
                    q3 = quaternion[2]
                    q4 = quaternion[3]
             
                    imu_msg.orientation.x=  q1
                    imu_msg.orientation.y = q2
                    imu_msg.orientation.z = q3
                    imu_msg.orientation.w = q4

                    imu_msg.linear_acceleration.x = accelration_x
                    imu_msg.linear_acceleration.y = accelration_y
                    imu_msg.linear_acceleration.z = accelration_z

                    imu_msg.angular_velocity.x = gyro_x
                    imu_msg.angular_velocity.y = gyro_y
                    imu_msg.angular_velocity.z = gyro_z

                    magnetic_msg.magnetic_field.x = magnetic_x
                    magnetic_msg.magnetic_field.y = magnetic_y
                    magnetic_msg.magnetic_field.z = magnetic_z

                    print (imu_msg.orientation.x, imu_msg.linear_acceleration.x, imu_msg.angular_velocity.x , magnetic_msg.magnetic_field.x)
                 
                    imu_pub.publish(imu_msg)
                    magnetic_pub.publish(magnetic_msg)

    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down the imu node...")
