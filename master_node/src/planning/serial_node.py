import serial
import rospy
import struct

from master_node.msg import Serial_Info


class Serial_Node:
    def __init__(self):
        # Serial Connect
        self.ser = serial.Serial("/dev/ttyUSB0", 115200)
        
        # ROS Publish
        rospy.init_node("Serial", anonymous=False)
        serial_pub = rospy.Publisher("/serial", Serial_Info, queue_size=1)
                    
        # ROS Subscribe        
        def controlCallback(self, msg): self.control_input = msg
        rospy.Subscriber("/control", Serial_Info, controlCallback)

        # Messages/Data
        self.serial_msg = Serial_Info()
        self.control_input = Serial_Info()
        self.serial_data = []
        
        # Main Loop
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.serialRead()
            self.serialCal()
            serial_pub.publish(self.serial_msg)
            self.serialWrite()
            
            rate.sleep()

    def serialRead(self):
        serial_input = self.ser.readline()
        self.ser.flushInput()

        if (
            serial_input[0] is 0x53
            and serial_input[1] is 0x54
            and serial_input[2] is 0x58
        ):
            while True:
                for i in range(len(serial_input)):
                    if serial_input[i] is 0x0A and i is not 17:
                        # print("### 0x0A Found!", i, "th data")
                        self.serial_data.append(0x0B)
                    else:
                        self.serial_data.append(serial_input[i])

                if len(self.serial_data) < 18:
                    serial_input = self.ser.readline()
                else:
                    break

            # cnt=int(self.serial_data[15])
            self.V_veh = int(self.serial_data[6])
            
    def serialCal(self):
        self.serial_msg.auto_manual =  int(self.serial_data[3])
        self.serial_msg.emergency_stop = int(self.serial_data[4])
        self.serial_msg.gear = int(self.serial_data[5])
        self.serial_msg.speed = int(
                                    self.serial_data[6]
                                    + 256*self.seria_data[7]      
                                    )

        self.serial_msg.steer = int(
                                    self.serial_data[8]
                                    + 256*self.seria_data[7]      
                                    )
        self.serial_msg.brake = int(self.serial_data[10])
        self.serial_msg.encoder = float(
                                        self.serial_data[11]
                                        + 256*self.serial_data[12]
                                        + 65536*self.serial_data[13]
                                        + 16777216*self.serial_data[14]
                                        )

    def serialWrite(self):
        steering = self.control_data["steering"]
        break_val = self.control_data["break_val"]
        cnt = 0x00
        result = struct.pack(
            "!BBBBBBHhBBBB",
            0x53,
            0x54,
            0x58,
            0x01,
            self.control_input.emergency_stop,
            self.control_input.gear,
            self.control_input.speed,
            self.control_input.steer,
            self.control_input.brake,
            cnt,
            0x0D,
            0x0A,
        )  # big endian 방식으로 타입에 맞춰서 pack

        self.ser.write(result)

    def controlCallback(self, msg):
        self.control_input = msg