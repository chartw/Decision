import serial
import rospy
import struct

from master_node.msg import Serial_Info


class Serial_Node:
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyUSB0", 115200)

        rospy.init_node("Serial", anonymous=False)
        serial_info_pub = rospy.Publisher("/serial", Serial_Info, queue_size=1)

        rospy.Subscriber("/control", Serial_Info, self.controlCallback)
        self.serial_info = Serial_Info()

        self.serial_data = []
        rate = rospy.Rate(100)
        # main loop
        while not rospy.is_shutdown():
            self.serialRead()
            
            self.serialWrite()
            rate.sleep()

    def serialRead(self):
        # serial_data를 가공하여 어디에 저장할지가 관건?

        serial_input = self.ser.readline()
        self.ser.flushInput()
        # print(serial_input)
        # print(serial_input[0])

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
            0x00,
            0x00,
            int(self.control_data["speed"]),
            self.control_data["steering"],
            self.control_data["break_val"],
            cnt,
            0x0D,
            0x0A,
        )  # big endian 방식으로 타입에 맞춰서 pack

        self.ser.write(result)

    def controlCallback(self, msg):
        self.serial_info = msg
