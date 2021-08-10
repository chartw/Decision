import rospy
from std_msgs.msg import String

class FakeSignal:
    
    def __init__(self):
        self.input = None
        self.text = 'What to do:\n \
                    1. Person detected in vision\n \
                    9. Exit'
        
    def run(self):
        rospy.init_node('GroundControl', anonymous=False)
        rate = rospy.Rate(1) #1hz
        self.pub_detection = rospy.Publisher('/object_detection', String, queue_size=10)

        while not rospy.is_shutdown():

            
            self.input = input(self.text)
            
            
            
            if self.input == '1':
                self.pub_detection.publish('person')
            
            
            elif self.input == '9':
                print("==EXIT==")
                break
            
            rate.sleep()
        
        

if __name__ == '__main__':
    FS = FakeSignal()
    try:
        FS.run()
    except rospy.ROSInterruptException:
        pass