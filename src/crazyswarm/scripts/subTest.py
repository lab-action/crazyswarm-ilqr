
import rospy
from std_msgs.msg import String

def callback(data):
    print("Data Received!")

rospy.init_node('Guidance', anonymous=True)

rospy.Subscriber("/tf", String, callback)

rospy.spin()
