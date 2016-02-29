import rospy
from msgs.msg import DriveStatus

class ArlobotDriveStatusPublisher:
    def __init__(self, queue_size=10):
        self._publisher = rospy.Publisher('drive_status', DriveStatus, queue_size=queue_size)

    def _msg(self, left_speed, right_speed, left_count, right_count):
        msg = DriveStatus()
        msg.left_speed = left_speed
        msg.right_speed = right_speed
        msg.delta_left_count = left_count
        msg.delta_right_count = right_count

        return msg

    def Publish(self, left_speed, right_speed, left_count, right_count):
        self._publisher.publish(self._msg(left_speed, right_speed, left_count, right_count))
