

import roslib
roslib.load_manifest('tld_msgs')
import rospy

from std_msgs.msg import Float32, Float32MultiArray, Char
from tld_msgs.msg import BoundingBox, Target
from sensor_msgs.msg import CompressedImage

NUM_OF_TRACKERS = 4

class TLD_TRACKER_HELPER(object):
    def __init__(self, id):
        self.id = id

        rospy.Subscriber('/tld_tracked_object' + str(id), BoundingBox, self.tracked_object_callback)
        rospy.Subscriber('/tld_fps' + str(id), Float32, self.fps_callback)
        self.pub_gui_cmds = rospy.Publisher('/tld_gui_cmds' + str(id), Char, queue_size=10)
        self.pub_gui_bb = rospy.Publisher('/tld_gui_bb' + str(id), Target, queue_size=10)
        self.bb = None
        self.fps = 0

    def tracked_object_callback(self, data):
        self.bb = (data.x, data.y, data.width, data.height, data.confidence)

    def fps_callback(self, data):
        self.fps = data

    def is_idle(self):
        return self.bb is None

    def set_target(self, x, y, width, height, img):
        target = Target()
        target.bb.x = x
        target.bb.y = y
        target.bb.width = width
        target.bb.height = height
        target.bb.confidence = 1.0
        target.img = img
        self.pub_gui_cmds.publish(114)  # 'r' stands for reset, following TLD's convention
        self.pub_gui_bb.publish(target)

    def reset_target(self):
        self.pub_gui_cmds.publish(114)

class TLD_MODULE(object):
    def __init__(self):

        # the image is not used, this is for publishing at the same rate
        rospy.Subscriber('/bebop/image_raw/compressed', CompressedImage, self.image_compressed_callback, queue_size=1)

        # one copy of publishers for GUI
        self.pub_tracked_objects = rospy.Publisher('fleye/tld_tracked_objects', Float32MultiArray, queue_size=10)  # format is [x0,y0,w0,h0,c0,...]
        self.pub_fps = rospy.Publisher('fleye/tld_fps', Float32MultiArray, queue_size=10)  # format is [fps0, ...]

        # multiple helpers for TLD_trackers
        self.trackers = [TLD_TRACKER_HELPER(i) for i in range(NUM_OF_TRACKERS)]

    def image_compressed_callback(self, unused_data):
        fps = Float32MultiArray()
        bb = Float32MultiArray()

        for tracker in self.trackers:
            fps.data.append(tracker.fps)
            for i in range(5):
                bb.data.append(tracker.bb[i] if not tracker.is_idle() else 0)

        self.pub_fps.publish(fps)
        self.pub_tracked_objects.publish(bb)

    def get_idle_tracker(self):
        for tracker in self.trackers:
            if tracker.is_idle():
                return tracker
        return None

    # return tracker id if an idle tracker was found otherwise return -1
    def select_target(self, x, y, width, height, img):
        idle_tracker = self.get_idle_tracker()
        if idle_tracker is None:
            print "Max number of targets is 4"
            return -1
        idle_tracker.set_target(x, y, width, height, img)
        print "sign target to tracker", idle_tracker.id
        return idle_tracker.id

    def cancel_target(self, id):
        self.trackers[id].reset_target()
        print "target cancel at tracker", id