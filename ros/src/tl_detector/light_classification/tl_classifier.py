from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image
import tensorflow as tf
from keras.applications.vgg16 import VGG16
from cv_bridge import CvBridge, CvBridgeError
import rosbag


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # load in neural network, I think transfer learning might be best?
        self.bridge = CvBridge()
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN

    def train(self):
        # load in faster rnn that's been trained on coco data?
        # https://github.com/rbgirshick/py-faster-rcnn/tree/master/data

        # run on the bags data
        bag1 = rosbag.Bag('/home/pcarney/udacity/traffic_light_bag_files/just_traffic_light.bag')
        # for topic, msg, t in bag1.read_messages(topics=['current_pose', 'image_raw']):
        for topic, msg, t in bag1.read_messages(topics=['image_raw']):
            # msg should be sensor_msgs/Image
            cv2_image = self.bridge.imgmsg_to_cv2(msg, "rgb8") # double check that encoding
            # add to collection so it can be run
            print msg

        bag2 = rosbag.Bag('/home/pcarney/udacity/traffic_light_bag_files/loop_with_traffic_light.bag')
        for topic, msg, t in bag2.read_messages(topics=['current_pose', 'image_raw']):
            print msg

        # get the rectangular coordinates -> highlight and output images
        # -> then label by hand or using sloth

if __name__ == '__main__':
    TLClassifier().train()
