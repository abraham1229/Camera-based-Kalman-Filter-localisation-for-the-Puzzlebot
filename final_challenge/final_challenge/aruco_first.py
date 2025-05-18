import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from cv2 import aruco

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.markerLength = 0.09
        self.aruco_type = "DICT_4X4_250"
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.get_logger().info('ArucoDetectorNode started, waiting for images on /image_raw')

    def listener_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(img)

        if len(markerCorners) > 0:
            cv.aruco.drawDetectedMarkers(img, markerCorners, markerIds)
            ids = markerIds.flatten()
            for (markerCorner, markerID) in zip(markerCorners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv.circle(img, (cX, cY), 4, (0, 0, 255), -1)

        cv.imshow("Aruco Detection", img)
        k = cv.waitKey(1)
        if k%256 == 27:
            self.get_logger().info("Escape hit, shutting down node...")
            rclpy.shutdown()
            cv.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
