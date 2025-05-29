import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
from cv2 import aruco

class ArucoVisualizeNode(Node):
    def __init__(self):
        super().__init__('aruco_visualizer_node')

        # --- Calibración hardcodeada ---
        self.K = np.array([
            [650.123, 0.0, 320.456],
            [0.0, 649.789, 240.123],
            [0.0, 0.0, 1.0]
        ])
        self.D = np.array([0.12, -0.23, 0.001, 0.002, 0.05])

        self.markerLength = 0.06  # 6 cm

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

        self.bridge = CvBridge()

        self.subscription_image = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.get_logger().info("Aruco visualize node started (solo visualización, distancia incluida)")

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejected = self.detector.detectMarkers(gray)

        if markerIds is not None and len(markerIds) > 0:
            aruco.drawDetectedMarkers(img, markerCorners, markerIds)
            for i in range(len(markerIds)):
                rvec, tvec = aruco.estimatePoseSingleMarkers(
                    markerCorners[i], self.markerLength, self.K, self.D)[:2]
                cv.drawFrameAxes(img, self.K, self.D, rvec, tvec, 0.01)

                # --- Calcular y dibujar distancia ---
                distance = np.linalg.norm(tvec[0][0])
                corners = markerCorners[i].reshape((4, 2))
                topLeft, _, bottomRight, _ = corners
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv.putText(img, f"{distance:.2f} m", (cX, cY - 15),
                           cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)

        cv.imshow("Aruco Detection", img)
        key = cv.waitKey(1)
        if key == 27:  # ESC
            self.get_logger().info("ESC pressed, closing visualization window.")
            cv.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoVisualizeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
