import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
from cv2 import aruco

from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Calibración hardcodeada
        self.K = np.array([
            [650.123, 0.0, 320.456],
            [0.0, 649.789, 240.123],
            [0.0, 0.0, 1.0]
        ])
        self.D = np.array([0.12, -0.23, 0.001, 0.002, 0.05])
        self.markerLength = 0.06

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

        self.bridge = CvBridge()

        self.subscription_image = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.publisher_aruco = self.create_publisher(
            ArucoDetection,
            '/aruco_detections',
            10)

        self.get_logger().info("Aruco detector node started (publicando MarkerPose en array)")

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejected = self.detector.detectMarkers(gray)

        # Preparar mensaje ArucoDetection
        detection_msg = ArucoDetection()
        detection_msg.header = msg.header
        # detection_msg.frame_id = msg.header.frame_id
        detection_msg.markers = []

        if markerIds is not None and len(markerIds) > 0:
            aruco.drawDetectedMarkers(img, markerCorners, markerIds)
            for i, marker_id in enumerate(markerIds.flatten()):
                # Pose estimation (en el frame de la cámara)
                rvec, tvec = aruco.estimatePoseSingleMarkers(
                    markerCorners[i], self.markerLength, self.K, self.D)[:2]
                cv.drawFrameAxes(img, self.K, self.D, rvec, tvec, 0.01)

                # Armar el MarkerPose
                marker_pose = MarkerPose()
                marker_pose.marker_id = int(marker_id)
                # Posición en el frame de la cámara
                position = Point()
                position.x = float(tvec[0][0][0])  # X camera
                position.y = float(tvec[0][0][1])  # Y camera
                position.z = float(tvec[0][0][2])  # Z camera (frontal, distancia)
                # Orientación en quaternion (a partir de rvec)
                quat = self.rvec_to_quaternion(rvec[0][0])
                orientation = Quaternion()
                orientation.x = quat[0]
                orientation.y = quat[1]
                orientation.z = quat[2]
                orientation.w = quat[3]
                pose = Pose()
                pose.position = position
                pose.orientation = orientation
                marker_pose.pose = pose

                detection_msg.markers.append(marker_pose)

                # (Opcional) Visual
                cX = int((markerCorners[i][0][0][0] + markerCorners[i][0][2][0]) / 2.0)
                cY = int((markerCorners[i][0][0][1] + markerCorners[i][0][2][1]) / 2.0)
                distance = np.linalg.norm(tvec[0][0])
                cv.putText(img, f"{distance:.2f} m", (cX, cY - 15),
                           cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)

        # Publicar detección (aunque esté vacío, es importante para odometry)
        self.publisher_aruco.publish(detection_msg)

        # Visualización local
        cv.imshow("Aruco Detection", img)
        key = cv.waitKey(1)
        if key == 27:
            self.get_logger().info("ESC pressed, closing visualization window.")
            cv.destroyAllWindows()

    def rvec_to_quaternion(self, rvec):
        """Convierte un vector de rotación Rodrigues (rvec) a quaternion (x, y, z, w)"""
        R, _ = cv.Rodrigues(rvec)
        qw = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
