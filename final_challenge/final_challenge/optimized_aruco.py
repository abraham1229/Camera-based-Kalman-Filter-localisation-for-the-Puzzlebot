    def calculate_scaled_calibration(self, original_width, original_height):
        """Calculate scaled camera calibration parameters for downscaled image"""
        # Calculate scaling factors for both dimensions
        self.scale_factor_x = self.target_width / original_width
        self.scale_factor_y = self.target_height / original_height
        
        # Scale the camera matrix K - this is crucial for accurate pose estimation
        # We need to scale focal lengths and principal point coordinates
        self.K_scaled = self.K_original.copy()
        self.K_scaled[0, 0] *= self.scale_factor_x  # fx (focal length in x)
        self.K_scaled[1, 1] *= self.scale_factor_y  # fy (focal length in y)  
        self.K_scaled[0, 2] *= self.scale_factor_x  # cx (principal point x)
        self.K_scaled[1, 2] *= self.scale_factor_y  # cy (principal point y)
        
        # Distortion coefficients typically remain the same for scaling
        # (though in practice, slight recalibration might be beneficial)
        self.D_scaled = self.D_original.copy()
        
        self.get_logger().info(f"Scaled calibration calculated:")
        self.get_logger().info(f"Scale factors: x={self.scale_factor_x:.3f}, y={self.scale_factor_y:.3f}")
        self.get_logger().info(f"Original resolution: {original_width}x{original_height}")
        self.get_logger().info(f"Target resolution: {self.target_width}x{self.target_height}")

    def image_callback(self, msg):
        # Increment frame counter for skipping logic
        self.frame_counter += 1
        
        # Skip processing if this isn't the frame we want to process
        # This immediately reduces computational load by half
        if self.frame_counter % self.process_every_n_frames != 0:
            return
            
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Get original image dimensions and calculate calibration if first time
        original_height, original_width = img.shape[:2]
        if self.K_scaled is None:
            self.calculate_scaled_calibration(original_width, original_height)
        
        # Downscale the image using OpenCV's efficient resize function
        # INTER_AREA is best for downscaling as it considers pixel neighborhoods
        img_downscaled = cv.resize(img, (self.target_width, self.target_height), 
                                   interpolation=cv.INTER_AREA)
        
        # Calculate cropping boundaries on the downscaled image (remove top and bottom 20%)
        crop_top = int(self.target_height * 0.2)      # Remove top 20%
        crop_bottom = int(self.target_height * 0.8)   # Keep until 80% from top
        
        # Apply cropping to the already downscaled image
        # This compounds our performance gains: smaller image + smaller processing area
        cropped_img = img_downscaled[crop_top:crop_bottom, :]
        
        # Convert to grayscale for marker detection
        gray = cv.cvtColor(cropped_img, cv.COLOR_BGR2GRAY)
    
        # Detect markers using the scaled camera calibration parameters
        markerCorners, markerIds, rejected = self.detector.detectMarkers(gray)

        # Prepare ArucoDetection message
        detection_msg = ArucoDetection()
        detection_msg.header = msg.header
        detection_msg.markers = []

        if markerIds is not None and len(markerIds) > 0:
            # Draw markers on the processed image for visualization
            aruco.drawDetectedMarkers(cropped_img, markerCorners, markerIds)
            
            for i, marker_id in enumerate(markerIds.flatten()):
                # CRITICAL: Use the scaled camera matrix for pose estimation
                # This ensures accurate 3D pose calculation despite image downscaling
                rvec, tvec = aruco.estimatePoseSingleMarkers(
                    markerCorners[i], self.markerLength, self.K_scaled, self.D_scaled)[:2]
                cv.drawFrameAxes(cropped_img, self.K_scaled, self.D_scaled, rvec, tvec, 0.01)

                # Create MarkerPose message - poses remain accurate in world coordinates
                # because we properly scaled our camera calibration
                marker_pose = MarkerPose()
                marker_pose.marker_id = int(marker_id)
                
                # Position in camera frame (accurate despite downscaling)
                position = Point()
                position.x = float(tvec[0][0][0])  # X camera
                position.y = float(tvec[0][0][1])  # Y camera  
                position.z = float(tvec[0][0][2])  # Z camera (distance)
                
                # Convert rotation vector to quaternion
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

                # Visual feedback on the downscaled, cropped image
                cX = int((markerCorners[i][0][0][0] + markerCorners[i][0][2][0]) / 2.0)
                cY = int((markerCorners[i][0][0][1] + markerCorners[i][0][2][1]) / 2.0)
                distance = np.linalg.norm(tvec[0][0])
                
                # Log every few detections to avoid overwhelming the console
                if self.frame_counter % 10 == 0:  # Log every 10th processed frame
                    self.get_logger().info(
                        f"Frame {self.frame_counter}: Id={int(marker_id)}, distance={distance:.3f}m"
                    )
                cv.putText(cropped_img, f"{distance:.2f} m", (cX, cY - 15),
                           cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)

        # Always publish detection results (even if empty) for consistent data flow
        self.publisher_aruco.publish(detection_msg)

        # Display the processed image - shows the actual working resolution
        cv.imshow("Aruco Detection (Downscaled + Cropped)", cropped_img)
        key = cv.waitKey(1)
        if key == 27:
            self.get_logger().info("ESC pressed, closing visualization window.")
            cv.destroyAllWindows()