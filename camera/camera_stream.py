from picamera2 import Picamera2
import cv2

class CameraStream:
    def __init__(self, camera_id=0):
        self.camera = Picamera2(camera_id)
        # Add main and raw configuration
        config = self.camera.create_preview_configuration(
            main={"size": (640, 480)},
            raw={"size": (3280, 2464)}
        )
        self.camera.configure(config)
        self.camera.start()

    def get_frame(self):
        return self.camera.capture_array()

    def stop(self):
        self.camera.stop()