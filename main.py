from camera.camera_stream import CameraStream
from detection.lane_detection import LaneDetection
from detection.parameter_adjuster import ParameterAdjuster
from decision.lane_follower import LaneFollower
from motor.motor_controller import MotorController
import numpy as np
import cv2
import json

def main(dual_camera=False, show_visuals=False, adjust_parameters=False, use_vanishing_point=True):

    config_file = "dual_camera_config.json" if dual_camera else "single_camera_config.json"
    
    if adjust_parameters and show_visuals:
        # Use camera 0 to adjust parameters
        camera1 = CameraStream(camera_id=0)
        lane_detector = LaneDetection(video_source=camera1, dual_camera=dual_camera)
        adjuster = ParameterAdjuster(lane_detector)
        adjuster.adjust_all_parameters()
        # Save adjusted parameters to a file
        with open(config_file, "w") as f:
            json.dump(lane_detector.get_parameters(), f)
        print(f"Configuration sauvegardée dans {config_file}")
        camera1.stop()
        return

    # Load parameters from configuration file
    try:
        with open(config_file, "r") as f:
            parameters = json.load(f)
    except FileNotFoundError:
        print(f"Configuration file {config_file} not found. Using default parameters.")
        parameters = {}

    camera1 = CameraStream(camera_id=0)
    camera2 = CameraStream(camera_id=1) if dual_camera else None

    if dual_camera:
        lane_detector1 = LaneDetection(video_source=camera1, dual_camera=True, camera_side='left', 
                                      parameters=parameters)
        lane_detector2 = LaneDetection(video_source=camera2, dual_camera=True, camera_side='right', 
                                      parameters=parameters)
    else:
        lane_detector1 = LaneDetection(video_source=camera1, dual_camera=False, 
                                      parameters=parameters)

    motor_controller = MotorController()
    lane_follower = LaneFollower(dual_camera=dual_camera)

    try:
        while True:
            frame1 = camera1.get_frame()
            frame2 = camera2.get_frame() if dual_camera else None

            if dual_camera:
                lines1 = lane_detector1.get_lines(frame1)
                lines2 = lane_detector2.get_lines(frame2)
                if show_visuals:
                    lane_detector1.display(frame1, lines1, window_name="Lane Detection - Camera 1", resize=None)
                    lane_detector2.display(frame2, lines2, window_name="Lane Detection - Camera 2", resize=None)
                # Motor control logic for dual-camera mode could be added here
            else:
                lines1 = lane_detector1.get_lines(frame1)
                if show_visuals:
                    lane_detector1.display(frame1, lines1, window_name="Lane Detection - Single Camera")

                # Commande proportionnelle du moteur
                offset = lane_follower.get_offset(lines1, frame1.shape)
                motor_controller.set_steering(offset)

            if show_visuals and cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera1.stop()
        if dual_camera:
            camera2.stop()
        if show_visuals:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    """
    Runtime configuration parameters:

    - dual_camera: False to use a single camera, True for two cameras
    - show_visuals: Enables real-time display of images (requires a connected screen)
    - adjust_parameters: Allows manual adjustment of detection parameters (requires show_visuals=True)

    SSH mode (headless, no screen):
        main(dual_camera=False, show_visuals=False, adjust_parameters=False)

    Screen mode (visualization and parameter tuning):
        main(dual_camera=False, show_visuals=True, adjust_parameters=True)
    """
    main(dual_camera=False, show_visuals=False, adjust_parameters=False)
