import cv2
import numpy as np
import pyrealsense2 as rs

def get_realsense_pipeline():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    return pipeline


def main():
    # Setup ArUco
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    print("Opening RealSense D435 live stream. Press 'q' to quit.")
    pipeline = get_realsense_pipeline()
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue
            image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten()
                for i, marker_corners in enumerate(corners):
                    pts = marker_corners[0].astype(int)
                    # Draw bounding box
                    cv2.polylines(image, [pts], isClosed=True, color=(0,255,0), thickness=2)
                    # Compute top-left for text
                    top_left = pts[0]
                    marker_id = int(ids_flat[i])
                    # Draw ID above the box
                    cv2.putText(
                        image,
                        f"ID {marker_id}",
                        (top_left[0], top_left[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                # Optionally print IDs to console
                print("Detected ArUco marker IDs:", ids_flat.tolist())
            else:
                # Optionally print only once or clear line
                pass

            cv2.imshow("RealSense ArUco Detection", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
