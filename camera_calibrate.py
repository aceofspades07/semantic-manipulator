import cv2
import numpy as np
import pyrealsense2 as rs

DEFAULT_ROBOT_COORDS = np.array(
    [
        [414.36, 214.40],  # ID 0 (Top-Left)
        [417.66, -224.17],  # ID 1 (Top-Right)
        [108.74, -224.55],  # ID 2 (Bottom-Right)
        [106.34, 225.74],  # ID 3 (Bottom-Left)
    ],
    dtype="float32",
)


class Calibrator:
    def __init__(
        self,
        robot_coords: np.ndarray | list,
        *,
        marker_ids: tuple[int, int, int, int] = (0, 1, 2, 3),
        aruco_dict_name: int = cv2.aruco.DICT_4X4_50,
        save_path: str = "calibration_matrix.npy",
    ) -> None:
        coords = np.asarray(robot_coords, dtype="float32")
        if coords.shape != (4, 2):
            raise ValueError(
                f"robot_coords must have shape (4, 2), got {coords.shape}"
            )

        self.robot_coords = coords
        self.marker_ids = tuple(marker_ids)
        self.save_path = save_path

        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    def get_realsense_frame(self) -> np.ndarray:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)

        try:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if color_frame is None:
                raise RuntimeError("No color frame received from RealSense")
            image = np.asanyarray(color_frame.get_data())
        finally:
            pipeline.stop()

        return image

    def compute_homography(self, image: np.ndarray) -> np.ndarray | None:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, _rejected = self.detector.detectMarkers(gray)
        if ids is None or len(ids) < 4:
            print("Error: Less than 4 markers detected.")
            return None

        ids = ids.flatten()

        image_points: list[list[float]] = []
        try:
            for target_id in self.marker_ids:
                index = list(ids).index(target_id)
                marker_corners = corners[index][0]
                center_x = float(np.mean(marker_corners[:, 0]))
                center_y = float(np.mean(marker_corners[:, 1]))
                image_points.append([center_x, center_y])
                print(f"Found ID {target_id} at {center_x:.1f}, {center_y:.1f}")
        except ValueError:
            print(f"Error: Marker ID {target_id} missing from view.")
            return None

        image_points_arr = np.array(image_points, dtype="float32")
        return cv2.getPerspectiveTransform(image_points_arr, self.robot_coords)

    def run(self) -> bool:
        print("Capturing image...")
        image = self.get_realsense_frame()

        matrix = self.compute_homography(image)
        if matrix is None:
            return False

        np.save(self.save_path, matrix)
        print(f"Calibration successful! Matrix saved to '{self.save_path}'")
        return True

if __name__ == "__main__":
    Calibrator(DEFAULT_ROBOT_COORDS).run()