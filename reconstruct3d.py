from numpy.linalg import lstsq
import cv2
import numpy as np
def create_objective_function_and_gradient(cameras, ball_positions, ball_radii):

    def proj(point3d, camera):
        # Returns a 2d point
        return camera.proj(point3d)

    def squared_distance(p1, p2):
        return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

    def objective_function(B):
        # Measures how poorly the projection of 3d point B
        # onto the camera planes matches with the ball_positions
        total_err = 0

        # This loop iterates once per camera
        for camera, ball_position, ball_radius in zip(cameras, ball_positions, ball_radii):
            ball2d = proj(ball_position, camera)
            total_err += squared_distance(ball2d, ball_position)

        return total_err

    def gradient(B):
        e = 0.001
        Bx = (B[0]+e, B[1],   B[2])
        By = (B[0],   B[1]+e, B[2])
        Bz = (B[0],   B[1],   B[2]+e)
        obj = objective_function(B)
        dx = (objective_function(Bx) - obj) / e
        dy = (objective_function(By) - obj) / e
        dz = (objective_function(Bz) - obj) / e
        return [dx, dy, dz]

    return objective_function, gradient

def find_point3d(cameras, ball_positions, ball_radii):
    objective_function, gradient = create_objective_function_and_gradient(cameras, ball_positions, ball_radii)
    point3d = np.array([0,0,0])  # initial guess
    learning_rate = .3

    eps = 0.1
    while err > eps:
        point3d = point3d - learning_rate * gradient(point3d)
        err = objective_function(point3d)

    return point3d

# distance is in centimeters
class Camera():
    def __init__(self, position, point_correspondences):
        self.position = position
        self.point_correspondences = point_correspondences

        object_points = np.array([point3d for point3d, point2d in point_correspondences])
        image_points  = np.array([point2d for point3d, point2d in point_correspondences])
        object_points = object_points.astype('float32')
        image_points = image_points.astype('float32')
        image_size = (1167,655)

        retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera([object_points], [image_points], image_size, flags=cv2.CALIB_USE_INTRINSIC_GUESS)
        self.cameraMatrix = cameraMatrix
        self.distCoeffs = distCoeffs
        self.rvecs = np.array(rvecs)
        self.tvecs = np.array(tvecs)

    def proj(self, point3d):
        imagePoints, jacobian = cv2.projectPoints(np.array([point3d]), self.rvecs, self.tvecs, self.cameraMatrix, self.distCoeffs)
        return imagePoints[0]


# 3d: 0,0,0 is center of table
# 2d: 0,0 is top left corner of image
point_correspondences = [
    [[-70,0,0], [854, 352]],
    [[-70,15,0], [737, 352]],
    [[-70,0,10], [858, 273]],
    [[-35,0,0], [1068, 356]]
]

camera_position = [70, 60, 4]
camera = Camera(camera_position, point_correspondences)
# create_objective_function_and_gradient([camera], ball_positions, ball_radii)
print camera.proj([-35.0,0,0.0])

def reconstruct(cameras, ball_positions_in_all_frames, ball_radii_in_all_frames):
    point3ds = []
    for ball_positions, ball_radii in zip(ball_positions_in_all_frames, ball_radii_in_all_frames):
        point3d.append(find_point3d(cameras, ball_positions, ball_radii))

    return point3ds

