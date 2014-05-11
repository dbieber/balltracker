from numpy.linalg import lstsq
import cv2
import numpy as np

def create_objective_function_and_gradient(cameras, ball_positions, ball_radii):

    def proj(point3d, camera):
        # Returns a 2d point
        return camera.proj(point3d)

    def squared_distance(p1, p2):
        p1 = np.reshape(p1, 2)
        p2 = np.reshape(p2, 2)
        return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

    def objective_function(B):
        # Measures how poorly the projection of 3d point B
        # onto the camera planes matches with the ball_positions
        total_err = 0

        # This loop iterates once per camera
        for camera, ball_position, ball_radius in zip(cameras, ball_positions, ball_radii):
            ball2d = proj(B, camera)
            total_err += squared_distance(ball2d, ball_position)

        return total_err

    def gradient(B):
        e = 1
        Bx = (B[0]+e, B[1],   B[2])
        By = (B[0],   B[1]+e, B[2])
        Bz = (B[0],   B[1],   B[2]+e)
        obj = objective_function(B)
        dx = (objective_function(Bx) - obj) / e
        dy = (objective_function(By) - obj) / e
        dz = (objective_function(Bz) - obj) / e
        print "d", np.array([dx, dy, dz])
        return np.array([dx, dy, dz])

    return objective_function, gradient

def find_point3d(cameras, ball_positions, ball_radii):
    if ball_positions is None or None in ball_positions or ball_radii is None or None in ball_radii or cameras is None or None in cameras:
        return
    objective_function, gradient = create_objective_function_and_gradient(cameras, ball_positions, ball_radii)
    point3d = np.array([0,0,0])  # initial guess
    learning_rate = 2

    eps = 1
    prev_err = 100
    err = prev_err + 2*eps
    old_point3d = np.array([1,1,1])
    while np.linalg.norm(point3d - old_point3d) > eps:
        old_point3d = point3d
       # prev_err = err
        point3d = point3d - learning_rate * gradient(point3d)
        err = objective_function(point3d)
        print err, 

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
        self.rvecs = np.array(rvecs).astype('float32')
        self.tvecs = np.array(tvecs).astype('float32')

    def proj(self, point3d):
        imagePoints, jacobian = cv2.projectPoints(np.array([point3d]).astype('float32'), self.rvecs, self.tvecs, self.cameraMatrix, self.distCoeffs)
        return imagePoints[0]

def reconstruct(cameras, ball_positions_in_all_frames, ball_radii_in_all_frames):
    point3ds = []
    for i, (ball_positions, ball_radii) in enumerate(zip(ball_positions_in_all_frames, ball_radii_in_all_frames)):
        #print i, ball_positions, ball_radii
        point3ds.append(find_point3d(cameras, ball_positions, ball_radii))

    return point3ds
