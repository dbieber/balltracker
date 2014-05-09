from numpy.linalg import lstsq
import cv2

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

# distance is in centimeters
class Camera():
    def __init__(self, position, point_correspondences):
        self.position = position
        self.point_correspondences = point_correspondences

        object_points = [point3d for point3d, point2d in point_correspondences]
        image_points  = [point2d for point3d, point2d in point_correspondences]
        image_size = [1167,655]

        retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, image_size)
        self.cameraMatrix = cameraMatrix
        self.distCoeffs = distCoeffs
        self.rvecs = rvecs
        self.tvecs = tvecs

    def proj(self, point3d):
        imagePoints, jacobian = cv2.projectPoints([point3d], self.rvec, self.tvec, self.cameraMatrix, self.distCoeffs)
        return imagePoints[0]


# 3d: 0,0,0 is center of table
# 2d: 0,0 is top left corner of image
point_correspondences = [
    ((-70,0,0), (854, 352)),
    ((-70,0,10), (858, 273)),
    ((-35,0,0), (1068, 356))
]

camera_position = [70, 60, 4]
camera = Camera(camera_position, point_correspondences)
# create_objective_function_and_gradient([camera], ball_positions, ball_radii)
print camera.proj([50,50,3.5])
