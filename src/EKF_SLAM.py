#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from RBE595_Final_Project.srv import GetDist
from geometry_msgs.msg import Pose


#NOTE:
#  This code is incomplete (though nearly done)
#  Required updates are marked with comments beginning with "TODO"

class EKF_SLAM():
    l = 1               # wheelbase length
    fwd_stddev = 1      # Standard deviation of encoder (forward distance) readings
    steer_stddev = 1    # Standard deviation of steering angle (avg angle of front wheel angles)
    sigma_x = 1         # Variance of feature x dimension
    sigma_y = 1         # Variance of feature y dimension
    sigma_z = 1         # Variance of feature z dimension
    sigma_s = 1         # Variance of feature s dimension

    l_c = 1             # horizontal distance from center of rear axle to camera frame
    h_c = 1             # vertical distance from rear axle to camera frame

    alpha = 6           # Threshold Mahalanobis distance to create a new feature in the map

    state = np.array([0,0,0,0]) # Current robot state
    cov = np.eye(4)             # Covariance matrix for robot state
    map_features = np.array([]) # list of features in the map
    N_features = 0              # Number of features in the map
    
    def __init__(self):
        rospy.init_node('EKF_SLAM')
        # Subscribe/ServiceProxy for features
        self.odom_pub = rospy.Subscriber('/state', Pose, self.updatePose)

        # Create the subscribers to take in the images from Coppelia, topic names are set in Coppelia 
        self.left_image_sub = rospy.Subscriber("/left_image", Image, self.updateLeftImage)
        self.right_image_sub = rospy.Subscriber("/right_image", Image, self.updateRightImage)

        self.state = None
        self.img_left = Image()
        self.img_right = Image()

        print("Waiting for server...")
        rospy.wait_for_service('image_dist_service')
        print("Connected to server")

        self.distFromImagesProxy = rospy.ServiceProxy('image_dist_service', self.getDistFromImages)
        print("DistanceFromImages proxy is live")

        rospy.spin()
        
        self.T_CR = self.transform_cam_robot()
        controls = np.array([1, 0.1])   # placeholder control inputs
        obs_features = np.array([[1.1,1.1,1.1,1.1],[4,5,6,7]])  # placeholder feature inputs
        self.updateEKF(controls, obs_features)

    # self.state is the robot state at t-1 (x,y,theta, phi)
    # self.cov is the 3x3 covariance matrix of the state at t-1
    # controls is [d,phi]
    #   d is the distance traveled in from t-1 to t
    #   phi is the steering angle (average angle of the front wheels)
    # obs_features is an Nx4 matrix containing N observed features
    #   each feature is [x,y,z,s]
    def updateEKF(self, controls, obs_features):
        # Run the EKF update once
        N = self.map_features.shape[0]
        F_x = np.hstack((np.eye(4), np.zeros((4, 4*N))))
        
        state_update = np.array([
            [ controls[0] * np.cos(self.state[2]) ],
            [ controls[0] * np.sin(self.state[2]) ],
            [ (controls[0]/self.l) * np.tan(self.state[3]) ],
            [ controls[1] ]
        ])
        
        # Estimate the new robot state
        state_est = self.state + np.dot(F_x.T, state_update).flatten()
        
        # jacobian of state_update w.r.t. (x,y,theta,phi)
        g = np.array([
            [ 0, 0, -controls[0] * np.sin(self.state[2]), 0 ],
            [ 0, 0,  controls[0] * np.cos(self.state[2]), 0 ],
            [ 0, 0, 0, (controls[0]/self.l) * np.tan(self.state[3]) ],
            [ 0, 0, 0, 0]
        ])
        G = np.eye(4+(4*N)) + F_x.T.dot(g).dot(F_x)   # I + F_x^T . g . F_x
        
        V = np.array([
            [ np.cos(self.state[2]), 0 ],
            [ np.sin(self.state[2]), 0 ],
            [ (1/self.l) * np.tan(self.state[3]), 0 ],
            [ 0, 1 ]
        ])
        M = np.diag((self.fwd_stddev, self.steer_stddev))
        R = V.dot(M).dot(V.T)   # R = V . M . V^T
        # Estimate new covariance matrix
        cov_est = G.dot(self.cov).dot(G.T) + F_x.T.dot(R).dot(F_x)   # Sigma = G . self.cov . G^T + F_x^T . R . F_x
        
        Q = np.diag((self.sigma_x, self.sigma_y, self.sigma_z, self.sigma_s))

        feature_lst = np.vstack((self.map_features, np.zeros(4)))
        num_features = obs_features.shape[0]
        
        for i in range(num_features):
            T_RW = self.transform_robot_world(state_est[0:3])
            T_CW = self.T_CR.dot(T_RW)
            new_feature = T_CW.dot(np.hstack((obs_features[i,:3], 1))).flatten()
            new_feature[3] = obs_features[i,3]
            feature_lst[-1] = new_feature

            z_hat_lst = np.zeros((N+1, 4))      # List of N+1 z_hat vectors
            H_lst = np.zeros((N+1, 4, 4+4*(N+1)))   # List of N+1 H matrices
            Psi_lst = np.zeros((N+1, 4, 4))     # List of N+1 Psi matrices
            pi_lst = np.zeros(N+1)              # List of N+1 pi values

            # TODO. Need to:
            #   Expand covariance matrix to include a hypothetical map feature

            for k in range(N+1):
                z_hat_lst[k] = self.get_z_hat(state_est, feature_lst[k])
                F_k = np.hstack((np.eye(8,4), np.zeros((8,4*k)), np.eye(8,4,-4), np.zeros((8,4*(N-k)))))
                H_lst[k] = np.dot(self.get_h(state_est, feature_lst[k]), F_k)
                Psi_lst[k] = H_lst[k].dot(cov_est).dot(H_lst[k].T)
                pi_lst[k] = (new_feature - z_hat_lst[k]).T.dot(np.linalg.inv(Psi_lst[k])).dot(new_feature - z_hat_lst[k])

            pi_lst = np.append(pi_lst, self.alpha)
            j = np.argmin(pi_lst)
            if j == N:  # A close enough feature was not in the map - add this feature to the map
                pass
                # TODO. Need to:
                #   Increment N
                #   Resize the state vector and covariance matrix
            
            # Kalman gain
            K = cov_est.dot(H_lst[j].T).dot(np.linalg.inv(Psi_lst[j]))
            # update state estimate
            state_est = state_est + K.dot(new_feature - z_hat_lst[j])
            # Update covariance
            cov_est = (np.eye(4*N+4) - K.dot(H_lst[j])).dot(cov_est)

        # Update overching variables with final estimates
        self.state = state_est
        self.cov = cov_est


    # returns the low-dimensional jacobian of z w.r.t. state variables and the current measurement variables
    def get_h(self, state_est, feature):
        sin_t = np.sin(state_est[2])
        cos_t = np.cos(state_est[2])
        return np.array([
            [ 1, 0, (self.l_c - feature[0])*sin_t - feature[1]*cos_t, 0, cos_t, -sin_t, 0, 0 ],
            [ 0, 1, (feature[0] - self.l_c)*cos_t - feature[1]*sin_t, 0, sin_t,  cos_t, 0, 0 ],
            [ 0, 0,                                                0, 0,     0,      0, 1, 0 ],
            [ 0, 0,                                                0, 0,     0,      0, 0, 1 ]
        ])

    # returns the position of the feature in the camera frame (sensor space) based on the current state estimate
    # equivalent to (T_CR*T_RW)^-1 * [homogeneous feature vector]
    def get_z_hat(self, state_est, feature):
        return np.array([
            state_est[0] - self.l_c*np.cos(state_est[2]) + feature[0]*np.cos(state_est[2]) - feature[1]*np.sin(state_est[2]),
            state_est[1] - self.l_c*np.sin(state_est[2]) + feature[0]*np.sin(state_est[2]) + feature[1]*np.cos(state_est[2]),
            feature[2] - self.h_c,
            feature[3]
        ])

    # Homogeneous transformation matrix from the camera frame to the robot frame (center of the rear axle)
    def transform_cam_robot(self):
        T_CR = np.array([
            [1, 0, 0, self.l_c],
            [0, 1, 0, 0],
            [0, 0, 1, self.h_c],
            [0, 0, 0, 1]
        ])
        return T_CR

    # Homogeneous transformation matrix from the robot to the world
    def transform_robot_world(self, state):
        T_RW = np.array([
            [  np.cos(state[2]), np.sin(state[2]), 0, -state[0]*np.cos(state[2]) - state[1]*np.sin(state[2]) ],
            [ -np.sin(state[2]), np.cos(state[2]), 0,  state[0]*np.sin(state[2]) - state[1]*np.cos(state[2]) ],
            [ 0, 0, 1, 0 ],
            [ 0, 0, 0, 1 ]
        ])
        return T_RW

    def updatePose(self, data):
        pos = data.Position
        rot = data.orientation
        # z axis in coppelia is y axis, w from quaternion is now steering angle in rad 
        self.state = [pos.x, pos.y, rot.y, rot.w]

    # Callback to update the left image, expecting the data from Coppelia to be a sensor_msgs/Image.msg
    def updateLeftImage(self, data):
        self.img_left = data

    # Callback to update the right image, expecting the data from Coppelia to be a sensor_msgs/Image.msg
    def updateRightImage(self, data):
        self.img_right = data
    
    # Ask Server for distance between images.
    def getDistFromImages(self, data):
        try:
            # Asking for distance from two images from the distance server.
            distance = self.distFromImagesProxy(self.img_left, self.img_right)
            return distance
            print("Measured distance:", distance)
        except rospy.ServiceException as e:
            print ("Service call failed: ")

if __name__ == "__main__":
    slam = EKF_SLAM()
