#!/usr/bin/env python
import rospy
import numpy as np

class EKF_SLAM():
    l = 1      # wheelbase length
    fwd_stddev = 1      # Standard deviation of encoder (forward distance) readings
    steer_stddev = 1    # Standard deviation of steering angle (avg angle of front wheel angles)
    sigma_x = 1         # Variance of feature x dimension
    sigma_y = 1         # Variance of feature y dimension
    sigma_z = 1         # Variance of feature z dimension
    sigma_s = 1         # Variance of feature s dimension

    l_c = 1             # horizontal distance from center of rear axle to camera frame
    h_c = 1             # vertical distance from rear axle to camera frame
    
    def __init__(self):
        #rospy.init_node('EKF_SLAM')
        # Subscribe/ServiceProxy for features
        # Subscribe/ServiceProxy for odometry
        #rospy.spin()
        N_old = 3
        old_state = np.zeros((4+(4*N_old)))
        old_cov = np.eye(4+(4*N_old))
        controls = np.array([1, 0.1])
        features = np.array([])
        self.updateEKF(old_state, old_cov, controls, features, N_old)

    # state_old is the robot state at t-1 (x,y,theta, phi)
    # cov_old is the 3x3 covariance matrix of the state at t-1
    # controls is [d,phi]
    #   d is the distance traveled in from t-1 to t
    #   phi is the steering angle (average angle of the front wheels)
    # features is an Nx4 matrix containing N observed features
    #   each feature is [x,y,z,s]
    def updateEKF(self, state_old, cov_old, controls, features, N_old):
        # Run the EKF update once
        N = N_old
        F_x = np.hstack((np.eye(4), np.zeros((4, 4*N))))
        
        state_update = np.array([
            [ controls[0] * np.cos(state_old[2]) ],
            [ controls[0] * np.sin(state_old[2]) ],
            [ (controls[0]/self.l) * np.tan(state_old[3]) ],
            [ controls[1] ]
        ])
        
        # Estimate the new robot state
        state_est = state_old + np.dot(F_x.T, state_update).flatten()
        
        # jacobian of state_update w.r.t. (x,y,theta)
        g = np.array([
            [ 0, 0, -controls[0] * np.sin(state_old[2]), 0 ],
            [ 0, 0,  controls[0] * np.cos(state_old[2]), 0 ],
            [ 0, 0, 0, (controls[0]/self.l) * np.tan(state_old[3]) ],
            [ 0, 0, 0, 0]
        ])
        G = np.eye(4+(4*N)) + F_x.T.dot(g).dot(F_x)   # I + F_x^T . g . F_x
        
        V = np.array([
            [ np.cos(state_old[2]), 0 ],
            [ np.sin(state_old[2]), 0 ],
            [ (1/self.l) * np.tan(state_old[3]), 0 ],
            [ 0, 1 ]
        ])
        M = np.diag((self.fwd_stddev, self.steer_stddev))
        R = V.dot(M).dot(V.T)   # R = V . M . V^T
        # Estimate new covariance matrix
        cov_est = G.dot(cov_old).dot(G.T) + F_x.T.dot(R).dot(F_x)   # Sigma = G . cov_old . G^T + F_x^T . R . F_x
        
        Q = np.diag((self.sigma_x, self.sigma_y, self.sigma_z, self.sigma_s))

        num_features,_ = features.shape
        for i in range(num_features):
            T_CR = self.transform_cam_robot()
            T_RW = self.transform_robot_world(state_est[0:3])
            new_feature = T_CR.dot(T_RW).dot(np.hstack((features[i], 1))).flatten()
            new_feature[3] = features[i,3]

            for k in range(N + 1):
                pass

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

if __name__ == "__main__":
    slam = EKF_SLAM()
