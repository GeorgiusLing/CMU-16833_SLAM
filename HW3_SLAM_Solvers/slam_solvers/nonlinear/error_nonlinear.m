% ERROR_NONLINEAR
% 16-833 Spring 2019 - *Stub* Provided
% Computes the total error of all measurements (odometry and landmark)
% given the current state estimate
%
% Arguments: 
%     x       - Current estimate of the state vector
%     odom    - Matrix that contains the odometry measurements
%               between consecutive poses. Each row corresponds to
%               a measurement. 
%                 odom(:,1) - x-value of odometry measurement
%                 odom(:,2) - y-value of odometry measurement
%     obs     - Matrix that contains the landmark measurements and
%               relevant information. Each row corresponds to a
%               measurement.
%                 obs(:,1) - idx of pose at which measurement was 
%                   made
%                 obs(:,2) - idx of landmark being observed
%                 obs(:,3) - bearing theta of landmark measurement
%                 obs(:,4) - range d of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     err     - total error of all measurements
%
function err = error_nonlinear(x, odom, obs, sigma_odom, sigma_landmark)
%% Extract useful constants which you may wish to use
n_poses = size(odom, 1) + 1;                % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;                                  % pose dimension
l_dim = 2;                                  % landmark dimension
o_dim = size(odom, 2);                      % odometry dimension
m_dim = size(obs(1, 3:end), 2);    % landmark measurement dimension

% A matrix is MxN, b is Nx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;         % +1 for prior on the first pose

%% Initialize error
err = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for i = 2 : n_poses
%     err_i = transpose(odom(i - 1, : )) - meas_odom(x(o_dim * (i - 2) + 1), x(o_dim * (i - 1)), x(o_dim * (i - 1) + 1), x(o_dim * i));%(x(o_dim * (i - 1) + 1 : o_dim * i) - x(o_dim * (i - 2) + 1 : o_dim * (i - 1);
%     err = err + norm(err_i) ^ 2;
% end
% 
% for i = 1 : n_obs
%     pose_index = obs(i, 1);
%     landmark_index = obs(i, 2);
%     err_i = transpose(obs(i, 3 : 4)) - meas_landmark(x((pose_index - 1) * p_dim + 1), x(pose_index * p_dim), x((landmark_index - 1) * p_dim + 1 + n_poses * p_dim), x(landmark_index * p_dim + n_poses * p_dim));
%     err_i(1) = wrapToPi(err_i(1));
%     err = err + norm(err_i) ^ 2;
% end
[~, b] = create_Ab_nonlinear(x, odom, obs, sigma_odom, sigma_landmark);
err = norm(b) ^ 2;

end