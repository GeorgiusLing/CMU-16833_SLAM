% CREATE_AB_LINEAR
% 16-833 Spring 2019 - *Stub* Provided
% Computes the A and b matrices for the 2D linear SLAM problem
%
% Arguments: 
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
%                 obs(:,3) - x-value of landmark measurement
%                 obs(:,4) - y-value of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     A       - MxN matrix
%     b       - Mx1 vector
%
function [As, b] = create_Ab_linear(odom, obs, sigma_o, sigma_l)


% Useful Constants
n_poses = size(odom, 1) + 1; % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;
l_dim = 2;
o_dim = size(odom, 2);
m_dim = size(obs(1, 3:end), 2);

% A matrix is MxN, b is Mx1
N = p_dim * n_poses + l_dim * n_landmarks;
M = o_dim * (n_odom + 1) + m_dim * n_obs;     % +1 for prior on the first pose

% Initialize matrices
A = zeros(M, N);
b = zeros(M, 1);

% Add odometry and landmark measurements to A, b - including prior on first
% pose

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A(1 : o_dim, 1 : p_dim) = sigma_o ^ (-1/2) * eye(p_dim);%eye(p_dim);

for i = 2 : n_poses
    A(o_dim * (i - 1) + 1 : o_dim * i, p_dim * (i - 1) + 1 : p_dim * i) = sigma_o ^ (-1/2) * eye(p_dim);
    A(o_dim * (i - 1) + 1 : o_dim * i, p_dim * (i - 2) + 1 : p_dim * (i - 1)) = sigma_o ^ (-1/2) * (-eye(p_dim));
    b(o_dim * (i - 1) + 1 : o_dim * i) = sigma_o ^ (-1/2) * transpose(odom(i - 1, :));
end

for i = 1 : n_obs
    mat_indices = [];
    pose_index = obs(i, 1);
    landmark_index = obs(i, 2);
    
    for j = 0 : p_dim - 1        
        mat_indices = [mat_indices, (((pose_index - 1 ) * p_dim + j) * M + o_dim * (n_odom + 1) + (i - 1) * m_dim + 1 : ((pose_index - 1 ) * p_dim + j) * M + o_dim * (n_odom + 1) + i * m_dim)];
    end
    
    A(mat_indices) = sigma_l ^ (-1/2) * (-eye(p_dim));
    
    mat_indices = [];
    for j = 0 : l_dim - 1        
        mat_indices = [mat_indices, (((landmark_index - 1 ) * l_dim + j) * M + o_dim * (n_odom + 1) + (i - 1) * m_dim + 1 : ((landmark_index - 1 ) * l_dim + j) * M + o_dim * (n_odom + 1) + i * m_dim)];
    end
    mat_indices = mat_indices + ones(size(mat_indices)) * n_poses * p_dim * M;

    A(mat_indices) = sigma_l ^ (-1/2) * eye(l_dim);
    
    b(o_dim * (n_odom + 1) + (i - 1) * m_dim + 1 : o_dim * (n_odom + 1) + i * m_dim) = sigma_l ^ (-1/2) * transpose(obs(i, 3 : 4));
end
%% Make A a sparse matrix 
As = sparse(A);
end

    