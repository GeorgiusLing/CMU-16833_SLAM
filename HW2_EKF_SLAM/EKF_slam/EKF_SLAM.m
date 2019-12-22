%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Assignment #2                         %
%  EKF-SLAM                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc;

%==== TEST: Setup uncertainity parameters (try different values!) ===
sig_x = 0.25;
sig_y = 0.1;
sig_alpha = 0.1;
sig_beta = 0.01;
sig_r = 0.08;

%==== Generate sigma^2 from sigma ===
sig_x2 = sig_x^2;
sig_y2 = sig_y^2;
sig_alpha2 = sig_alpha^2;
sig_beta2 = sig_beta^2;
sig_r2 = sig_r^2;

%==== Open data file ====
fid = fopen('../data/data.txt');

%==== Read first measurement data ====
tline = fgets(fid);
arr = str2num(tline);
measure = arr';
t = 1;
 
%==== Setup control and measurement covariances ===
control_cov = diag([sig_x2, sig_y2, sig_alpha2]);
measure_cov = diag([sig_beta2, sig_r2]);

%==== Setup initial pose vector and pose uncertainty ====
pose = [0 ; 0 ; 0];
pose_cov = diag([0.02^2, 0.02^2, 0.1^2]);

%==== TODO: Setup initial landmark vector landmark[] and covariance matrix landmark_cov[] ====
%==== (Hint: use initial pose with uncertainty and first measurement) ====

% Write your code here...
[num_measure, ~] = size(measure);
landmark = [];
landmark_cov = [];
for k = 1 : num_measure / 2
    beta = measure(2 * k - 1);
    r = measure(2 * k);
    [dx, dy] = pol2cart(beta + pose(3), r);
    r_vec = [dx ; dy];
    landmark_k = pose(1 : 2) + r_vec;
    landmark = [landmark ; landmark_k];
    
    landmark_pose = [1, 0, -r * sin(pose(3) + beta); 0, 1, r * cos(pose(3) + beta)];
    landmark_measure = [-r * sin(pose(3) + beta), cos(pose(3) + beta) ; r * cos(pose(3) + beta), sin(pose(3) + beta)];
    landmark_cov = blkdiag(landmark_cov, landmark_pose * pose_cov * landmark_pose' + landmark_measure * measure_cov * landmark_measure');
end

%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];

%==== Setup covariance matrix P with pose and landmark covariances ====
P = [pose_cov zeros(3, 2*k) ; zeros(2*k, 3) landmark_cov];

%==== Plot initial state and conariance ====
last_x = x;
drawTrajAndMap(x, last_x, P, 0);

%==== Read control data ====
tline = fgets(fid);
while ischar(tline)
    arr = str2num(tline);
    d = arr(1);
    alpha = arr(2);
    
    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ====
    
    % Write your code here...
    
    input = [d ; 0 ; alpha];
    x_pre = x + [eul2rotm([0 0 x(3)] , 'XYZ') * input ; zeros(2 * k , 1)];
    
    [tx, ty] = pol2cart(-x(3) , input(1));
    state_mat = makehgtform('translate', [ty tx 0]);
    state_mat(3,:) = [];
    state_mat(:,3) = [];
    B = [cos(x(3)), -sin(x(3)), 0; sin(x(3)), cos(x(3)), 0; 0, 0, 1];
    input_mat = eul2rotm([0 0 x(3)] , 'XYZ');
    state_jacobi = blkdiag(state_mat, eye(2 * k));
    %P_pre = P;
    %P_pre(1 : 3, 1 : 3) = state_mat * P(1 : 3, 1 : 3) * state_mat' + input_mat * control_cov * input_mat';
    P_pre = state_jacobi * P * state_jacobi'; 
    P_pre(1 : 3, 1 : 3) = P_pre(1 : 3, 1 : 3) + input_mat * control_cov * input_mat';  
    
    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
    
    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';
    
    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====
    
    % Write your code here...
    measure_jacobi = [];
    measure_pre = [];
    Q = [];
    for i = 1 : k
        landmark_i = x_pre(3 + 2 * i - 1 : 3 + 2 * i);
        r_vec = landmark_i - x_pre(1 : 2);
        
        measure_jacobi = [measure_jacobi ; zeros(2, 3 + 2 * k)];
        measure_jacobi(end - 1, 1 : 2) = fliplr(r_vec')/norm(r_vec)^2;
        measure_jacobi(end - 1, 2) = -measure_jacobi(end - 1, 2);
        measure_jacobi(end, 1 : 2) = -r_vec'/norm(r_vec);
        measure_jacobi(end - 1 : end, 3) = [-1 ; 0];
        measure_jacobi(end - 1 : end, 3 + 2 * i - 1 : 3 + 2 * i) = -measure_jacobi(end - 1 : end, 1 : 2);
        
        [ang, r] = cart2pol(r_vec(1),r_vec(2));
        measure_pre = [measure_pre ; wrapToPi(ang - x_pre(3)) ; r];
        Q = blkdiag(Q, measure_cov);
    end
    kalman_gain = P_pre * measure_jacobi' * inv(measure_jacobi * P_pre * measure_jacobi' + Q);

    x = x_pre + kalman_gain * (measure - measure_pre);
    P = (eye(size(P)) - kalman_gain * measure_jacobi) * P_pre;
    
    %==== Plot ====   
    drawTrajAndMap(x, last_x, P, t);
    last_x = x;
    
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);
end

%==== EVAL: Plot ground truth landmarks ====

% Write your code here...
ground_truth = [3 6 3 12 7 8 7 14 12 6 11 12];
euclidean = [];
mahalanobis = [];
for i = 1 : k
    plot(ground_truth(2 * i - 1), ground_truth(2 * i), 'pk', 'MarkerSize',10);
    hold on
    error = [ground_truth(2 * i - 1) - x(3 + 2 * i - 1) ; ground_truth(2 * i) - x(3 + 2 * i)];
    euclidean = [euclidean norm(error)];
    mahalanobis = [mahalanobis sqrt(error' * inv(P(3 + 2 * i - 1 : 3 + 2 * i , 3 + 2 * i - 1 : 3 + 2 * i)) * error)];
end
hold off
%==== Close data file ====
fclose(fid);
