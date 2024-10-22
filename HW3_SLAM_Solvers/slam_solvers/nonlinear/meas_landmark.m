% MEAS_LANDMARK
% 16-833 Spring 2019 - *Stub* Provided
% Simple function to predict a nonlinear landmark measurement from a pose
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     h     - odometry measurement prediction 
%
function h = meas_landmark(rx, ry, lx, ly)
[theta, d] = cart2pol(lx - rx, ly - ry);
h = [theta; d];
end