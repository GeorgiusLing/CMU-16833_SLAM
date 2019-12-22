% MEAS_LANDMARK_JACOBIAN
% 16-833 Spring 2019 - *Stub* Provided
% Compute the Jacobian of the measurement function
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     H     - Jacobian of the measurement fuction
%
function H = meas_landmark_jacobian(rx, ry, lx, ly)

syms x1 y1 x2 y2
[theta, d] = cart2pol(x2 - x1, y2 - y1);
H = jacobian([theta ; d], [x1, y1, x2, y2]);
H = subs(H, [x1, y1, x2, y2], [rx, ry, lx, ly]);

end