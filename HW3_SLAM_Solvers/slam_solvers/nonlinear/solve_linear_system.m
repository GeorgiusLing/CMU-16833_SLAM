% SOLVE_LINEAR_SYSTEM
% 16-833 Spring 2019 - *Stub* Provided
% Solve the linear system with your method of choice
%
% Arguments: 
%     A     - A matrix from linear system that you generate yourself
%     b     - b vector from linear system that you generate yourself
%
% Returns:
%     x     - solution to the linear system, computed using your method of
%             choice
%
function x = solve_linear_system(A, b)

% R = chol(A' * A);
% y = forward_sub(R', A' * b);
% x = back_sub(R, y);
[R,p,S] = chol(A' * A, 'matrix');

y = forward_sub(R', S' * A' * b);
x = back_sub(R, y);
x = S * x;

end