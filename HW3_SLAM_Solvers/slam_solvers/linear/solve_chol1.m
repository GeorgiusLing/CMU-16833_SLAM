% SOLVE_CHOL1
% 16-833 Spring 2019 - *Stub* Provided
% Solves linear system using first Cholesky method
%
% Arguments: 
%     A     - A matrix from linear system that you generate yourself
%     b     - b vector from linear system that you generate yourself
%
% Returns:
%     x     - solution to the linear system, computed using the specified
%             version of the Cholesky decomposition
%     R     - R factor from the Cholesky decomposition
%
function [x, R] = solve_chol1(A, b)

R = chol(A' * A);
y = forward_sub(R', A' * b);
x = back_sub(R, y);

end