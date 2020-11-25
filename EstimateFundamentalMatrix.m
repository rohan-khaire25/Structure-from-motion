function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

A = [];
for i=1:8
B = [x1(i,1)*x2(i,1) x1(i,1)*x2(i,2) x1(i,1) x1(i,2)*x2(i,1) x1(i,2)*x2(i,2) x1(i,2) x2(i,1) x2(i,2) 1];
A = [A;B];
end

[U, S, V] = svd(A);
x = V(:,9);

F = reshape(x,3,3);
[U1, S1, V1] = svd(F);
S1(3,3) = 0;
F = U1 * S1 * V1';
F = F/norm(F);