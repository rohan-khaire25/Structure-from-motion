function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly

N = size(X, 1);
A = zeros(N*3, 12);

for i=1:N
    j = (i-1)*3 + 1;
    X1=[X(i,:),1];
    x1=[x(i,:),1]';
    f1=K\x1;
skew=Vec2Skew(f1);
f2=[X1,zeros(1,4),zeros(1,4);zeros(1,4),X1,zeros(1,4);zeros(1,4),zeros(1,4),X1];
    A(j:j+2,:)=skew*f2;
end
[~,~,V] = svd(A);

P = reshape(V(:,end),4,3)';

pre_R = P(:,1:3);

[RU, RS, RV] = svd(pre_R);
detUV = round(det(RU*RV'));

R = (detUV*RU)*RV';
t = detUV*(P(:,4))/RS(1,1);

C = -R'*t;
end





