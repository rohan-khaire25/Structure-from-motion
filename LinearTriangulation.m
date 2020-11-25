function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 3) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 3) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points

function [R,t]=convertor(Rtemp,C)
[U1,~,V1]=svd(Rtemp);
if det(U1*(V1'))==1
R=U1*(V1');
t=Rtemp*C;
else
R=-U1*(V1');
t=-R*C;
end
end

[R1,t1]=convertor(R1,C1);
[R2,t2]=convertor(R2,C2);
P1=K*[R1,t1];
P2=K*[R2,t2];
[m,~]=size(x1);
for i=1:m
    a=[x1(i,1);x1(i,2);1];
    b=[x2(i,1);x2(i,2);1];
skew1=Vec2Skew(a);
skew2=Vec2Skew(b);

A=[skew1*P1;skew2*P2];
[~,~,V]=svd(A);
w=V(:,end)/V(end,end);
X(i,:)=[w(1),w(2),w(3)];
end
end
