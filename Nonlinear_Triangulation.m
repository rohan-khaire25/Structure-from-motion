function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 


function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
end

function J = Jacobian_Triangulation(C, R, K, X)
    U = K*R*(X-C);
    f = K(1,1);
    px = K(1,3);
    py = K(2,3);
    dudx = [(f*R(1,1)+px*R(3,1)), (f*R(1,2)+px*R(3,2)), (f*R(1,3)+px*R(3,3))];
    dvdx = [(f*R(2,1)+py*R(3,1)), (f*R(2,2)+py*R(3,2)), (f*R(2,3)+py*R(3,3))];
    dwdx = [R(3,1) R(3,2) R(3,3)];
    J = [(U(3)*dudx - U(1)*dwdx)/(U(3)^2); (U(3)*dvdx - U(2)*dwdx)/(U(3)^2)];
end

n = size(x1,1);
X = zeros(n,3);
for i = 1:n
    f1 = K*R1*(X0(i,:)'-C1);
    f2 = K*R2*(X0(i,:)'-C2);
    f3 = K*R3*(X0(i,:)'-C3);
    fX = [f1(1)/f1(3), f1(2)/f1(3), f2(1)/f2(3), f2(2)/f2(3), f3(1)/f3(3), f3(2)/f3(3)]';
    J1 = Jacobian_Triangulation(C1, R1, K, X0(i,:)');
    J2 = Jacobian_Triangulation(C2, R2, K, X0(i,:)');
    J3 = Jacobian_Triangulation(C3, R3, K, X0(i,:)');
    J = [J1' J2' J3']';
    b = [x1(i,1) x1(i,2) x2(i,1) x2(i,2) x3(i,1) x3(i,2)]';
    deltaX = inv(J'*J)*J'*(b-fX);
    X(i,:) = X0(i,:) + deltaX';
end
end