function model = hmfc_robot_model(q,dq,robot)
%HMFC_ROBOT_MODEL Shared joint and Cartesian dynamics of the planar 2R arm.
%   model = hmfc_robot_model(q,dq,robot) returns q, dq, J, dJ, dJdq,
%   M, dM, C, G, Lambda, dLambda and eta at a nonsingular configuration.
%   Joint dynamics: M*ddq + C*dq + G = tau + J'*contactForce.
%   Cartesian dynamics: Lambda*acceleration + eta = F + contactForce.
%   This function has no contact geometry, controller or operating-region data.
J = hmfc_jacobian(q,robot);
dJ = [-robot.l1*cos(q(1))*dq(1)-robot.l2*cos(sum(q))*sum(dq), ...
        -robot.l2*cos(sum(q))*sum(dq); ...
        -robot.l1*sin(q(1))*dq(1)-robot.l2*sin(sum(q))*sum(dq), ...
        -robot.l2*sin(sum(q))*sum(dq)];
A = robot.I1+robot.m1*robot.lc1^2+robot.m2*robot.l1^2;
B = robot.m2*robot.l1*robot.lc2;
D = robot.I2+robot.m2*robot.lc2^2;
M = [A+D+2*B*cos(q(2)),D+B*cos(q(2));D+B*cos(q(2)),D];
dM = -B*sin(q(2))*dq(2)*[2,1;1,0];
b = B*sin(q(2));
C = [-b*dq(2),-b*sum(dq);b*dq(1),0];
G = [(robot.m1*robot.lc1+robot.m2*robot.l1)*robot.g*cos(q(1))+ ...
    robot.m2*robot.lc2*robot.g*cos(sum(q)); ...
    robot.m2*robot.lc2*robot.g*cos(sum(q))];
K = J\eye(2);
dK = -K*dJ*K;
Lambda = K'*M*K;
dLambda = dK'*M*K+K'*dM*K+K'*M*dK;
dJdq = dJ*dq;
eta = J'\(C*dq+G)-Lambda*dJdq;
model = struct('q',q,'dq',dq,'J',J,'dJ',dJ,'dJdq',dJdq, ...
    'M',M,'dM',dM,'C',C,'G',G,'Lambda',Lambda, ...
    'dLambda',dLambda,'eta',eta);
end
