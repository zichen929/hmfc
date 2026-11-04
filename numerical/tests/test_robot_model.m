function tests = test_robot_model
%TEST_ROBOT_MODEL Independent geometry, energy and dynamics checks.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'shared')));
[testCase.TestData.robot,~,~,~] = hmfc_parameter_groups();
end

function testKinematicRoundTripAndVelocity(testCase)
r = testCase.TestData.robot;
for state = {[-0.7;1.3],[0.4;1.8],[1.1;0.9]}
    q = state{1}; dq = [0.13;-0.09];
    position = hmfc_forward_kinematics(q,r);
    verifyEqual(testCase,hmfc_inverse_kinematics(position,r),q,'AbsTol',1e-14);
    J = hmfc_jacobian(q,r);
    step = 1e-5;
    velocity = (hmfc_forward_kinematics(q+step*dq,r) ...
        -hmfc_forward_kinematics(q-step*dq,r))/(2*step);
    verifyEqual(testCase,J*dq,velocity,'AbsTol',1e-10);
    model = hmfc_robot_state(position,J*dq,r);
    verifyEqual(testCase,model.q,q,'AbsTol',1e-14);
    verifyEqual(testCase,model.dq,dq,'AbsTol',1e-14);
end
end

function testInertiaFromLinkKineticEnergy(testCase)
r = testCase.TestData.robot;
q = [-0.4;1.6]; dq = [0.3;-0.2];
m = hmfc_robot_model(q,dq,r);
% Independently form inertia from each center-of-mass velocity and rotation.
Jc1 = [-r.lc1*sin(q(1)),0;r.lc1*cos(q(1)),0];
Jc2 = [-r.l1*sin(q(1))-r.lc2*sin(sum(q)),-r.lc2*sin(sum(q)); ...
        r.l1*cos(q(1))+r.lc2*cos(sum(q)), r.lc2*cos(sum(q))];
expectedM = r.m1*(Jc1'*Jc1)+r.m2*(Jc2'*Jc2) ...
    +r.I1*[1,0;0,0]+r.I2*ones(2);
verifyEqual(testCase,m.M,expectedM,'AbsTol',1e-14);
verifyGreaterThan(testCase,min(eig(m.M)),0);
% dM - 2C must be skew symmetric for energy-consistent dynamics.
S = m.dM-2*m.C;
verifyEqual(testCase,S+S',zeros(2),'AbsTol',1e-14);
end

function testGravityFromPotentialEnergy(testCase)
r = testCase.TestData.robot;
q = [0.3;1.5];
m = hmfc_robot_model(q,[0;0],r);
potential = @(angles) r.m1*r.g*r.lc1*sin(angles(1)) ...
    +r.m2*r.g*(r.l1*sin(angles(1))+r.lc2*sin(sum(angles)));
step = 1e-5;
gradient = zeros(2,1);
for k = 1:2
    delta = zeros(2,1); delta(k) = step;
    gradient(k) = (potential(q+delta)-potential(q-delta))/(2*step);
end
verifyEqual(testCase,m.G,gradient,'AbsTol',1e-8);
end

function testAnalyticModelDerivatives(testCase)
r = testCase.TestData.robot;
for state = {[-0.7;1.3],[0.4;1.8],[1.1;0.9]}
    q = state{1}; dq = [0.17;-0.11];
    m = hmfc_robot_model(q,dq,r);
    step = 1e-5;
    plus = hmfc_robot_model(q+step*dq,dq,r);
    minus = hmfc_robot_model(q-step*dq,dq,r);
    for pair = {{'J','dJ'},{'M','dM'},{'Lambda','dLambda'}}
        names = pair{1};
        difference = (plus.(names{1})-minus.(names{1}))/(2*step);
        verifyEqual(testCase,m.(names{2}),difference,'AbsTol',1e-8);
    end
    verifyEqual(testCase,m.dJdq,m.dJ*dq,'AbsTol',1e-15);
end
end

function testJointAndCartesianDynamicsAgree(testCase)
r = testCase.TestData.robot;
q = [0.3;1.5]; dq = [0.2;-0.1]; ddq = [-0.4;0.6];
m = hmfc_robot_model(q,dq,r);
jointForce = m.J'\(m.M*ddq+m.C*dq+m.G);
acceleration = m.J*ddq+m.dJdq;
verifyEqual(testCase,m.Lambda*acceleration+m.eta,jointForce,'AbsTol',1e-12);
end

function testWorkspaceCheckIsIndependentOfCertificateRegion(testCase)
r = testCase.TestData.robot;
verifyError(testCase,@() hmfc_inverse_kinematics([2;0],r),'HMFC:Workspace');
verifyError(testCase,@() hmfc_inverse_kinematics([0;0],r),'HMFC:Workspace');
% A valid physical state outside q2 in [70,115] degrees must remain usable.
q = [0.2;0.8];
m = hmfc_robot_state(hmfc_forward_kinematics(q,r),[0;0],r);
verifyEqual(testCase,m.q,q,'AbsTol',1e-14);
verifyTrue(testCase,all(isfinite(m.Lambda),'all'));
end
