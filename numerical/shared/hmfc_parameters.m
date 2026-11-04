function c = hmfc_parameters()
%HMFC_PARAMETERS Common SI parameters for numerical and Simscape simulations.
%   Robot, contact scene, FIC gains, initial state and sampling defaults are
%   defined here. NAC/IAC-specific estimator settings stay in numerical/.
c.l1 = 0.6; c.l2 = 0.5;
c.m1 = 2.0; c.m2 = 1.5;
c.lc1 = c.l1/2; c.lc2 = c.l2/2;
c.I1 = c.m1*c.l1^2/12; c.I2 = c.m2*c.l2^2/12;
c.g = 9.81;
c.kp = 25; c.kv = 10; c.fd = 10;
c.surfaceAngle = 0;
c.n = [-sin(c.surfaceAngle); cos(c.surfaceAngle)];
c.T = [0,1;-1,0]*c.n;
c.p0 = [0.6;0.4];
c.initialVelocity = 0;
c.q2Min = 70*pi/180; c.q2Max = 115*pi/180;
c.minSinQ2 = min(sin([c.q2Min,c.q2Max]));
c.xMin = -0.04; c.xMax = 0.16;
c.q0 = hmfc_inverse_kinematics(c.p0,c);
assert(c.q0(2)>c.q2Min && c.q0(2)<c.q2Max,'HMFC:Region', ...
    'Initial elbow angle must be inside the operating region.');
J = hmfc_jacobian(c.q0,c);
c.dq0 = J\(c.T*c.initialVelocity);
c.targetX = 0.08;
c.setpoint = c.p0+c.T*c.targetX;
c.duration = 6; c.outputStep = 0.002;
end
