function c = hmfc_simscape_parameters()
%HMFC_SIMSCAPE_PARAMETERS SI parameters matching fic_simulation.m exactly.
c.l1=0.6; c.l2=0.5; c.m1=2; c.m2=1.5;
c.lc1=c.l1/2; c.lc2=c.l2/2;
c.I1=c.m1*c.l1^2/12; c.I2=c.m2*c.l2^2/12;
c.g=9.81; c.kp=25; c.kv=10; c.fd=10;
c.p0=[0.6;0.4]; c.n=[0;1]; c.T=[1;0];
c.targetX=0.08; c.duration=6; c.outputStep=0.002;
c.q2Min=70*pi/180; c.q2Max=115*pi/180;
c.xMin=-0.04; c.xMax=0.16;
q2=acos((c.p0'*c.p0-c.l1^2-c.l2^2)/(2*c.l1*c.l2));
c.q0=[atan2(c.p0(2),c.p0(1))-atan2(c.l2*sin(q2),c.l1+c.l2*cos(q2));q2];
c.dq0=zeros(2,1);
end
