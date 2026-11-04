function [tau,p,pd,uf] = fic1_simscape_controller(t,q,dq,f,c,mode,kf)
%#codegen
%FIC1_SIMSCAPE_CONTROLLER Cartesian FIC1; q,dq,f come from Multibody sensors.
% This function is embedded in the generated model's MATLAB Function block.
% c, mode (0 regulation, 1 tracking), kf are model-workspace parameters.
s1=sin(q(1)); c1=cos(q(1));
s12=sin(q(1)+q(2)); c12=cos(q(1)+q(2));
p=[c.l1*c1+c.l2*c12;c.l1*s1+c.l2*s12];
J=[-c.l1*s1-c.l2*s12,-c.l2*s12;c.l1*c1+c.l2*c12,c.l2*c12];
v=J*dq;
if mode==0
    pd=c.p0+c.T*c.targetX;
    dpd=zeros(2,1); ddpd=zeros(2,1);
else
    pd=c.p0+c.T*(0.08+0.045*sin(0.8*t)+0.02*cos(1.3*t));
    dpd=c.T*(0.045*0.8*cos(0.8*t)-0.02*1.3*sin(1.3*t));
    ddpd=c.T*(-0.045*0.8^2*sin(0.8*t)-0.02*1.3^2*cos(1.3*t));
end
A=c.I1+c.m1*c.lc1^2+c.m2*c.l1^2;
B=c.m2*c.l1*c.lc2; D=c.I2+c.m2*c.lc2^2;
M=[A+D+2*B*cos(q(2)),D+B*cos(q(2));D+B*cos(q(2)),D];
b=B*sin(q(2));
C=[-b*dq(2),-b*(dq(1)+dq(2));b*dq(1),0];
G=[(c.m1*c.lc1+c.m2*c.l1)*c.g*c1+c.m2*c.lc2*c.g*c12;c.m2*c.lc2*c.g*c12];
dJdq=-[c.l1*c1*dq(1)^2+c.l2*c12*(dq(1)+dq(2))^2; ...
    c.l1*s1*dq(1)^2+c.l2*s12*(dq(1)+dq(2))^2];
up=ddpd+c.kv*(dpd-v)+c.kp*(pd-p);
uf=c.fd+kf*(c.fd-f);
% tau = J'*(eta + Lambda*up - n*uf), evaluated without explicit inverse.
tau=C*dq+G+M*(J\(up-dJdq))-J'*c.n*uf;
end
