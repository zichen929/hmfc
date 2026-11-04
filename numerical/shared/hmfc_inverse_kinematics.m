function q = hmfc_inverse_kinematics(position,robot)
%HMFC_INVERSE_KINEMATICS Nonsingular positive-elbow branch, 0 < q2 < pi.
%   Only physical workspace validity is checked here. Certificate-region
%   limits belong to the simulation that uses this model.
cos2 = (position'*position-robot.l1^2-robot.l2^2)/(2*robot.l1*robot.l2);
assert(abs(cos2)<1,'HMFC:Workspace','Unreachable or singular endpoint position.');
q2 = acos(cos2);
q1 = atan2(position(2),position(1)) ...
    -atan2(robot.l2*sin(q2),robot.l1+robot.l2*cos(q2));
q = [q1;q2];
end
