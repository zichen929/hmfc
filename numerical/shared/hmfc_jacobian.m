function J = hmfc_jacobian(q,robot)
%HMFC_JACOBIAN Planar 2R endpoint Jacobian, velocity = J*dq.
J = [-robot.l1*sin(q(1))-robot.l2*sin(sum(q)),-robot.l2*sin(sum(q)); ...
      robot.l1*cos(q(1))+robot.l2*cos(sum(q)), robot.l2*cos(sum(q))];
end
