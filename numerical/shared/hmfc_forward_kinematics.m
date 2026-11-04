function position = hmfc_forward_kinematics(q,robot)
%HMFC_FORWARD_KINEMATICS Cartesian endpoint of the planar 2R robot (m).
position = [robot.l1*cos(q(1))+robot.l2*cos(sum(q)); ...
            robot.l1*sin(q(1))+robot.l2*sin(sum(q))];
end
