function model = hmfc_robot_state(position,velocity,robot)
%HMFC_ROBOT_STATE Reconstruct the robot model from Cartesian measurements.
%   The positive-elbow branch matches hmfc_inverse_kinematics. Contact and
%   certificate-region checks remain the responsibility of each simulation.
q = hmfc_inverse_kinematics(position,robot);
J = hmfc_jacobian(q,robot);
dq = J\velocity;
model = hmfc_robot_model(q,dq,robot);
end
