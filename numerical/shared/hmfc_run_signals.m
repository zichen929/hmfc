function signals = hmfc_run_signals(run)
%HMFC_RUN_SIGNALS Common CSV columns, followed by optional estimator signals.
signals = table(run.t,run.state(:,1),run.state(:,2),run.error,run.force, ...
    run.forceCommand,run.angle,run.tau(:,1),run.tau(:,2),'VariableNames', ...
    {'time_s','x_t_m','dx_t_mps','motion_error_m','force_N', ...
    'force_command_N','normal_angle_rad','tau1_Nm','tau2_Nm'});
if isfield(run,'hatTheta') && ~isempty(run.hatTheta)
    signals.hat_theta_rad = run.hatTheta;
    signals.beta_rad = run.beta;
    signals.bar_theta_rad = run.barTheta;
    signals.angle_error_rad = run.angleError;
end
end
