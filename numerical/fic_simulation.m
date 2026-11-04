function results = fic_simulation(options)
%FIC_SIMULATION Run selected FIC controllers using the common numerical API.
%   fic_simulation(struct('controllerIds',1,'makePlots',false,'saveResults',false))
%   IDs are distinct integers 1:4; default selection is 1:4.
%   Every controller independently integrates [x_t; dx_t] with a joint-space
%   constrained plant. FIC 1/2 use Cartesian feedback; FIC 3/4 use intrinsic
%   feedback. Their reference conversions and equivalence checks are retained.

if nargin == 0, options = struct(); end
common = hmfc_parameters();
defaults = struct('controllerIds',1:4,'makePlots',true,'saveResults',true, ...
    'visible','on','outputDir','','duration',common.duration, ...
    'initialVelocity',common.initialVelocity);
options = hmfc_simulation_options(options,defaults);
ids = options.controllerIds;
assert(isnumeric(ids) && isreal(ids) && isvector(ids) && ~isempty(ids) ...
    && all(ismember(ids,1:4)) && numel(unique(ids))==numel(ids), ...
    'FIC:Controller','controllerIds must be distinct IDs from 1 to 4.');
options.controllerIds = double(ids(:)');
if isempty(options.outputDir)
    options.outputDir = fullfile(hmfc_numerical_root(),'results', ...
        ['fic',sprintf('%d',options.controllerIds)]);
end
if isscalar(options.controllerIds)
    results = run_controller(options);
else
    results = run_comparison(options);
end
end

function results = run_controller(options)
common = hmfc_parameters();
[robot,plant,controller,task] = hmfc_parameter_groups();
controller.id = options.controllerIds;
% FIC has full information; only its controller receives the true geometry.
controller.n = plant.n; controller.T = plant.T; controller.p0 = plant.p0;
odeOptions = odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.01);
time = linspace(0,options.duration,ceil(options.duration/common.outputStep)+1)';
modes = {'regulation','tracking'};
results = struct('name',sprintf('FIC %d',controller.id),'source','TCST26E3.md / FIC', ...
    'options',options,'robot',robot,'plant',plant,'controller',controller, ...
    'task',task,'odeOptions',odeOptions,'runs',[]);
for im = 1:numel(modes)
    for kf = [0,2]
        rhs = @(t,state) closed_loop(t,state,robot,plant,controller,task,modes{im},kf);
        [t,state] = ode45(rhs,time,[0;options.initialVelocity],odeOptions);
        run = postprocess(t,state,robot,plant,controller,task,modes{im},kf);
        run.checks = verify_run(run,plant);
        results.runs = [results.runs;run];
    end
end
for im = 1:numel(modes)
    runA = results.runs(2*im-1); runB = results.runs(2*im);
    gap = max(vecnorm(runA.state-runB.state,2,2));
    assert(gap<1e-7,'FIC:GainInvariance','Ideal trajectories unexpectedly depend on kf.');
    results.gainComparison(im) = struct('mode',modes{im},'maxStateDifference',gap);
end
results.summary = hmfc_run_summary(results.runs);
results.checks = struct('gainInvariancePassed',true,'passed',true);
results.allChecksPassed = all(arrayfun(@(run) run.checks.passed,results.runs));
if options.saveResults
    if ~exist(options.outputDir,'dir'), mkdir(options.outputDir); end
    writetable(results.summary,fullfile(options.outputDir,'summary.csv'));
    for k = 1:numel(results.runs)
        run = results.runs(k);
        signals = hmfc_run_signals(run);
        writetable(signals,fullfile(options.outputDir,[run.name,'.csv']));
    end
    save(fullfile(options.outputDir,sprintf('fic%d_results.mat',controller.id)),'results');
end
if options.makePlots, make_plots(results); end
fprintf('FIC %d: all numerical checks passed.\n',controller.id);
end

function dstate = closed_loop(t,state,robot,plant,controller,task,mode,kf)
evaluation = evaluate(t,state,robot,plant,controller,task,mode,kf);
dstate = [state(2);plant.T'*evaluation.acceleration];
end

function evaluation = evaluate(t,state,robot,plant,controller,task,mode,kf)
[position,velocity,model] = measured_state(state(1),state(2),robot,plant);
q = model.q; dq = model.dq; J = model.J;
ref = reference_command(t,task,mode,controller);
% First obtain F0, then close the instantaneous force-feedback loop exactly.
cmd = controller_command(position,velocity,ref,model,controller,kf,0);
[acceleration,force,ddq] = contact_plant(cmd,model,plant.n);
cmd = controller_command(position,velocity,ref,model,controller,kf,force);
evaluation = cmd;
evaluation = rmfield(evaluation,{'F0','forceFeedback','F','tau'});
evaluation.q = q; evaluation.dq = dq; evaluation.ddq = ddq;
evaluation.position = position; evaluation.velocity = velocity; evaluation.acceleration = acceleration;
evaluation.force = force; evaluation.tau = cmd.tau;
evaluation.angle = atan2(-controller.n(1),controller.n(2));
evaluation.gap = plant.n'*(position-plant.p0);
evaluation.normalVelocity = plant.n'*velocity;
evaluation.normalAcceleration = plant.n'*acceleration;
evaluation.errorDynamicsResidual = cmd.referenceAcceleration-acceleration+controller.kv*cmd.velocityError+controller.kp*cmd.positionError;
evaluation.jointResidual = model.M*ddq+model.C*dq+model.G-cmd.tau-J'*plant.n*force;
evaluation.cartesianResidual = model.Lambda*acceleration+model.eta-cmd.F-plant.n*force;
Li_n = model.Lambda\plant.n;
evaluation.forceReconstructionError = force+(Li_n'*(cmd.F-model.eta))/(plant.n'*Li_n);
evaluation.commandResidual = acceleration-cmd.up;
evaluation.V = 0.5*(cmd.velocityError'*cmd.velocityError+controller.kp*(cmd.positionError'*cmd.positionError));
evaluation.dV = cmd.velocityError'*(cmd.referenceAcceleration-acceleration)+controller.kp*cmd.positionError'*cmd.velocityError;
evaluation.lyapunovResidual = evaluation.dV+controller.kv*(cmd.velocityError'*cmd.velocityError);
evaluation.minLambda = min(eig(model.Lambda)); evaluation.maxLambda = max(eig(model.Lambda));
evaluation.condJ = cond(J);
if controller.id>=3
    evaluation.intrinsicErrorResidual = cmd.ddxtd-plant.T'*acceleration+controller.kv*cmd.det+controller.kp*cmd.et;
    evaluation.errorMappingResidual = norm(cmd.positionError-plant.T*cmd.et)+norm(cmd.velocityError-plant.T*cmd.det);
end
end

function [acceleration,force,ddq] = contact_plant(cmd,model,n)
% Original joint dynamics and acceleration constraint, including force feedback.
Jn = n'*model.J;
solution = [model.M,-model.J'*(n+cmd.forceFeedback);Jn,0]\ ...
    [model.J'*cmd.F0-model.C*model.dq-model.G;-n'*model.dJdq];
ddq = solution(1:2); force = solution(3);
assert(force>0,'FIC:ContactLoss','The calculated reaction is not positive.');
acceleration = model.J*ddq+model.dJdq;
end

function cmd = controller_command(position,velocity,ref,model,controller,kf,force)
% force is the instantaneous scalar contact-force measurement.
switch controller.id
    case 1
        % FIC 1 directly receives three Cartesian reference vectors.
        pd = ref.pd; dpd = ref.dpd; ddpd = ref.ddpd;
    case {2,3}
        % FIC 2 maps these for feedback; FIC 3 uses the mapping for diagnostics.
        xtd = ref.xtd; dxtd = ref.dxtd; ddxtd = ref.ddxtd;
        pd = controller.p0 + controller.T*xtd;
        dpd = controller.T*dxtd;
        ddpd = controller.T*ddxtd;
    case 4
        % FIC 4 receives Cartesian references, then converts to intrinsic ones.
        pInputd = ref.pd; dpInputd = ref.dpd; ddpInputd = ref.ddpd;
        xtd = controller.T'*(pInputd-controller.p0);
        dxtd = controller.T'*dpInputd;
        ddxtd = controller.T'*ddpInputd;
        % Effective on-surface reference used only for diagnostics/verification.
        % If the input has a normal offset, this is its contact-line projection.
        pd = controller.p0 + controller.T*xtd;
        dpd = controller.T*dxtd;
        ddpd = controller.T*ddxtd;
end
ep = pd - position;
ev = dpd - velocity;

if controller.id >= 3
    % The scalar controller has no access to pd, dpd, ddpd, ep, or ev.
    [ut,xt,dxt,et,det] = intrinsic_control(position,velocity,xtd,dxtd,ddxtd,controller);
    up = controller.T*ut;
else
    % FIC 1 / FIC 2 Cartesian feedback.
    up = ddpd + controller.kv*ev + controller.kp*ep;
end
cmd = struct('reference',pd,'referenceVelocity',dpd,'referenceAcceleration',ddpd, ...
    'positionError',ep,'velocityError',ev,'up',up, ...
    'F0',model.eta+model.Lambda*up-controller.n*(1+kf)*controller.fd);
cmd.forceCommand = controller.fd+kf*(controller.fd-force);
cmd.forceFeedback = controller.n*kf;
cmd.F = cmd.F0+controller.n*kf*force;
cmd.tau = model.J'*cmd.F;
if controller.id>=3
    cmd.xt = xt; cmd.dxt = dxt; cmd.xtd = xtd; cmd.dxtd = dxtd;
    cmd.ddxtd = ddxtd; cmd.et = et; cmd.det = det; cmd.ut = ut;
end
if controller.id==4
    cmd.pInputd = pInputd; cmd.dpInputd = dpInputd; cmd.ddpInputd = ddpInputd;
    cmd.referenceNormalOffset = controller.n'*(pInputd-controller.p0);
    cmd.referenceProjectionResidual = norm(controller.T'*(pInputd-pd)) ...
        +norm(controller.T'*(dpInputd-dpd))+norm(controller.T'*(ddpInputd-ddpd));
end
end

function [position,velocity,model] = measured_state(x,tangentialVelocity,robot,plant)
assert(x>plant.xMin && x<plant.xMax,'FIC:Region','Tangential domain exceeded.');
position = plant.p0+plant.T*x;
velocity = plant.T*tangentialVelocity;
model = hmfc_robot_state(position,velocity,robot);
assert(abs(sin(model.q(2)))>=plant.minSinQ2,'FIC:Singularity', ...
    'Trajectory left the chosen nonsingularity region.');
assert(model.q(2)>plant.q2Min && model.q(2)<plant.q2Max,'FIC:Region', ...
    'Elbow angle left the common operating region.');
end

function [ut,xt,dxt,et,det] = intrinsic_control(pm,dpm,xtd,dxtd,ddxtd,controller)
% FIC 3 / FIC 4: intrinsic errors use measured Cartesian position/velocity.
% This function deliberately receives no Cartesian desired position or error.
xt = controller.T'*(pm-controller.p0);
dxt = controller.T'*dpm;
et = xtd-xt;
det = dxtd-dxt;
ut = ddxtd + controller.kv*det + controller.kp*et;
end

function ref = reference_command(t,task,mode,controller)
% Generate the controller's native inputs before evaluating its control law.
if ismember(controller.id,[1,4])
    [pd,dpd,ddpd] = cartesian_reference(t,task,mode);
    ref = struct('pd',pd,'dpd',dpd,'ddpd',ddpd);
else
    [xtd,dxtd,ddxtd] = intrinsic_reference(t,task,mode);
    ref = struct('xtd',xtd,'dxtd',dxtd,'ddxtd',ddxtd);
end
end

function [pd, dpd, ddpd] = cartesian_reference(t, task, mode)
% FIC 1 and FIC 4 receive these Cartesian vectors directly. No scalar target
% or scalar tracking error is evaluated inside the controller.
switch mode
    case 'regulation'
        pd = task.setpoint;
        dpd = zeros(2,1); ddpd = zeros(2,1);
    case 'tracking'
        pd = task.origin + task.axis*(0.08 + 0.045*sin(0.8*t) + 0.02*cos(1.3*t));
        dpd = task.axis*(0.045*0.8*cos(0.8*t) - 0.02*1.3*sin(1.3*t));
        ddpd = task.axis*(-0.045*0.8^2*sin(0.8*t) - 0.02*1.3^2*cos(1.3*t));
    otherwise
        error('FIC:Reference', 'Unknown reference mode: %s', mode);
end
end

function [xtd, dxtd, ddxtd] = intrinsic_reference(t, task, mode)
% Input to FIC 2 and FIC 3: scalar desired position and its derivatives.
% Kept separate from the FIC 1 Cartesian generator to exercise conversion.
switch mode
    case 'regulation'
        xtd = task.targetX; dxtd = 0; ddxtd = 0;
    case 'tracking'
        xtd = 0.08 + 0.045*sin(0.8*t) + 0.02*cos(1.3*t);
        dxtd = 0.045*0.8*cos(0.8*t) - 0.02*1.3*sin(1.3*t);
        ddxtd = -0.045*0.8^2*sin(0.8*t) - 0.02*1.3^2*cos(1.3*t);
    otherwise
        error('FIC:Reference', 'Unknown reference mode: %s', mode);
end
end

function run = postprocess(t, state, robot, plant, controller, task, mode, kf)
run.controller = sprintf('FIC %d',controller.id);
run.name = sprintf('%s_kf_%g',mode,kf);
run.initialState = state(1,:)';
run.mode = mode; run.kf = kf; run.t = t; run.state = state;
for i = 1:numel(t)
    evaluation = evaluate(t(i),state(i,:)',robot,plant,controller,task,mode,kf);
    if i == 1
        names = fieldnames(evaluation);
        for j = 1:numel(names)
            run.(names{j}) = zeros(numel(t), numel(evaluation.(names{j})));
        end
    end
    for j = 1:numel(names)
        run.(names{j})(i,:) = evaluation.(names{j})(:)';
    end
end
% Independent analytical benchmark, e'' + kv*e' + kp*e = 0.
A = [0,1;-controller.kp,-controller.kv];
e0 = [run.positionError(1,:)*plant.T; run.velocityError(1,:)*plant.T];
exact = zeros(numel(t),2);
for i = 1:numel(t)
    exact(i,:) = (expm(A*(t(i)-t(1)))*e0)';
end
run.analyticPositionError = exact(:,1)*plant.T';
run.analyticVelocityError = exact(:,2)*plant.T';
metrics.finalPositionError = norm(run.positionError(end,:));
metrics.finalVelocityError = norm(run.velocityError(end,:));
metrics.tailRmsPositionError = sqrt(mean(sum(run.positionError(t>=t(end)-2,:).^2,2)));
metrics.maxForceError = max(abs(controller.fd-run.force));
metrics.maxAnalyticPositionError = max(vecnorm(run.positionError-run.analyticPositionError,2,2));
metrics.maxAnalyticVelocityError = max(vecnorm(run.velocityError-run.analyticVelocityError,2,2));
metrics.maxGap = max(abs(run.gap));
metrics.maxNormalVelocity = max(abs(run.normalVelocity));
metrics.maxNormalAcceleration = max(abs(run.normalAcceleration));
metrics.maxErrorDynamicsResidual = max(vecnorm(run.errorDynamicsResidual,2,2));
metrics.maxJointResidual = max(vecnorm(run.jointResidual,inf,2));
metrics.maxCartesianResidual = max(vecnorm(run.cartesianResidual,inf,2));
metrics.maxForceReconstructionError = max(abs(run.forceReconstructionError));
metrics.maxCommandResidual = max(vecnorm(run.commandResidual,2,2));
metrics.maxLyapunovResidual = max(abs(run.lyapunovResidual));
metrics.maxPositive_dV = max([0;run.dV]);
metrics.minSinQ2 = min(abs(sin(run.q(:,2))));
metrics.maxJacobianCondition = max(run.condJ);
metrics.minCartesianInertia = min(run.minLambda);
metrics.maxCartesianInertia = max(run.maxLambda);
metrics.minForce = min(run.force);
metrics.peakTorque = max(abs(run.tau),[],'all');
run.error = run.positionError*plant.T;
run.x = state(:,1);
run = hmfc_run_region(run,plant);
run.metrics = hmfc_run_metrics(run,plant,controller);
for field = fieldnames(metrics)', run.metrics.(field{1}) = metrics.(field{1}); end
end

function checks = verify_run(run, plant)
metrics = run.metrics;
checks = metrics;
assert(all(isfinite(run.state(:))) && all(isfinite(run.tau(:))), ...
    'FIC:Nonfinite', 'Nonfinite simulation output.');
assert(metrics.finalPositionError < 1e-7 && metrics.finalVelocityError < 1e-7, ...
    'FIC:Convergence', 'Motion error did not converge.');
assert(metrics.maxAnalyticPositionError < 1e-7 && metrics.maxAnalyticVelocityError < 1e-7, ...
    'FIC:Analytic', 'ODE trajectory disagrees with analytical error solution.');
residuals = [metrics.maxForceError, metrics.maxGap, ...
    metrics.maxNormalVelocity, metrics.maxNormalAcceleration, metrics.maxErrorDynamicsResidual, ...
    metrics.maxJointResidual, metrics.maxCartesianResidual, metrics.maxForceReconstructionError, ...
    metrics.maxCommandResidual, metrics.maxLyapunovResidual, metrics.maxPositive_dV];
assert(all(residuals < 1e-8), 'FIC:Residual', ...
    'A force, contact, dynamics, or Lyapunov residual exceeds tolerance.');
assert(metrics.minSinQ2 >= plant.minSinQ2 && metrics.minCartesianInertia > 0, ...
    'FIC:Region', 'Nonsingularity or positive inertia condition violated.');
assert(metrics.minForce>0,'FIC:ContactLoss','Unilateral contact was not maintained.');
if isfield(run,'et')
    assert(max(abs(run.intrinsicErrorResidual)) < 1e-8 ...
        && max(run.errorMappingResidual) < 1e-8, ...
        'FIC:IntrinsicDynamics','Intrinsic dynamics or error mapping failed.');
end
if isfield(run,'referenceProjectionResidual')
    assert(max(run.referenceProjectionResidual) < 1e-8, ...
        'FIC:ReferenceProjection','FIC 4 reference projection failed.');
end
checks.operatingRegionSatisfied = run.operatingRegionSatisfied;
assert(run.operatingRegionSatisfied);
checks.passed = true;
end

function make_plots(results)
for index = 1:2:numel(results.runs)
    plot_case_pair(results.runs(index:index+1),results.plant,results.controller,results.options);
end
plot_diagnostics(results.runs,results.plant,results.controller,results.options);
end

function plot_case_pair(pair, plant, controller, options)
runA = pair(1); runB = pair(2); t = runA.t;
fig = figure('Name', [runA.controller,' - ',runA.mode], 'Color','w','Visible',options.visible, ...
    'Position',[80,80,1250,900]);
hmfc_latex_figure(fig);
if isprop(fig,'Theme'), set(fig,'Theme','light'); end
tl = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
title(tl,sprintf('%s | %s | 2R arm | $k_p=%g$, $k_v=%g$, $f_d=%g$ N', ...
    runA.controller,runA.mode,controller.kp,controller.kv,controller.fd),'Interpreter','latex');
nexttile;
linePoints = plant.p0 + plant.T*[-0.07,0.20];
plot(linePoints(1,:),linePoints(2,:),'k:','LineWidth',1); hold on;
plot(runA.reference(:,1),runA.reference(:,2),'k--','LineWidth',2);
plot(runA.position(:,1),runA.position(:,2),'b-','LineWidth',1.3);
plot(runA.position(1,1),runA.position(1,2),'go','MarkerFaceColor','g');
axis equal; grid on; xlabel('$p_1$ (m)'); ylabel('$p_2$ (m)');
title('Cartesian motion on the horizontal contact line');
legend('Contact line','Reference','Actual','Initial contact','Location','best');
nexttile;
plot(t,(runA.reference-plant.p0')*plant.T,'k--',t,runA.state(:,1),'b-',t,runB.state(:,1),'r:','LineWidth',1.4);
grid on; xlabel('$t$ (s)'); ylabel('$x_t$ (m)'); title('Tangential position (diagnostic)');
legend('Reference','$k_f = 0$','$k_f = 2$','Location','best');
nexttile;
semilogy(t,max(vecnorm(runA.positionError,2,2),1e-16),'b-', ...
    t,max(vecnorm(runB.positionError,2,2),1e-16),'r:', ...
    t,max(vecnorm(runA.analyticPositionError,2,2),1e-16),'k--','LineWidth',1.3);
grid on; xlabel('$t$ (s)'); ylabel('$\|e_p\|$ (m)'); title('Exponential error convergence');
legend('$k_f = 0$','$k_f = 2$','Analytical solution','Location','best');
nexttile;
plot(t,runA.force,'b-',t,runB.force,'r:','LineWidth',1.5); hold on;
yline(controller.fd,'k--'); ylim([controller.fd-1,controller.fd+1]);
grid on; xlabel('$t$ (s)'); ylabel('$f$ (N)'); title('Algebraic normal force: no force transient');
legend('$k_f = 0$','$k_f = 2$','$f_d$','Location','best');
nexttile;
semilogy(t,max(runA.V,1e-30),'b-',t,max(runB.V,1e-30),'r:','LineWidth',1.3);
grid on; xlabel('$t$ (s)'); ylabel('$V$'); title('$V = \frac{1}{2}(\|\dot{e}_p\|^2 + k_p\|e_p\|^2)$');
legend('$k_f = 0$','$k_f = 2$','Location','best');
nexttile;
plot(t,runA.tau(:,1),'b-',t,runA.tau(:,2),'r-', ...
    t,runB.tau(:,1),'k:',t,runB.tau(:,2),'m:','LineWidth',1.2);
grid on; xlabel('$t$ (s)'); ylabel('Joint torque ($\mathrm{N\,m}$)'); title('Applied actuator torques');
legend('$\tau_1,\ k_f=0$','$\tau_2,\ k_f=0$', ...
    '$\tau_1,\ k_f=2$','$\tau_2,\ k_f=2$','Location','best');
hmfc_latex_figure(fig);
if options.saveResults
    exportgraphics(fig,fullfile(options.outputDir,[runA.mode,'.png']),'Resolution',160);
    savefig(fig,fullfile(options.outputDir,[runA.mode,'.fig']));
end
end

function plot_diagnostics(runs, plant, controller, options)
fig = figure('Name',[runs(1).controller,' - residuals'],'Color','w','Visible',options.visible,'Position',[100,100,1250,900]);
hmfc_latex_figure(fig);
if isprop(fig,'Theme'), set(fig,'Theme','light'); end
tl = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
title(tl,[runs(1).controller,' | physical dynamics, contact, and analytical checks'],'Interpreter','latex');
labels = cell(1,numel(runs));
for k = 1:numel(runs)
    run = runs(k); labels{k} = sprintf('%s, $k_f=%g$',run.mode,run.kf);
    series = {abs(run.force-controller.fd), abs(run.gap), vecnorm(run.errorDynamicsResidual,2,2), ...
        vecnorm(run.jointResidual,inf,2), vecnorm(run.positionError-run.analyticPositionError,2,2), abs(sin(run.q(:,2)))};
    for j = 1:6
        nexttile(j); hold on;
        if j < 6
            semilogy(run.t,max(series{j},1e-16),'LineWidth',1);
            set(gca,'YScale','log');
        else
            plot(run.t,series{j},'LineWidth',1.2);
        end
        grid on; xlabel('$t$ (s)');
    end
end
titles = {'Normal force error $|f-f_d|$ (N)', 'Contact residual $|n^T(p-p_0)|$ (m)', ...
    'Error ODE residual norm ($\mathrm{m/s^2}$)', 'Joint dynamics residual norm ($\mathrm{N\,m}$)', ...
    'ODE vs analytical position error (m)', 'Nonsingularity margin $|\sin(q_2)|$'};
for j = 1:6
    nexttile(j); title(titles{j});
end
nexttile(6); yline(plant.minSinQ2,'k--','Operating-region boundary');
legend(labels,'Location','best');
hmfc_latex_figure(fig);
if options.saveResults
    exportgraphics(fig,fullfile(options.outputDir,'diagnostics.png'),'Resolution',160);
    savefig(fig,fullfile(options.outputDir,'diagnostics.fig'));
end
end

function results = run_comparison(options)
controllerIds = options.controllerIds;
outputDir = options.outputDir;
makePlots = options.makePlots;
% Run each selected controller independently, then compare every pair.
if options.saveResults && ~exist(outputDir,'dir'), mkdir(outputDir); end
comparisonDir = fullfile(outputDir,'comparison');
if options.saveResults && ~exist(comparisonDir,'dir'), mkdir(comparisonDir); end
controllerResults = cell(1,numel(controllerIds));
[robot,plant,controller,task] = hmfc_parameter_groups();
controller.ids = controllerIds;
results = struct('name','FIC comparison','source','TCST26E3.md / FIC', ...
    'options',options,'robot',robot, ...
    'plant',plant,'controller',controller,'task',task,'runs',[],'allChecksPassed',true);
for controllerIndex = 1:numel(controllerIds)
    controllerId = controllerIds(controllerIndex);
    % Every controller performs its own integration on the same physical plant.
    childOptions = options;
    childOptions.controllerIds = controllerId;
    childOptions.outputDir = fullfile(outputDir,sprintf('fic%d',controllerId));
    run = run_controller(childOptions);
    results.runs = hmfc_append_runs(results.runs,run.runs);
    controllerResults{controllerIndex} = run;
    if controllerIndex==1, results.odeOptions = run.odeOptions; end
    results.(sprintf('fic%d',controllerId)) = run;
    results.allChecksPassed = results.allChecksPassed && run.allChecksPassed;
    controllerSummary = run.summary;
    if controllerIndex == 1
        results.summary = controllerSummary;
    else
        results.summary = [results.summary;controllerSummary];
    end
end

pairs = nchoosek(1:numel(controllerIds),2);
for caseIndex = 1:numel(controllerResults{1}.runs)
    matchedCases = cell(1,numel(controllerIds));
    for controllerIndex = 1:numel(controllerIds)
        assert(numel(controllerResults{controllerIndex}.runs) ...
            == numel(controllerResults{1}.runs), ...
            'FIC:ComparisonCases','The controllers produced different case counts.');
        matchedCases{controllerIndex} = controllerResults{controllerIndex}.runs(caseIndex);
    end
    for pairIndex = 1:size(pairs,1)
        runA = matchedCases{pairs(pairIndex,1)};
        runB = matchedCases{pairs(pairIndex,2)};
        assert(strcmp(runA.mode,runB.mode) && runA.kf == runB.kf && isequal(runA.t,runB.t), ...
            'FIC:ComparisonInputs','Compared cases must share mode, gain, and times.');
        comparison = compare_run(runA,runB);
        if ~isfield(results,'comparison')
            results.comparison = comparison;
        else
            results.comparison(end+1) = comparison;
        end
        fprintf('%s vs %s | %-10s kf=%g: max |delta p|=%.3e m, max |delta tau|=%.3e N m\n', ...
            runA.controller,runB.controller,runA.mode,runA.kf, ...
            comparison.maxPositionDifference,comparison.maxTorqueDifference);
    end
    if makePlots
        plot_side_by_side(matchedCases,plant,controller,options,comparisonDir);
    end
end
results.comparisonSummary = struct2table(results.comparison);
if options.saveResults
    writetable(results.comparisonSummary,fullfile(comparisonDir,'summary.csv'));
    writetable(results.summary,fullfile(outputDir,'summary.csv'));
end
if makePlots
    for pairIndex = 1:size(pairs,1)
        if size(pairs,1) == 1
            name = 'equivalence';
        else
            name = sprintf('equivalence_%d_%d', ...
                controllerIds(pairs(pairIndex,1)),controllerIds(pairs(pairIndex,2)));
        end
        plot_equivalence(controllerResults{pairs(pairIndex,1)}.runs, ...
            controllerResults{pairs(pairIndex,2)}.runs,options,comparisonDir,name);
    end
end
fileTag = ['fic',sprintf('%d',controllerIds)];
results.checks = struct('pairwiseEquivalencePassed',true,'passed',results.allChecksPassed);
if options.saveResults, save(fullfile(outputDir,[fileTag,'_results.mat']),'results'); end
fprintf('All %d controllers and pairwise equivalence checks passed. Results: %s\n', ...
    numel(controllerIds),outputDir);
end

function comparison = compare_run(runA,runB)
comparison.controllerA = runA.controller;
comparison.controllerB = runB.controller;
comparison.mode = runA.mode;
comparison.kf = runA.kf;
comparison.maxReferencePositionDifference = max(vecnorm(runA.reference-runB.reference,2,2));
comparison.maxReferenceVelocityDifference = max(vecnorm(runA.referenceVelocity-runB.referenceVelocity,2,2));
comparison.maxStateDifference = max(vecnorm(runA.state-runB.state,2,2));
comparison.maxPositionDifference = max(vecnorm(runA.position-runB.position,2,2));
comparison.maxVelocityDifference = max(vecnorm(runA.velocity-runB.velocity,2,2));
comparison.maxAccelerationDifference = max(vecnorm(runA.acceleration-runB.acceleration,2,2));
comparison.maxCommandDifference = max(vecnorm(runA.up-runB.up,2,2));
comparison.maxForceCommandDifference = max(abs(runA.forceCommand-runB.forceCommand));
comparison.maxForceDifference = max(abs(runA.force-runB.force));
comparison.maxTorqueDifference = max(vecnorm(runA.tau-runB.tau,2,2));
residuals = [comparison.maxReferencePositionDifference, ...
    comparison.maxReferenceVelocityDifference,comparison.maxStateDifference, ...
    comparison.maxPositionDifference,comparison.maxVelocityDifference, ...
    comparison.maxAccelerationDifference,comparison.maxCommandDifference, ...
    comparison.maxForceCommandDifference,comparison.maxForceDifference, ...
    comparison.maxTorqueDifference];
assert(all(isfinite(residuals)) && all(residuals < 1e-7), ...
    'FIC:Equivalence','Compared controllers differ beyond numerical tolerance.');
end

function plot_side_by_side(cases,plant,controller,options,outputDir)
runA = cases{1};
comparisonTitle = strjoin(cellfun(@(run) run.controller,cases,'UniformOutput',false),' vs ');
columnCount = numel(cases);
fig = figure('Name',sprintf('%s - %s - kf=%g',comparisonTitle,runA.mode,runA.kf), ...
    'Color','w','Visible',options.visible,'Position',[100,60,640*columnCount,1100]);
hmfc_latex_figure(fig);
if isprop(fig,'Theme'), set(fig,'Theme','light'); end
tl = tiledlayout(fig,4,columnCount,'TileSpacing','compact','Padding','compact');
title(tl,sprintf('%s | %s | $k_f=%g$ | identical plant, gains, and reference',comparisonTitle,runA.mode,runA.kf),'Interpreter','latex');
axesHandles = gobjects(4,columnCount);
for column = 1:columnCount
    run = cases{column};
    axesHandles(1,column) = nexttile(column);
    plot(run.t,(run.reference-plant.p0')*plant.T,'k--',run.t,run.state(:,1),'b-','LineWidth',1.4);
    grid on; xlabel('$t$ (s)'); ylabel('$x_t$ (m)');
    title([run.controller,' | Tangential position']);
    legend('Reference','Actual','Location','best');

    axesHandles(2,column) = nexttile(columnCount+column);
    semilogy(run.t,max(vecnorm(run.positionError,2,2),1e-16),'b-', ...
        run.t,max(vecnorm(run.analyticPositionError,2,2),1e-16),'k--','LineWidth',1.2);
    grid on; xlabel('$t$ (s)'); ylabel('$\|e_p\|$ (m)');
    title([run.controller,' | Position error']);
    legend('ODE','Analytical','Location','best');

    axesHandles(3,column) = nexttile(2*columnCount+column);
    plot(run.t,run.force,'b-','LineWidth',1.4); hold on;
    yline(controller.fd,'k--'); ylim([controller.fd-1,controller.fd+1]);
    grid on; xlabel('$t$ (s)'); ylabel('$f$ (N)');
    title([run.controller,' | Normal contact force']);
    legend('Actual','Desired','Location','best');

    axesHandles(4,column) = nexttile(3*columnCount+column);
    plot(run.t,run.tau(:,1),'b-',run.t,run.tau(:,2),'r-','LineWidth',1.2);
    grid on; xlabel('$t$ (s)'); ylabel('Joint torque ($\mathrm{N\,m}$)');
    title([run.controller,' | Applied torques']);
    legend('$\tau_1$','$\tau_2$','Location','best');
end
% Matched rows share limits so the visual comparison uses the same scales.
for row = 1:4
    limits = zeros(columnCount,2);
    for column = 1:columnCount
        limits(column,:) = ylim(axesHandles(row,column));
    end
    sharedLimits = [min(limits(:,1)),max(limits(:,2))];
    linkaxes(axesHandles(row,:),'xy');
    ylim(axesHandles(row,1),sharedLimits);
end
name = sprintf('%s_kf_%g',runA.mode,runA.kf);
hmfc_latex_figure(fig);
if options.saveResults
    exportgraphics(fig,fullfile(outputDir,[name,'.png']),'Resolution',160);
    savefig(fig,fullfile(outputDir,[name,'.fig']));
end
end

function plot_equivalence(fic1Cases,fic2Cases,options,outputDir,name)
comparisonTitle = [fic1Cases(1).controller,' vs ',fic2Cases(1).controller];
fig = figure('Name',[comparisonTitle,' - numerical differences'], ...
    'Color','w','Visible',options.visible,'Position',[120,80,1250,850]);
hmfc_latex_figure(fig);
if isprop(fig,'Theme'), set(fig,'Theme','light'); end
tl = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
title(tl,[comparisonTitle,' | numerical differences (display floor: $10^{-16}$)'],'Interpreter','latex');
labels = cell(1,numel(fic1Cases));
for i = 1:numel(fic1Cases)
    runA = fic1Cases(i); runB = fic2Cases(i);
    labels{i} = sprintf('%s, $k_f=%g$',runA.mode,runA.kf);
    series = {vecnorm(runA.position-runB.position,2,2),vecnorm(runA.up-runB.up,2,2), ...
        abs(runA.force-runB.force),vecnorm(runA.tau-runB.tau,2,2)};
    for panel = 1:4
        nexttile(panel); hold on;
        semilogy(runA.t,max(series{panel},1e-16),'LineWidth',1.2);
        set(gca,'YScale','log'); ylim([1e-16,1e-6]);
        grid on; xlabel('$t$ (s)');
    end
end
titles = {'Position difference norm (m)','Acceleration command difference norm ($\mathrm{m/s^2}$)', ...
    'Contact force difference (N)','Joint torque difference norm ($\mathrm{N\,m}$)'};
for panel = 1:4
    nexttile(panel); title(titles{panel});
    yline(1e-7,'k--','Check tolerance');
end
nexttile(1); legend(labels,'Location','best');
hmfc_latex_figure(fig);
if options.saveResults
    exportgraphics(fig,fullfile(outputDir,[name,'.png']),'Resolution',160);
    savefig(fig,fullfile(outputDir,[name,'.fig']));
end
end
