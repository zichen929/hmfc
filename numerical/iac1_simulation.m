function results = iac1_simulation(options)
%IAC1_SIMULATION Standalone ODE verification of TCST26, IAC 1.
%   results = iac1_simulation();
%   results = iac1_simulation(struct('makePlots',false,'saveResults',false));
% Independent controller, estimator and contact solve with a shared robot model.
% Source: obsidian-vault/04-writing/tcst26/TCST26E3.md, heading TCST26.
% The ODE integrates [x_t; dx_t; hat_theta], NOT the derived z dynamics.
% The maintained rigid-contact branch is parameterized exactly by x_t.

if nargin == 0, options = struct(); end
common = hmfc_parameters();
defaults = struct('makePlots',true,'saveResults',true,'visible','on', ...
    'outputDir',fullfile(hmfc_numerical_root(),'results','iac1'), ...
    'initialAngleDeg',5,'initialVelocity',common.initialVelocity,'duration',common.duration, ...
    'sectorHalfWidthDeg',6,'sectorMarginDeg',1e-6);
options = hmfc_simulation_options(options,defaults);
validateattributes(options.initialAngleDeg,{'numeric'},{'real','finite','scalar'});
validateattributes(options.initialVelocity,{'numeric'},{'real','finite','scalar'});
validateattributes(options.duration,{'numeric'},{'real','finite','scalar','>=',6});
validateattributes(options.sectorMarginDeg,{'numeric'},{'real','finite','scalar','positive'});
results = run_controller(options);
end

function results = run_controller(options)
common = hmfc_parameters();
[robot,plant,controller,task] = hmfc_parameter_groups();
% Controller data deliberately contain neither the true n nor true theta.
controller.n0 = [0;1]; controller.thetaC = 0; controller.delta = 6*pi/180; controller.gamma = 0.4;
controller.Tc = [0,1;-1,0]*rotation(controller.thetaC)*controller.n0;

% Uniform analytic inertia bounds on q2 in [70,115] deg, any q1.
% A=I1+m1*lc1^2+m2*l1^2, B=m2*l1*lc2, D=I2+m2*lc2^2.
% trace(Lambda)=(A/l1^2+D/l2^2-2*B*cos(q2)^2/(l1*l2))/sin(q2)^2.
% det(Lambda)=(A*D-B^2*cos(q2)^2)/(l1^2*l2^2*sin(q2)^2).
% lambda_max <= traceUpper; lambda_min >= detLower/traceUpper.
domain = struct('q2Min',common.q2Min,'q2Max',common.q2Max, ...
    'xMin',common.xMin,'xMax',common.xMax);
A = robot.I1+robot.m1*robot.lc1^2+robot.m2*robot.l1^2;
B = robot.m2*robot.l1*robot.lc2;
D = robot.I2+robot.m2*robot.lc2^2;
cmax = max(abs(cos([domain.q2Min,domain.q2Max])));
smin2 = 1-cmax^2;
massMax = (A/robot.l1^2+D/robot.l2^2)/smin2;
detLower = (A*D-B^2*cmax^2)/(robot.l1^2*robot.l2^2);
massMin = detLower/massMax;
chi = (massMax-massMin)/(2*sqrt(massMax*massMin));
deltaSupremum=0.5*atan2(1,chi);
if isequal(options.sectorHalfWidthDeg,'supremum')
    controller.delta=deltaSupremum-options.sectorMarginDeg*pi/180;
else
    validateattributes(options.sectorHalfWidthDeg,{'numeric'},{'real','finite','scalar','positive'});
    assert(options.sectorHalfWidthDeg<deltaSupremum*180/pi,'IAC1:Sector', ...
        'Sector half-width must be strictly below %.12g degrees.',deltaSupremum*180/pi);
    controller.delta=options.sectorHalfWidthDeg*pi/180;
end
assert(controller.delta>0 && controller.delta<deltaSupremum,'IAC1:Sector', ...
    'Sector half-width must be strictly below %.12g degrees.',deltaSupremum*180/pi);
rhoMin = cos(2*controller.delta)-chi*sin(2*controller.delta);
assert(massMin>0 && rhoMin>0,'Angular sector cannot certify positive contact.');
assert(abs(options.initialAngleDeg*pi/180)<=controller.delta, ...
    'Initial effective angle must be inside the +/-%.12g degree sector.',controller.delta*180/pi);
certificate = struct('massMin',massMin,'massMax',massMax,'chi',chi, ...
    'rhoMin',rhoMin,'forceMin',rhoMin*controller.fd, ...
    'alphaZ',controller.gamma*rhoMin*controller.fd*cos(controller.delta),'domain',domain, ...
    'sectorHalfWidthSupremumDeg',deltaSupremum*180/pi, ...
    'sectorHalfWidthUsedDeg',controller.delta*180/pi);

% The first case is the requested scenario. Opposite sign, invariant
% manifold (nonzero momentum), and regulation from rest are cross-checks.
cases = struct('name',{'positive_tilt','negative_tilt','on_manifold','at_rest'}, ...
    'angleDeg',{options.initialAngleDeg,-abs(options.initialAngleDeg),0,options.initialAngleDeg}, ...
    'velocity',{options.initialVelocity,options.initialVelocity,0.02,0}, ...
    'targetX',{common.targetX,common.targetX,common.targetX,0});
results = struct('source','TCST26E3.md / TCST26 / IAC 1', ...
    'name','IAC 1','options',options,'robot',robot,'plant',plant,'controller',controller, ...
    'task',task, ...
    'certificate',certificate,'runs',[]);
odeOptions = odeset('RelTol',1e-9,'AbsTol',[1e-11,1e-11,1e-12], ...
    'MaxStep',0.01,'Events',@(t,state) domain_events(t,state,robot,plant,controller,task,domain));
results.odeOptions = odeOptions;
tRequested = linspace(0,options.duration,ceil(options.duration/common.outputStep)+1)';
fprintf('\nIAC 1: horizontal rigid contact, shared 2R model\n');
fprintf('Certified f >= %.9g N; alpha_z = %.9g 1/s\n', ...
    certificate.forceMin,certificate.alphaZ);
for k = 1:numel(cases)
    spec = cases(k);
    caseTask = task; caseTask.targetX = spec.targetX;
    [~,~,initialModel] = measured_state(0,spec.velocity,robot,plant);
    % hat_theta(0)=bar_theta(0)+gamma*Tc'*Lambda(0)*dp(0).
    hat0 = spec.angleDeg*pi/180+controller.gamma*controller.Tc'*initialModel.Lambda* ...
        (plant.T*spec.velocity);
    y0 = [0;spec.velocity;hat0];
    rhs = @(t,state) closed_loop(t,state,robot,plant,controller,caseTask,'regulation');
    caseOdeOptions = odeset(odeOptions,'Events', ...
        @(t,state) domain_events(t,state,robot,plant,controller,caseTask,domain));
    [t,state,te,~,ie] = ode45(rhs,tRequested,y0,caseOdeOptions);
    assert(isempty(te),'Contact/sector/robot domain event %d at t=%.6g.', ...
        first_or_zero(ie),first_or_zero(te));
    assert(abs(t(end)-options.duration)<1e-10,'ODE ended prematurely.');
    run = postprocess(t,state,robot,plant,controller,caseTask,'regulation',certificate);
    run.name = spec.name;
    run.spec = spec;
    run.initialState = y0;
    run.checks = verify_run(run,robot,plant,controller,certificate);
    results.runs = [results.runs;run];
    fprintf('%-14s z: %+.3f -> %+.3e deg, e: %+.3e m, f: %.9f N, min f: %.6f N\n', ...
        run.name,run.angleError(1)*180/pi,run.angleError(end)*180/pi, ...
        run.error(end),run.force(end),min(run.force));
end

results.summary = hmfc_run_summary(results.runs);
results.checks = struct('passed',all(arrayfun(@(run) run.checks.passed,results.runs)));
results.allChecksPassed = results.checks.passed;
if options.saveResults
    if ~exist(options.outputDir,'dir'), mkdir(options.outputDir); end
    writetable(results.summary,fullfile(options.outputDir,'summary.csv'));
    for k = 1:numel(results.runs)
        run = results.runs(k);
        signals = hmfc_run_signals(run);
        writetable(signals,fullfile(options.outputDir,[run.name,'.csv']));
    end
    save(fullfile(options.outputDir,'iac1_results.mat'),'results');
    write_report(results);
end
if options.makePlots, make_plots(results); end
fprintf('All %d cases passed the numerical verification checks.\n\n',numel(cases));
end

function dstate = closed_loop(t,state,robot,plant,controller,task,mode)
evaluation = evaluate(t,state,robot,plant,controller,task,mode);
dstate = [state(2);plant.T'*evaluation.acceleration;evaluation.dhatTheta];
end

function evaluation = evaluate(t,state,robot,plant,controller,task,mode)
[position,velocity,model] = measured_state(state(1),state(2),robot,plant);
ref = reference_command(t,task,mode,controller);
cmd = controller_command(position,velocity,ref,model,controller,state(3));
[acceleration,force,ddq] = contact_plant(cmd,model,plant.n);
dhatTheta = estimator_update(cmd,force,velocity,model,controller);
dmomentum = model.dLambda*velocity+model.Lambda*acceleration;
gp = cmd.F-model.eta+model.dLambda*velocity;
trueTheta = atan2(-plant.n(1),plant.n(2));
z = cmd.barTheta-trueTheta;
lt = plant.T'*model.Lambda*plant.T;
coupling = (plant.T'*model.Lambda*plant.n)/lt;
eigenvalues = eig(model.Lambda);
forceRatio = force/controller.fd;
rhoFormula = cos(z)+coupling*sin(z);
evaluation = struct('position',position,'velocity',velocity,'acceleration',acceleration, ...
    'reference',cmd.reference,'referenceVelocity',cmd.referenceVelocity, ...
    'referenceAcceleration',cmd.referenceAcceleration, ...
    'positionError',cmd.positionError,'velocityError',cmd.velocityError, ...
    'q',model.q,'dq',model.dq,'ddq',ddq,'up',cmd.up,'tau',cmd.tau, ...
    'force',force,'forceCommand',cmd.forceCommand, ...
    'beta',cmd.beta,'barTheta',cmd.barTheta,'angle',cmd.barTheta,'angleError',z, ...
    'error',plant.T'*cmd.positionError,'rho',forceRatio, ...
    'rhoFormula',rhoFormula,'forceRatioResidual',forceRatio-rhoFormula,'dhatTheta',dhatTheta, ...
    'dz',dhatTheta-controller.gamma*controller.Tc'*dmomentum, ...
    'dzTheory',-controller.gamma*force*(sin(cmd.barTheta-controller.thetaC)- ...
        sin(trueTheta-controller.thetaC)), ...
    'momentumResidual',norm(dmomentum-gp-plant.n*force,inf), ...
    'jointResidual',model.M*ddq+model.C*model.dq+model.G-cmd.tau-model.J'*plant.n*force, ...
    'cartesianResidual',model.Lambda*acceleration+model.eta-cmd.F-plant.n*force, ...
    'motionResidual',plant.T'*(acceleration-cmd.up)-cmd.forceCommand*sin(z)/lt, ...
    'q2',model.q(2),'minLambda',min(eigenvalues),'maxLambda',max(eigenvalues), ...
    'detJ',det(model.J),'gap',plant.n'*(position-plant.p0), ...
    'normalVelocity',plant.n'*velocity,'normalAcceleration',plant.n'*acceleration);
end

function cmd = controller_command(position,velocity,ref,model,controller,hatTheta)
% Only measured p, velocity, internal state, known model and reference.
cmd.reference = ref.pd; cmd.referenceVelocity = ref.dpd; cmd.referenceAcceleration = ref.ddpd;
cmd.positionError = ref.pd-position; cmd.velocityError = ref.dpd-velocity;
cmd.beta = -controller.gamma*controller.Tc'*model.Lambda*velocity;
cmd.barTheta = hatTheta+cmd.beta;
cmd.nbar = rotation(cmd.barTheta)*controller.n0;
% IAC 1 is a regulation law; its generated reference derivatives are zero.
cmd.up = controller.kp*cmd.positionError-controller.kv*velocity;
cmd.forceCommand = controller.fd;
cmd.forceFeedback = zeros(2,1);
cmd.F0 = model.eta+model.Lambda*cmd.up-cmd.nbar*cmd.forceCommand;
cmd.F = cmd.F0;
cmd.tau = model.J'*cmd.F;
end

function ref = reference_command(~,task,mode,~)
assert(strcmp(mode,'regulation'),'IAC1:Reference','Only the matched regulation task is supported.');
ref = struct('pd',task.origin+task.axis*task.targetX,'dpd',zeros(2,1),'ddpd',zeros(2,1));
end

function dhatTheta = estimator_update(cmd,force,velocity,model,controller)
% Scalar force sensor only; no true normal, z, acceleration or force derivative.
gp = cmd.F-model.eta+model.dLambda*velocity;
dhatTheta = controller.gamma*controller.Tc'*(gp+cmd.nbar*force);
end

function [acceleration,force,ddq] = contact_plant(cmd,model,n)
% Original joint dynamics and rigid acceleration constraint; force is solved.
Jn = n'*model.J;
solution = [model.M,-model.J'*(n+cmd.forceFeedback);Jn,0]\ ...
    [model.J'*cmd.F0-model.C*model.dq-model.G;-n'*model.dJdq];
ddq = solution(1:2); force = solution(3);
acceleration = model.J*ddq+model.dJdq;
end

function [position,velocity,model] = measured_state(x,tangentialVelocity,robot,plant)
% Minimal-coordinate rigid plant: no normal drift and no Baumgarte gains.
position = plant.p0+plant.T*x;
velocity = plant.T*tangentialVelocity;
model = hmfc_robot_state(position,velocity,robot);
end

function [value,isterminal,direction] = domain_events(t,state,robot,plant,controller,task,domain)
evaluation = evaluate(t,state,robot,plant,controller,task,'regulation');
value = [evaluation.force;domain.xMax-state(1);state(1)-domain.xMin; ...
    evaluation.q(2)-domain.q2Min;domain.q2Max-evaluation.q(2); ...
    controller.delta-abs(evaluation.barTheta-controller.thetaC)];
isterminal = ones(size(value));
direction = -ones(size(value));
end

function run = postprocess(t,state,robot,plant,controller,task,mode,cert)
run = struct('controller','IAC 1','mode',mode,'kf',0,'t',t,'state',state, ...
    'initialState',state(1,:)');
for i = 1:numel(t)
    evaluation = evaluate(t(i),state(i,:)',robot,plant,controller,task,mode);
    if i == 1
        fields = fieldnames(evaluation);
        for j = 1:numel(fields)
            run.(fields{j}) = zeros(numel(t),numel(evaluation.(fields{j})));
        end
    end
    for j = 1:numel(fields), run.(fields{j})(i,:) = evaluation.(fields{j})(:)'; end
end
run.x = state(:,1);
run.hatTheta = state(:,3);
run.angleEnvelope = abs(run.angleError(1))*exp(-cert.alphaZ*t);
run = hmfc_run_region(run,plant);
run.metrics = hmfc_run_metrics(run,plant,controller);
end

function checks = verify_run(run,robot,plant,controller,cert)
checks = struct();
checks.maxCartesianResidual = max(vecnorm(run.cartesianResidual,inf,2));
checks.maxJointResidual = max(vecnorm(run.jointResidual,inf,2));
checks.maxMomentumResidual = max(run.momentumResidual);
checks.maxAngleIdentityResidual = max(abs(run.dz-run.dzTheory));
checks.maxForceRatioResidual = max(abs(run.forceRatioResidual));
checks.maxMotionIdentityResidual = max(abs(run.motionResidual));
checks.operatingRegionSatisfied = run.operatingRegionSatisfied;
checks.maxNormalAcceleration = max(abs(run.normalAcceleration));
checks.maxGap = max(abs(run.gap));
checks.maxNormalVelocity = max(abs(run.normalVelocity));
checks.minForce = min(run.force);
checks.maxEnvelopeExcess = max(abs(run.angleError)-run.angleEnvelope);
% Independent finite difference of Lambda along a nonzero-velocity model.
[~,~,model] = measured_state(0,0.07,robot,plant);
epsilon = 1e-5;
mp = hmfc_robot_model(model.q+epsilon*model.dq,model.dq,robot);
mm = hmfc_robot_model(model.q-epsilon*model.dq,model.dq,robot);
checks.dLambdaFiniteDifference = norm((mp.Lambda-mm.Lambda)/(2*epsilon)-model.dLambda,inf);
% Central difference of the INTEGRATED effective angle, excluding endpoints.
timeStep = run.t(2)-run.t(1);
dzNumeric = (run.angleError(3:end)-run.angleError(1:end-2))/(2*timeStep);
checks.integratedAngleDerivativeResidual = max(abs(dzNumeric-run.dzTheory(2:end-1)));
assert(checks.maxCartesianResidual<1e-9 && checks.maxJointResidual<1e-9);
assert(checks.maxMomentumResidual<1e-9 && checks.maxAngleIdentityResidual<1e-9);
% This residual compares force ratios and is dimensionless.
forceRatioResidualTolerance = 1e-10;
assert(checks.maxForceRatioResidual<forceRatioResidualTolerance,'IAC1:ForceRatioResidual', ...
    'Dimensionless force-ratio residual exceeds %g.',forceRatioResidualTolerance);
assert(checks.maxMotionIdentityResidual<1e-9);
assert(checks.dLambdaFiniteDifference<1e-7);
assert(checks.integratedAngleDerivativeResidual<1e-4);
assert(checks.maxGap<1e-12 && checks.maxNormalVelocity<1e-12 && checks.maxNormalAcceleration<1e-9);
assert(checks.minForce>=cert.forceMin-1e-8);
assert(checks.maxEnvelopeExcess<2e-9);
assert(all(abs(run.barTheta-controller.thetaC)<=controller.delta+1e-9));
assert(all(run.minLambda>=cert.massMin) && all(run.maxLambda<=cert.massMax));
assert(all(run.q2>cert.domain.q2Min & run.q2<cert.domain.q2Max));
assert(all(run.state(:,1)>cert.domain.xMin & run.state(:,1)<cert.domain.xMax));
assert(abs(run.angleError(end))<1e-6,'Effective angle did not converge.');
assert(abs(run.error(end))<1e-6,'Motion error did not converge.');
assert(abs(run.state(end,2))<1e-6,'Tangential velocity did not converge.');
assert(abs(run.force(end)-controller.fd)<1e-5,'Contact force did not converge.');
if strcmp(run.name,'on_manifold')
    assert(max(abs(run.angleError))<2e-9,'I&I manifold was not invariant.');
    assert(abs(run.beta(1))>1e-4,'Manifold check requires nonzero correction.');
end
checks.passed = true;
end

function make_plots(results)
options = results.options;
run = results.runs(1);
controller = results.controller;
figureHandle = figure('Name','IAC 1: horizontal surface','Color','w','Visible',options.visible, ...
    'Position',[100,100,1200,800]);
hmfc_latex_figure(figureHandle);
tiledlayout(2,3,'TileSpacing','compact');
nexttile; plot(run.t,[run.state(:,3),run.beta,run.barTheta]*180/pi,'LineWidth',1.2);
yline(0,'k:'); grid on; xlabel('$t$ (s)'); ylabel('Angle (${}^\circ$)');
legend('$\hat{\theta}$','$\beta$','$\bar{\theta}$','True $\theta$'); title('Implemented angle estimate');
nexttile; semilogy(run.t,max(abs(run.angleError),1e-15)*180/pi,'LineWidth',1.3); hold on;
semilogy(run.t,max(run.angleEnvelope,1e-15)*180/pi,'--','LineWidth',1.2);
grid on; xlabel('$t$ (s)'); ylabel('$|z|$ (${}^\circ$)'); legend('Actual','Certified envelope');
title('Effective angle convergence');
nexttile; plot(run.t,run.force,'LineWidth',1.3); hold on;
yline(controller.fd,'k--'); yline(results.certificate.forceMin,'r:'); grid on;
xlabel('$t$ (s)'); ylabel('Force (N)'); legend('Reaction','Desired','Certified minimum');
title('Scalar normal force');
nexttile; plot(run.t,1000*run.error,'LineWidth',1.3); grid on;
xlabel('$t$ (s)'); ylabel('Motion error (mm)'); title('Tangential regulation');
nexttile; hold on; colors = lines(numel(results.runs));
for k = 1:numel(results.runs)
    plot(results.runs(k).t,results.runs(k).angleError*180/pi, ...
        'Color',colors(k,:),'LineWidth',1.2,'DisplayName',strrep(results.runs(k).name,'_','\_'));
end
grid on; xlabel('$t$ (s)'); ylabel('$z$ (${}^\circ$)'); legend('show');
title('Cross-check scenarios');
nexttile; plot(run.t,run.tau,'LineWidth',1.2); grid on;
xlabel('$t$ (s)'); ylabel('Torque ($\mathrm{N\,m}$)'); legend('$\tau_1$','$\tau_2$');
title('Actuator torques');
sgtitle(sprintf('IAC 1 | true horizontal surface | initial effective tilt $%+.1f^\\circ$', ...
    results.runs(1).spec.angleDeg),'Interpreter','latex');
hmfc_latex_figure(figureHandle);
if options.saveResults
    exportgraphics(figureHandle,fullfile(options.outputDir,'iac1_summary.png'),'Resolution',180);
    savefig(figureHandle,fullfile(options.outputDir,'iac1_summary.fig'));
end
end

function write_report(results)
path = fullfile(results.options.outputDir,'verification.txt');
fid = fopen(path,'w');
assert(fid>=0,'Could not open verification output.');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'IAC 1 independent ODE verification\nSource: %s\n',results.source);
fprintf(fid,'ODE state: [x_t, dx_t, hat_theta]; exact active rigid contact.\n');
fprintf(fid,'True surface: y=0.4 m, n=[0;1]. Sector: +/-%.12g deg.\n', ...
    results.controller.delta*180/pi);
fprintf(fid,'Analytic inertia bounds: [%.9g, %.9g] kg\n', ...
    results.certificate.massMin,results.certificate.massMax);
fprintf(fid,'Certified force minimum: %.9g N; alpha_z: %.9g /s\n', ...
    results.certificate.forceMin,results.certificate.alphaZ);
for k = 1:numel(results.runs)
    run = results.runs(k);
    fprintf(fid,'\n%s\n',run.name);
    fprintf(fid,'Initial/final z: %.9g / %.9g deg\n',run.angleError(1)*180/pi,run.angleError(end)*180/pi);
    fprintf(fid,'Final motion error: %.9g m; velocity: %.9g m/s\n',run.error(end),run.state(end,2));
    fprintf(fid,'Final/min force: %.9g / %.9g N\n',run.force(end),min(run.force));
    fields = fieldnames(run.checks);
    for j = 1:numel(fields)
        fprintf(fid,'%s: %.9g\n',fields{j},run.checks.(fields{j}));
    end
end
fprintf(fid,'\nScope: exact model and scalar force, ideal actuation, established contact only.\n');
end

function R = rotation(theta)
R = [cos(theta),-sin(theta);sin(theta),cos(theta)];
end

function value = first_or_zero(values)
if isempty(values), value = 0; else, value = values(1); end
end
