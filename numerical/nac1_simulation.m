function results = nac1_simulation(options)
%NAC1_SIMULATION FIC 1 with a fixed, inaccurate surface-normal estimate.
%   results = nac1_simulation;
%   results = nac1_simulation(struct('visible','off'));
%   Independent controller and contact solve; shared robot model and defaults.
%   ODE state: [x_t; dx_t]. The normal estimate never changes.
%   Main case: regulation, kf=0, fixed +5 deg estimate, same scene as IAC 1.
%   Use fic1_nac1_iac1_simulation for the separate three-controller runner.

if nargin==0, options=struct(); end
common = hmfc_parameters();
defaults = struct('makePlots',true,'saveResults',true,'visible','on', ...
    'outputDir',fullfile(hmfc_numerical_root(),'results','nac1'), ...
    'initialAngleDeg',5,'initialVelocity',common.initialVelocity,'duration',common.duration, ...
    'initialAngleLimitDeg',6,'enforceOperatingRegion',true);
options = hmfc_simulation_options(options,defaults);
validateattributes(options.initialAngleLimitDeg,{'numeric'},{'real','finite','scalar','positive','<',90});
validateattributes(options.initialAngleDeg,{'numeric'},{'real','finite','scalar', ...
    '>=',-options.initialAngleLimitDeg,'<=',options.initialAngleLimitDeg});
validateattributes(options.initialVelocity,{'numeric'},{'real','finite','scalar'});
validateattributes(options.duration,{'numeric'},{'real','finite','scalar','>=',6});
validateattributes(options.enforceOperatingRegion,{'logical','numeric'},{'scalar','binary'});
results = run_controller(options);
end

function results = run_controller(options)
common = hmfc_parameters();
[robot,plant,controller,task] = hmfc_parameter_groups();
plant.enforceOperatingRegion = options.enforceOperatingRegion;
% Controller parameters deliberately contain no true normal or tangent.
controller.fixedAngle = options.initialAngleDeg*pi/180;
controller.nHat = [-sin(controller.fixedAngle);cos(controller.fixedAngle)];
time = linspace(0,options.duration,ceil(options.duration/common.outputStep)+1)';
odeOptions = odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.01);
results = struct('name','NAC 1','options',options,'robot',robot, ...
    'plant',plant,'controller',controller,'task',task,'odeOptions',odeOptions, ...
    'source','TCST26E3.md / FIC 1, with fixed estimated normal','runs',[]);
fprintf('\nNAC 1: fixed normal estimate %+.3f deg, true horizontal surface\n',options.initialAngleDeg);
mode = 'regulation';
for kf=[0,2]
    rhs = @(t,state) closed_loop(t,state,robot,plant,controller,task,mode,kf);
    [t,state] = ode45(rhs,time,[0;options.initialVelocity],odeOptions);
    run = postprocess(t,state,robot,plant,controller,task,mode,kf);
    run.checks = verify_run(run,robot,plant,controller,task);
    results.runs = [results.runs;run];
    fprintf('%-10s kf=%g: final e=%+.6f mm, f=%.9f N; predicted e_inf=%+.6f mm\n', ...
        mode,kf,run.error(end)*1000,run.force(end),run.equilibrium.error*1000);
    if ~run.operatingRegionSatisfied
        fprintf('  Original certificate region exited at approximately %.6f s; exact robot model continued.\n', ...
            run.firstRegionExitTime);
    end
end
results.summary = hmfc_run_summary(results.runs);
results.checks = struct('passed',all(arrayfun(@(run) run.checks.passed,results.runs)));
results.allChecksPassed = results.checks.passed;
if options.saveResults
    if ~exist(options.outputDir,'dir'), mkdir(options.outputDir); end
    writetable(results.summary,fullfile(options.outputDir,'summary.csv'));
    for i=1:numel(results.runs)
        run=results.runs(i);
        signals = hmfc_run_signals(run);
        writetable(signals,fullfile(options.outputDir,sprintf('%s_kf_%g.csv',run.mode,run.kf)));
    end
    save(fullfile(options.outputDir,'nac1_results.mat'),'results');
    write_report(results);
end
if options.makePlots, make_plots(results); end
fprintf('Both NAC 1 regulation cases passed dynamics, force, and bias checks.\n');
end

function dstate = closed_loop(t,state,robot,plant,controller,task,mode,kf)
evaluation=evaluate(t,state,robot,plant,controller,task,mode,kf);
dstate=[state(2);plant.T'*evaluation.acceleration];
end

function evaluation = evaluate(t,state,robot,plant,controller,task,mode,kf)
[position,velocity,model]=measured_state(state(1),state(2),robot,plant);
ref=reference_command(t,task,mode,controller);
cmd=controller_command(position,velocity,ref,model,controller,kf,0);
% F = F0 + nHat*kf*f. Solve the instantaneous scalar-force feedback loop
% together with the ORIGINAL joint dynamics and true rigid constraint.
[acceleration,force,ddq]=contact_plant(cmd,model,plant.n);
cmd=controller_command(position,velocity,ref,model,controller,kf,force);
z=controller.fixedAngle-atan2(-plant.n(1),plant.n(2));
lt=plant.T'*model.Lambda*plant.T;
rho=cos(z)+(plant.T'*model.Lambda*plant.n)/lt*sin(z);
expectedForce=(1+kf)*rho/(1+kf*rho)*controller.fd;
evaluation=struct('position',position,'velocity',velocity,'reference',cmd.reference,'referenceVelocity',cmd.referenceVelocity, ...
    'acceleration',acceleration,'force',force,'forceCommand',cmd.forceCommand,'tau',cmd.tau, ...
    'error',plant.T'*cmd.positionError,'angle',controller.fixedAngle,'rho',rho,'q',model.q,'dq',model.dq,'ddq',ddq, ...
    'referenceAcceleration',cmd.referenceAcceleration,'positionError',cmd.positionError,'velocityError',cmd.velocityError,'up',cmd.up, ...
    'jointResidual',model.M*ddq+model.C*model.dq+model.G-cmd.tau-model.J'*plant.n*force, ...
    'cartesianResidual',model.Lambda*acceleration+model.eta-cmd.F-plant.n*force, ...
    'forceResidual',force-expectedForce, ...
    'motionResidual',plant.T'*(acceleration-cmd.up)-cmd.forceCommand*sin(z)/lt, ...
    'gap',plant.n'*(position-plant.p0),'normalVelocity',plant.n'*velocity, ...
    'normalAcceleration',plant.n'*acceleration);
end

function [acceleration,force,ddq]=contact_plant(cmd,model,n)
% Joint-space constrained solve includes the instantaneous force-feedback loop.
Jn=n'*model.J;
solution=[model.M,-model.J'*(n+cmd.forceFeedback);Jn,0]\ ...
    [model.J'*cmd.F0-model.C*model.dq-model.G;-n'*model.dJdq];
ddq=solution(1:2); force=solution(3);
assert(force>0,'NAC1:ContactLoss','Calculated reaction is not compressive.');
acceleration=model.J*ddq+model.dJdq;
end

function cmd=controller_command(position,velocity,ref,model,controller,kf,force)
% Exact FIC 1 Cartesian motion law; only n is replaced by fixed nHat.
% No true geometry or adaptive state enters this function.
% force is the instantaneous scalar contact-force measurement.
cmd.reference=ref.pd; cmd.referenceVelocity=ref.dpd; cmd.referenceAcceleration=ref.ddpd;
cmd.positionError=ref.pd-position; cmd.velocityError=ref.dpd-velocity;
cmd.up=ref.ddpd+controller.kv*cmd.velocityError+controller.kp*cmd.positionError;
cmd.F0=model.eta+model.Lambda*cmd.up-controller.nHat*(1+kf)*controller.fd;
cmd.forceCommand=controller.fd+kf*(controller.fd-force);
cmd.forceFeedback=controller.nHat*kf;
cmd.F=cmd.F0+controller.nHat*kf*force;
cmd.tau=model.J'*cmd.F;
end

function ref=reference_command(~,task,mode,~)
assert(strcmp(mode,'regulation'),'NAC1:Reference','Only the matched regulation task is supported.');
ref=struct('pd',task.origin+task.axis*task.targetX,'dpd',zeros(2,1),'ddpd',zeros(2,1));
end

function [position,velocity,model]=measured_state(x,tangentialVelocity,robot,plant)
if plant.enforceOperatingRegion
    assert(x>plant.xMin && x<plant.xMax,'NAC1:Region','Tangential domain exceeded.');
end
position=plant.p0+plant.T*x;
velocity=plant.T*tangentialVelocity;
model=hmfc_robot_state(position,velocity,robot);
if plant.enforceOperatingRegion
    assert(model.q(2)>plant.q2Min && model.q(2)<plant.q2Max,'NAC1:Region','Elbow domain exceeded.');
end
end

function eq=equilibrium(robot,plant,controller,task,kf)
% Offline prediction: true geometry is used here only for verification.
% Variable inertia makes the regulation equilibrium an implicit equation.
bracket=[plant.xMin+1e-8,plant.xMax-1e-8];
if ~plant.enforceOperatingRegion
    % The certificate region is not a physical wall. Find the equilibrium
    % on the same exact robot's reachable, nonsingular contact branch.
    reach=sqrt((robot.l1+robot.l2)^2-plant.p0(2)^2)-1e-3;
    physical=[-reach-plant.p0(1),reach-plant.p0(1)];
    for k=1:20
        if balance(bracket(1))*balance(bracket(2))<=0, break; end
        bracket=[max(physical(1),bracket(1)-0.04),min(physical(2),bracket(2)+0.04)];
    end
end
xstar=fzero(@balance,bracket);
[~,uf,force]=balance(xstar);
eq=struct('x',xstar,'error',task.targetX-xstar,'force',force,'forceCommand',uf);
    function [value,uf,force]=balance(x)
        [~,~,model]=measured_state(x,0,robot,plant);
        z=controller.fixedAngle-atan2(-plant.n(1),plant.n(2));
        lt=plant.T'*model.Lambda*plant.T;
        rho=cos(z)+(plant.T'*model.Lambda*plant.n)/lt*sin(z);
        uf=(1+kf)*controller.fd/(1+kf*rho);
        force=rho*uf;
        value=controller.kp*(task.targetX-x)+uf*sin(z)/lt;
    end
end

function run=postprocess(t,state,robot,plant,controller,task,mode,kf)
run=struct('controller','NAC 1','name',sprintf('%s_kf_%g',mode,kf), ...
    'mode',mode,'kf',kf,'t',t,'state',state,'initialState',state(1,:)');
for i=1:numel(t)
    evaluation=evaluate(t(i),state(i,:)',robot,plant,controller,task,mode,kf);
    if i==1
        fields=fieldnames(evaluation);
        for j=1:numel(fields)
            run.(fields{j})=zeros(numel(t),numel(evaluation.(fields{j})));
        end
    end
    for j=1:numel(fields), run.(fields{j})(i,:)=evaluation.(fields{j})(:)'; end
end
run.equilibrium=equilibrium(robot,plant,controller,task,kf);
run.x=state(:,1);
run=hmfc_run_region(run,plant);
run.metrics=hmfc_run_metrics(run,plant,controller);
run.metrics.predictedRegulationError=run.equilibrium.error;
run.metrics.predictedRegulationForce=run.equilibrium.force;
end

function checks=verify_run(run,robot,plant,controller,task)
checks=struct('maxJointResidual',max(vecnorm(run.jointResidual,inf,2)), ...
    'maxCartesianResidual',max(vecnorm(run.cartesianResidual,inf,2)), ...
    'maxMotionIdentityResidual',max(abs(run.motionResidual)), ...
    'maxGap',max(abs(run.gap)),'maxNormalVelocity',max(abs(run.normalVelocity)), ...
    'maxNormalAcceleration',max(abs(run.normalAcceleration)));
values=cell2mat(struct2cell(checks));
assert(all(isfinite(run.state),'all') && all(values<1e-9),'NAC1:Dynamics','Dynamics check failed.');
% This residual is a force difference, measured in N.
checks.maxForceResidual=max(abs(run.forceResidual));
forceResidualTolerance=1e-9; % N
assert(checks.maxForceResidual<forceResidualTolerance,'NAC1:ForceResidual', ...
    'Force residual exceeds %g N.',forceResidualTolerance);
assert(all(run.force>0) && all(run.angle==controller.fixedAngle));
checks.operatingRegionSatisfied=run.operatingRegionSatisfied;
if plant.enforceOperatingRegion, assert(run.operatingRegionSatisfied); end
if strcmp(run.mode,'regulation')
    checks.regulationEquilibriumError=abs(run.error(end)-run.equilibrium.error);
    checks.regulationEquilibriumForceError=abs(run.force(end)-run.equilibrium.force);
    assert(checks.regulationEquilibriumError<1e-6);
    assert(checks.regulationEquilibriumForceError<1e-5 && abs(run.state(end,2))<1e-6);
end
if controller.fixedAngle==0
    % Zero mismatch must recover the independent analytical FIC 1 target.
    e0=[run.error(1);plant.T'*(run.referenceVelocity(1,:)'-run.velocity(1,:)')];
    A=[0,1;-controller.kp,-controller.kv];
    exact=zeros(numel(run.t),1);
    for i=1:numel(run.t), analyticErrorState=expm(A*run.t(i))*e0; exact(i)=analyticErrorState(1); end
    checks.zeroAngleAnalyticError=max(abs(run.error-exact));
    assert(checks.zeroAngleAnalyticError<1e-7 && max(abs(run.force-controller.fd))<1e-8);
end
% Verify the equilibrium prediction comes from the full joint plant as well.
evaluation=evaluate(0,[run.equilibrium.x;0],robot,plant,controller,task,'regulation',run.kf);
checks.equilibriumAcceleration=norm(evaluation.acceleration,inf);
assert(checks.equilibriumAcceleration<1e-9);
checks.passed=true;
end

function make_plots(results)
options=results.options; controller=results.controller;
runA=results.runs(1); runB=results.runs(2);
figureHandle=figure('Name','NAC 1: fixed inaccurate normal','Color','w','Visible',options.visible, ...
    'Position',[100,100,1200,800]);
hmfc_latex_figure(figureHandle);
tiledlayout(2,3,'TileSpacing','compact');
nexttile; plot(runA.t,[runA.state(:,1),runB.state(:,1)],'LineWidth',1.3); hold on;
yline(results.task.targetX,'k--'); grid on; xlabel('$t$ (s)'); ylabel('$x_t$ (m)');
legend('$k_f=0$','$k_f=2$','Desired'); title('Regulation with fixed geometry error');
nexttile; plot(runA.t,1000*[runA.error,runB.error],'LineWidth',1.3); hold on;
yline(runA.equilibrium.error*1000,'b:'); yline(runB.equilibrium.error*1000,'r:');
grid on; xlabel('$t$ (s)'); ylabel('Motion error (mm)'); title('Persistent motion bias');
legend('$k_f=0$','$k_f=2$','Predicted, $k_f=0$','Predicted, $k_f=2$');
nexttile; plot(runA.t,[runA.force,runB.force],'LineWidth',1.3); hold on;
yline(controller.fd,'k--'); grid on; xlabel('$t$ (s)'); ylabel('Force (N)');
legend('$k_f=0$','$k_f=2$','Desired'); title('Normal contact force');
nexttile; plot(runA.t,runA.angle*180/pi,'LineWidth',1.3); hold on; yline(0,'k--');
grid on; xlabel('$t$ (s)'); ylabel('Angle (${}^\circ$)'); title('Constant normal estimate');
legend('Fixed estimate','True normal');
nexttile; plot(runA.t,runA.tau,'LineWidth',1.2); grid on;
xlabel('$t$ (s)'); ylabel('Torque ($\mathrm{N\,m}$)'); legend('$\tau_1$','$\tau_2$'); title('Main-case torques');
nexttile; hold on;
origin=results.plant.p0;
lineX=origin(1)+[-0.04,0.16];
plot(lineX,origin(2)+[0,0],'k--','LineWidth',1.2);
plot(lineX,origin(2)+tan(controller.fixedAngle)*(lineX-origin(1)),'r:','LineWidth',1.2);
plot(runA.position(:,1),runA.position(:,2),'b','LineWidth',1.5);
plot(runA.reference(1,1),runA.reference(1,2),'ko','MarkerFaceColor','k');
plot(runA.position(end,1),runA.position(end,2),'bs','MarkerFaceColor','b');
axis equal; grid on; xlabel('$p_x$ (m)'); ylabel('$p_y$ (m)');
legend('True surface','Fixed estimated orientation','Actual path','Desired','Final');
title('Geometry and final position bias');
sgtitle(sprintf('NAC 1 | horizontal surface | fixed estimated normal $%+.1f^\\circ$',options.initialAngleDeg),'Interpreter','latex');
hmfc_latex_figure(figureHandle);
if options.saveResults
    exportgraphics(figureHandle,fullfile(options.outputDir,'nac1_summary.png'),'Resolution',180);
    savefig(figureHandle,fullfile(options.outputDir,'nac1_summary.fig'));
end
end

function write_report(results)
fid=fopen(fullfile(results.options.outputDir,'verification.txt'),'w');
assert(fid>=0); cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'NAC 1: standalone FIC 1 law with a fixed estimated normal.\n');
fprintf(fid,'True surface y=0.4 m; fixed angle %+.9g deg; kp=25, kv=10, fd=10 N.\n', ...
    results.options.initialAngleDeg);
fprintf(fid,'Same robot, initial state, reference, 6 s default duration and 0.002 s output as FIC/IAC.\n');
fprintf(fid,'No adaptive states, momentum correction, or angle update.\n');
fprintf(fid,'Original operating region enforced: %d\n',results.options.enforceOperatingRegion);
for i=1:numel(results.runs)
    run=results.runs(i);
    fprintf(fid,'\n%s, kf=%g\nFinal error %.12g m; final force %.12g N; min force %.12g N\n', ...
        run.mode,run.kf,run.error(end),run.force(end),min(run.force));
    if strcmp(run.mode,'regulation')
        fprintf(fid,'Predicted equilibrium error %.12g m; force %.12g N\n', ...
            run.equilibrium.error,run.equilibrium.force);
    end
    fields=fieldnames(run.checks);
    for j=1:numel(fields), fprintf(fid,'%s: %.12g\n',fields{j},run.checks.(fields{j})); end
    if ~run.operatingRegionSatisfied
        fprintf(fid,'Original certificate region first exited at approximately %.9g s.\n',run.firstRegionExitTime);
        fprintf(fid,'Continued using the same reachable nonsingular physical robot; no region certificate is claimed.\n');
    end
end
fprintf(fid,'\nAll enabled dynamics, force and equilibrium checks passed; region status is reported above.\n');
fprintf(fid,'Established frictionless rigid contact and ideal known model only.\n');
end
