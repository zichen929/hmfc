function results = fic_simulation(controllerId, makePlots, outputDir)
%FIC_SIMULATION Shared rigid-contact plant and verification for FIC 1 through 4.
%   Use fic1_simulation through fic4_simulation, or fic1234_simulation.
%   FIC 1 receives Cartesian references; FIC 2 receives intrinsic references
%   and converts them using the true geometry before Cartesian feedback.
%   FIC 3 directly uses scalar intrinsic errors to form ut, then up = T*ut.
%   FIC 4 projects Cartesian references into intrinsic coordinates before
%   applying the scalar feedback law shared with FIC 3.
%   Source: obsidian-vault/04-writing/tcst26/TCST26E3.md,
%   "FIC 1" through "FIC 4", and "2-DoF Planar Robot Arm Manipulator".
%   The main regulation scenario matches standalone iac1_simulation.m.
%   Values are duplicated deliberately to keep FIC and IAC independent.
%
%   The ODE state is z = [s; v], with p = p0 + T*s and pdot = T*v.
%   This eliminates the holonomic rigid-contact constraint without penalty
%   springs or constraint drift. IK reconstructs q; qdot = J\(T*v).
%   At EVERY RHS evaluation, the ORIGINAL JOINT dynamics and acceleration
%   constraint are solved simultaneously for qddot and the physical force f.
%   FIC 3 and FIC 4 reconstruct intrinsic errors from measurements.
%
%   In particular, neither sddot = T'*up nor f = fd is assigned to the plant.
%   Those are predictions checked against the constrained dynamics solution.

assert(isscalar(controllerId) && ismember(controllerId,1:4), ...
    'FIC:Controller', 'controllerId must be 1, 2, 3, or 4.');
if nargin < 2, makePlots = true; end
if nargin < 3
    outputDir = fullfile(fileparts(mfilename('fullpath')), 'results', sprintf('fic%d',controllerId));
end
if ~exist(outputDir, 'dir'), mkdir(outputDir); end
cfg = configuration();
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-11, 'MaxStep', 0.01);
time = linspace(0,cfg.duration,ceil(cfg.duration/cfg.outputStep)+1)';
modes = {'regulation', 'tracking'};
forceGains = [0, 2];
results = struct();
results.config = cfg;
results.controllerId = controllerId;
results.controller = sprintf('FIC %d',controllerId);
results.odeOptions = options;
results.source = 'obsidian-vault/04-writing/tcst26/TCST26E3.md';
results.cases = struct([]);

fprintf('FIC %d: exact-model 2R arm, maintained rigid frictionless unilateral contact\n',controllerId);
fprintf('kp = %.1f, kv = %.1f, fd = %.1f N; contact slope = %.1f deg\n', ...
    cfg.kp, cfg.kv, cfg.fd, cfg.surfaceAngle * 180 / pi);
fprintf('Error poles: %s\n', mat2str(roots([1, cfg.kv, cfg.kp]), 5));
for im = 1:numel(modes)
    for ik = 1:numel(forceGains)
        kf = forceGains(ik);
        [t, z] = ode45(@(t,z) plantODE(t, z, cfg, modes{im}, kf, controllerId), ...
            time, [0; cfg.initialVelocity], options);
        run = postprocess(t, z, cfg, modes{im}, kf, controllerId);
        checkCase(run, cfg);
        if isempty(results.cases)
            results.cases = run;
        else
            results.cases(end+1) = run;
        end
        fprintf('%-10s kf=%g: final |ep|=%.3e m, max |ef|=%.3e N, analytic error=%.3e m\n', ...
            run.mode, kf, run.metrics.finalPositionError, ...
            run.metrics.maxForceError, run.metrics.maxAnalyticError);
    end
end

% With the exact rigid-contact model, force feedback cannot improve an
% already exact algebraic force channel. Check both force gains agree.
for im = 1:numel(modes)
    a = results.cases(2*im-1);
    b = results.cases(2*im);
    gap = max(vecnorm(a.z - b.z, 2, 2));
    assert(gap < 1e-7, 'FIC:GainInvariance', ...
        'Ideal trajectories unexpectedly depend on kf (gap %.3e).', gap);
    results.gainComparison(im).mode = modes{im};
    results.gainComparison(im).maxStateDifference = gap;
end

summary = struct2table([results.cases.metrics]);
summary = addvars(summary, string({results.cases.mode})', ...
    [results.cases.kf]', 'Before', 1, 'NewVariableNames', {'mode','kf'});
results.summary = summary;
results.allChecksPassed = true;
writetable(summary, fullfile(outputDir, 'summary.csv'));
if makePlots
    for im = 1:numel(modes)
        plotCasePair(results.cases(2*im-1:2*im), cfg, outputDir);
    end
    plotDiagnostics(results.cases, cfg, outputDir);
end
save(fullfile(outputDir, sprintf('fic%d_results.mat',controllerId)), 'results');
fprintf('All numerical checks passed. Results: %s\n', outputDir);
end

function c = configuration()
c.l1 = 0.6; c.l2 = 0.5;
c.m1 = 2.0; c.m2 = 1.5;
c.lc1 = c.l1/2; c.lc2 = c.l2/2;
c.I1 = c.m1*c.l1^2/12; c.I2 = c.m2*c.l2^2/12;
c.g = 9.81;
c.kp = 25; c.kv = 10; c.fd = 10;
c.surfaceAngle = 0;
c.n = [-sin(c.surfaceAngle); cos(c.surfaceAngle)];
c.T = [0,1;-1,0]*c.n;
c.p0 = [0.6;0.4]; % FIRST contact point, identical to IAC 1.
c.initialVelocity = 0;
c.q2Min = 70*pi/180; c.q2Max = 115*pi/180;
c.minSinQ2 = min(sin([c.q2Min,c.q2Max]));
c.xMin = -0.04; c.xMax = 0.16;
c.q0 = inverseKinematics(c.p0,c); % Same positive-elbow branch as IAC 1.
c.targetX = 0.08;
c.setpoint = c.p0 + c.T*c.targetX;
c.duration = 6; c.outputStep = 0.002;
end

function dz = plantODE(t, z, c, mode, kf, controllerId)
a = evaluatePlant(t, z, c, mode, kf, controllerId);
dz = [z(2); c.T'*a.pdd];
end

function a = evaluatePlant(t, z, c, mode, kf, controllerId)
% Geometry is used by the PLANT coordinate chart, not as scalar feedback.
assert(z(1)>c.xMin && z(1)<c.xMax,'FIC:Region', ...
    'Tangential coordinate left the common FIC/IAC operating region.');
p = c.p0 + c.T*z(1);
q = inverseKinematics(p, c);
[~, ~, ~, J] = robotModel(q, zeros(2,1), c);
dq = J\(c.T*z(2));
[M, C, G, J, Jdotdq] = robotModel(q, dq, c);
Lambda = J'\(M/J);
eta = J'\(C*dq + G) - Lambda*Jdotdq;
switch controllerId
    case 1
        % FIC 1 directly receives three Cartesian reference vectors.
        [pd, dpd, ddpd] = cartesianReference(t, c, mode);
    case {2,3}
        % FIC 2 maps these for feedback; FIC 3 uses the mapping for diagnostics.
        [xtd, dxtd, ddxtd] = intrinsicReference(t, c, mode);
        pd = c.p0 + c.T*xtd;
        dpd = c.T*dxtd;
        ddpd = c.T*ddxtd;
    case 4
        % FIC 4 receives Cartesian references, then converts to intrinsic ones.
        [pdInput, dpdInput, ddpdInput] = cartesianReference(t, c, mode);
        xtd = c.T'*(pdInput-c.p0);
        dxtd = c.T'*dpdInput;
        ddxtd = c.T'*ddpdInput;
        % Effective on-surface reference used only for diagnostics/verification.
        % If the input has a normal offset, this is its contact-line projection.
        pd = c.p0 + c.T*xtd;
        dpd = c.T*dxtd;
        ddpd = c.T*ddxtd;
end
pm = forwardKinematics(q, c);
dpm = J*dq;
ep = pd - pm;
ev = dpd - dpm;

if controllerId >= 3
    % The scalar controller has no access to pd, dpd, ddpd, ep, or ev.
    [ut,xt,dxt,et,det] = intrinsicControl(pm,dpm,xtd,dxtd,ddxtd,c);
    up = c.T*ut;
else
    % FIC 1 / FIC 2 Cartesian feedback.
    up = ddpd + c.kv*ev + c.kp*ep;
end
% uf = (1+kf)*fd - kf*f, so F = F0 + n*kf*f.
F0 = eta + Lambda*up - c.n*(1+kf)*c.fd;
tau0 = J'*F0;
Jn = c.n'*J;
% M*qdd + C*dq + G = tau0 + (1+kf)*J'*n*f,
% n'*(J*qdd + Jdot*dq) = 0.
% Solve the instantaneous sensor/controller/contact algebraic loop exactly.
K = [M, -(1+kf)*Jn'; Jn, 0];
sol = K\[tau0 - C*dq - G; -c.n'*Jdotdq];
qdd = sol(1:2);
f = sol(3); % Reaction force is obtained from the constrained plant solve.
assert(f>0,'FIC:ContactLoss','The calculated unilateral reaction is not positive.');
uf = c.fd + kf*(c.fd - f);
F = eta + Lambda*up - c.n*uf;
tau = J'*F;
pdd = J*qdd + Jdotdq;

a.q = q; a.dq = dq; a.qdd = qdd;
a.p = pm; a.dp = dpm; a.pdd = pdd;
a.pd = pd; a.dpd = dpd; a.ep = ep; a.ev = ev;
a.f = f; a.uf = uf; a.up = up; a.tau = tau;
a.phi = c.n'*(pm - c.p0);
a.normalVelocity = c.n'*dpm;
a.normalAcceleration = c.n'*pdd;
a.errorResidual = ddpd - pdd + c.kv*ev + c.kp*ep;
a.dynamicResidual = M*qdd + C*dq + G - tau - J'*c.n*f;
% Independently reconstruct reaction from the Cartesian constraint formula.
Li_n = Lambda\c.n;
a.forceReconstructionError = f + (Li_n'*(F-eta))/(c.n'*Li_n);
a.commandResidual = pdd - up;
a.V = 0.5*(ev'*ev + c.kp*(ep'*ep));
a.Vdot = ev'*(ddpd - pdd) + c.kp*ep'*ev;
a.lyapunovResidual = a.Vdot + c.kv*(ev'*ev);
a.minLambda = min(eig(Lambda));
a.maxLambda = max(eig(Lambda));
a.condJ = cond(J);
if controllerId >= 3
    a.xt = xt; a.dxt = dxt; a.xtd = xtd; a.dxtd = dxtd; a.ddxtd = ddxtd;
    a.et = et; a.det = det; a.ut = ut;
    a.intrinsicErrorResidual = ddxtd - c.T'*pdd + c.kv*det + c.kp*et;
    a.errorMappingResidual = norm(ep-c.T*et) + norm(ev-c.T*det);
end
if controllerId == 4
    a.pdInput = pdInput; a.dpdInput = dpdInput; a.ddpdInput = ddpdInput;
    a.referenceNormalOffset = c.n'*(pdInput-c.p0);
    a.referenceProjectionResidual = norm(c.T'*(pdInput-pd)) ...
        + norm(c.T'*(dpdInput-dpd)) + norm(c.T'*(ddpdInput-ddpd));
end
end

function [ut,xt,dxt,et,det] = intrinsicControl(pm,dpm,xtd,dxtd,ddxtd,c)
% FIC 3 / FIC 4: intrinsic errors use measured Cartesian position/velocity.
% This function deliberately receives no Cartesian desired position or error.
xt = c.T'*(pm-c.p0);
dxt = c.T'*dpm;
et = xtd-xt;
det = dxtd-dxt;
ut = ddxtd + c.kv*det + c.kp*et;
end

function [pd, dpd, ddpd] = cartesianReference(t, c, mode)
% FIC 1 and FIC 4 receive these Cartesian vectors directly. No scalar target
% or scalar tracking error is evaluated inside the controller.
switch mode
    case 'regulation'
        pd = c.setpoint;
        dpd = zeros(2,1); ddpd = zeros(2,1);
    case 'tracking'
        pd = c.p0 + c.T*(0.08 + 0.045*sin(0.8*t) + 0.02*cos(1.3*t));
        dpd = c.T*(0.045*0.8*cos(0.8*t) - 0.02*1.3*sin(1.3*t));
        ddpd = c.T*(-0.045*0.8^2*sin(0.8*t) - 0.02*1.3^2*cos(1.3*t));
    otherwise
        error('FIC:Reference', 'Unknown reference mode: %s', mode);
end
end

function [xtd, dxtd, ddxtd] = intrinsicReference(t, c, mode)
% Input to FIC 2 and FIC 3: scalar desired position and its derivatives.
% Kept separate from the FIC 1 Cartesian generator to exercise conversion.
switch mode
    case 'regulation'
        xtd = c.targetX; dxtd = 0; ddxtd = 0;
    case 'tracking'
        xtd = 0.08 + 0.045*sin(0.8*t) + 0.02*cos(1.3*t);
        dxtd = 0.045*0.8*cos(0.8*t) - 0.02*1.3*sin(1.3*t);
        ddxtd = -0.045*0.8^2*sin(0.8*t) - 0.02*1.3^2*cos(1.3*t);
    otherwise
        error('FIC:Reference', 'Unknown reference mode: %s', mode);
end
end

function p = forwardKinematics(q, c)
p = [c.l1*cos(q(1)) + c.l2*cos(sum(q)); ...
     c.l1*sin(q(1)) + c.l2*sin(sum(q))];
end

function q = inverseKinematics(p, c)
cosq2 = (p'*p - c.l1^2 - c.l2^2)/(2*c.l1*c.l2);
assert(abs(cosq2) < 1, 'FIC:Workspace', ...
    'Contact trajectory left the reachable nonsingular workspace.');
q2 = acos(cosq2); % Positive-elbow branch, continuous with q0.
assert(abs(sin(q2)) >= c.minSinQ2, 'FIC:Singularity', ...
    'Trajectory left the chosen region |sin(q2)| >= %.2f.', c.minSinQ2);
assert(q2>c.q2Min && q2<c.q2Max,'FIC:Region', ...
    'Elbow angle left the common [70,115] degree operating region.');
q1 = atan2(p(2),p(1)) - atan2(c.l2*sin(q2), c.l1+c.l2*cos(q2));
q = [q1; q2];
end

function [M, C, G, J, Jdotdq] = robotModel(q, dq, c)
% Appendix equations, including the configuration-dependent gravity term.
c2 = cos(q(2)); s2 = sin(q(2));
c1 = cos(q(1)); s1 = sin(q(1));
c12 = cos(sum(q)); s12 = sin(sum(q));
M11 = c.I1+c.I2+c.m1*c.lc1^2 ...
    + c.m2*(c.l1^2+c.lc2^2+2*c.l1*c.lc2*c2);
M12 = c.I2+c.m2*(c.lc2^2+c.l1*c.lc2*c2);
M22 = c.I2+c.m2*c.lc2^2;
M = [M11,M12;M12,M22];
b = c.m2*c.l1*c.lc2*s2;
C = [-b*dq(2), -b*sum(dq); b*dq(1), 0];
G = [(c.m1*c.lc1+c.m2*c.l1)*c.g*c1 + c.m2*c.lc2*c.g*c12; ...
    c.m2*c.lc2*c.g*c12];
J = [-c.l1*s1-c.l2*s12, -c.l2*s12; c.l1*c1+c.l2*c12, c.l2*c12];
Jdotdq = -[c.l1*c1*dq(1)^2+c.l2*c12*sum(dq)^2; ...
    c.l1*s1*dq(1)^2+c.l2*s12*sum(dq)^2];
end

function r = postprocess(t, z, c, mode, kf, controllerId)
r.controller = sprintf('FIC %d',controllerId);
r.mode = mode; r.kf = kf; r.t = t; r.z = z;
for i = 1:numel(t)
    a = evaluatePlant(t(i), z(i,:)', c, mode, kf, controllerId);
    if i == 1
        names = fieldnames(a);
        for j = 1:numel(names)
            r.(names{j}) = zeros(numel(t), numel(a.(names{j})));
        end
    end
    for j = 1:numel(names)
        r.(names{j})(i,:) = a.(names{j})(:)';
    end
end
% Independent analytical benchmark, e'' + kv*e' + kp*e = 0.
A = [0,1;-c.kp,-c.kv];
e0 = [r.ep(1,:)*c.T; r.ev(1,:)*c.T];
exact = zeros(numel(t),2);
for i = 1:numel(t)
    exact(i,:) = (expm(A*(t(i)-t(1)))*e0)';
end
r.exactEp = exact(:,1)*c.T';
r.exactEv = exact(:,2)*c.T';
m.finalPositionError = norm(r.ep(end,:));
m.finalVelocityError = norm(r.ev(end,:));
m.tailRmsPositionError = sqrt(mean(sum(r.ep(t>=t(end)-2,:).^2,2)));
m.maxForceError = max(abs(c.fd-r.f));
m.maxAnalyticError = max(vecnorm(r.ep-r.exactEp,2,2));
m.maxAnalyticVelocityError = max(vecnorm(r.ev-r.exactEv,2,2));
m.maxContactPositionError = max(abs(r.phi));
m.maxNormalVelocity = max(abs(r.normalVelocity));
m.maxNormalAcceleration = max(abs(r.normalAcceleration));
m.maxErrorDynamicsResidual = max(vecnorm(r.errorResidual,2,2));
m.maxJointDynamicsResidual = max(vecnorm(r.dynamicResidual,2,2));
m.maxForceReconstructionError = max(abs(r.forceReconstructionError));
m.maxCommandResidual = max(vecnorm(r.commandResidual,2,2));
m.maxLyapunovResidual = max(abs(r.lyapunovResidual));
m.maxPositiveVdot = max([0;r.Vdot]);
m.minSinQ2 = min(abs(sin(r.q(:,2))));
m.maxJacobianCondition = max(r.condJ);
m.minCartesianInertia = min(r.minLambda);
m.maxCartesianInertia = max(r.maxLambda);
m.minForce = min(r.f);
m.peakTorque = max(abs(r.tau),[],'all');
r.metrics = m;
end

function checkCase(r, c)
m = r.metrics;
assert(all(isfinite(r.z(:))) && all(isfinite(r.tau(:))), ...
    'FIC:Nonfinite', 'Nonfinite simulation output.');
assert(m.finalPositionError < 1e-7 && m.finalVelocityError < 1e-7, ...
    'FIC:Convergence', 'Motion error did not converge.');
assert(m.maxAnalyticError < 1e-7 && m.maxAnalyticVelocityError < 1e-7, ...
    'FIC:Analytic', 'ODE trajectory disagrees with analytical error solution.');
residuals = [m.maxForceError, m.maxContactPositionError, ...
    m.maxNormalVelocity, m.maxNormalAcceleration, m.maxErrorDynamicsResidual, ...
    m.maxJointDynamicsResidual, m.maxForceReconstructionError, ...
    m.maxCommandResidual, m.maxLyapunovResidual, m.maxPositiveVdot];
assert(all(residuals < 1e-8), 'FIC:Residual', ...
    'A force, contact, dynamics, or Lyapunov residual exceeds tolerance.');
assert(m.minSinQ2 >= c.minSinQ2 && m.minCartesianInertia > 0, ...
    'FIC:Region', 'Nonsingularity or positive inertia condition violated.');
assert(m.minForce>0,'FIC:ContactLoss','Unilateral contact was not maintained.');
if isfield(r,'et')
    assert(max(abs(r.intrinsicErrorResidual)) < 1e-8 ...
        && max(r.errorMappingResidual) < 1e-8, ...
        'FIC:IntrinsicDynamics','Intrinsic dynamics or error mapping failed.');
end
if isfield(r,'referenceProjectionResidual')
    assert(max(r.referenceProjectionResidual) < 1e-8, ...
        'FIC:ReferenceProjection','FIC 4 reference projection failed.');
end
end

function plotCasePair(pair, c, outputDir)
a = pair(1); b = pair(2); t = a.t;
fig = figure('Name', [a.controller,' - ',a.mode], 'Color','w', ...
    'Position',[80,80,1250,900]);
if isprop(fig,'Theme'), set(fig,'Theme','light'); end
tl = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
title(tl,sprintf('%s | %s | 2R arm | k_p=%g, k_v=%g, f_d=%g N', ...
    a.controller,a.mode,c.kp,c.kv,c.fd));
nexttile;
linePoints = c.p0 + c.T*[-0.07,0.20];
plot(linePoints(1,:),linePoints(2,:),'k:','LineWidth',1); hold on;
plot(a.pd(:,1),a.pd(:,2),'k--','LineWidth',2);
plot(a.p(:,1),a.p(:,2),'b-','LineWidth',1.3);
plot(a.p(1,1),a.p(1,2),'go','MarkerFaceColor','g');
axis equal; grid on; xlabel('p_1 (m)'); ylabel('p_2 (m)');
title('Cartesian motion on the horizontal contact line');
legend('Contact line','Reference','Actual','Initial contact','Location','best');
nexttile;
plot(t,(a.pd-c.p0')*c.T,'k--',t,a.z(:,1),'b-',t,b.z(:,1),'r:','LineWidth',1.4);
grid on; xlabel('t (s)'); ylabel('x_t (m)'); title('Tangential position (diagnostic)');
legend('Reference','k_f = 0','k_f = 2','Location','best');
nexttile;
semilogy(t,max(vecnorm(a.ep,2,2),1e-16),'b-', ...
    t,max(vecnorm(b.ep,2,2),1e-16),'r:', ...
    t,max(vecnorm(a.exactEp,2,2),1e-16),'k--','LineWidth',1.3);
grid on; xlabel('t (s)'); ylabel('||e_p|| (m)'); title('Exponential error convergence');
legend('k_f = 0','k_f = 2','Analytical solution','Location','best');
nexttile;
plot(t,a.f,'b-',t,b.f,'r:','LineWidth',1.5); hold on;
yline(c.fd,'k--'); ylim([c.fd-1,c.fd+1]);
grid on; xlabel('t (s)'); ylabel('f (N)'); title('Algebraic normal force: no force transient');
legend('k_f = 0','k_f = 2','f_d','Location','best');
nexttile;
semilogy(t,max(a.V,1e-30),'b-',t,max(b.V,1e-30),'r:','LineWidth',1.3);
grid on; xlabel('t (s)'); ylabel('V'); title('V = (||de_p/dt||^2 + k_p ||e_p||^2)/2');
legend('k_f = 0','k_f = 2','Location','best');
nexttile;
plot(t,a.tau(:,1),'b-',t,a.tau(:,2),'r-', ...
    t,b.tau(:,1),'k:',t,b.tau(:,2),'m:','LineWidth',1.2);
grid on; xlabel('t (s)'); ylabel('Joint torque (N m)'); title('Applied actuator torques');
legend('\tau_1, k_f=0','\tau_2, k_f=0', ...
    '\tau_1, k_f=2','\tau_2, k_f=2','Location','best');
exportgraphics(fig,fullfile(outputDir,[a.mode,'.png']),'Resolution',160);
savefig(fig,fullfile(outputDir,[a.mode,'.fig']));
end

function plotDiagnostics(runs, c, outputDir)
fig = figure('Name',[runs(1).controller,' - residuals'],'Color','w','Position',[100,100,1250,900]);
if isprop(fig,'Theme'), set(fig,'Theme','light'); end
tl = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
title(tl,[runs(1).controller,' | physical dynamics, contact, and analytical checks']);
labels = cell(1,numel(runs));
for k = 1:numel(runs)
    r = runs(k); labels{k} = sprintf('%s, k_f=%g',r.mode,r.kf);
    series = {abs(r.f-c.fd), abs(r.phi), vecnorm(r.errorResidual,2,2), ...
        vecnorm(r.dynamicResidual,2,2), vecnorm(r.ep-r.exactEp,2,2), abs(sin(r.q(:,2)))};
    for j = 1:6
        nexttile(j); hold on;
        if j < 6
            semilogy(r.t,max(series{j},1e-16),'LineWidth',1);
            set(gca,'YScale','log');
        else
            plot(r.t,series{j},'LineWidth',1.2);
        end
        grid on; xlabel('t (s)');
    end
end
titles = {'Normal force error |f-f_d| (N)', 'Contact residual |n^T(p-p_0)| (m)', ...
    'Error ODE residual norm (m/s^2)', 'Joint dynamics residual norm (N m)', ...
    'ODE vs analytical position error (m)', 'Nonsingularity margin |sin(q_2)|'};
for j = 1:6
    nexttile(j); title(titles{j});
end
nexttile(6); yline(c.minSinQ2,'k--','Operating-region boundary');
legend(labels,'Location','best');
exportgraphics(fig,fullfile(outputDir,'diagnostics.png'),'Resolution',160);
savefig(fig,fullfile(outputDir,'diagnostics.fig'));
end
