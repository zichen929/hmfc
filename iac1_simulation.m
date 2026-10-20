function results = iac1_simulation(options)
%IAC1_SIMULATION Standalone ODE verification of TCST26, IAC 1.
%   results = iac1_simulation();
%   results = iac1_simulation(struct('makePlots',false,'saveResults',false));
% No FIC functions, shared settings, or toolboxes are used.
% Source: obsidian-vault/04-writing/tcst26/TCST26E3.md, heading TCST26.
% The ODE integrates [x_t; dx_t; hat_theta], NOT the derived z dynamics.
% The maintained rigid-contact branch is parameterized exactly by x_t.

if nargin == 0, options = struct(); end
defaults = struct('makePlots',true,'saveResults',true,'visible','on', ...
    'outputDir',fullfile(fileparts(mfilename('fullpath')),'results','iac1'), ...
    'initialAngleDeg',5,'initialVelocity',0,'duration',6);
assert(isstruct(options) && isscalar(options),'options must be a scalar struct.');
names = fieldnames(options);
for k = 1:numel(names)
    assert(isfield(defaults,names{k}),'Unknown option: %s',names{k});
    defaults.(names{k}) = options.(names{k});
end
options = defaults;
validateattributes(options.initialAngleDeg,{'numeric'},{'real','finite','scalar'});
validateattributes(options.initialVelocity,{'numeric'},{'real','finite','scalar'});
validateattributes(options.duration,{'numeric'},{'real','finite','scalar','>=',6});

% Exactly known 2R robot in the vertical plane; SI units throughout.
r = struct('l1',0.6,'l2',0.5,'m1',2,'m2',1.5, ...
    'lc1',0.3,'lc2',0.25,'I1',2*0.6^2/12,'I2',1.5*0.5^2/12,'g',9.81);
plant = struct('n',[0;1],'T',[1;0],'p0',[0.6;0.4]);
% Controller data deliberately contain neither the true n nor true theta.
c = struct('n0',[0;1],'thetaC',0,'delta',6*pi/180, ...
    'kp',25,'kv',10,'fd',10,'gamma',0.4);
c.Tc = [0,1;-1,0]*rotation(c.thetaC)*c.n0;

% Uniform analytic inertia bounds on q2 in [70,115] deg, any q1.
% A=I1+m1*lc1^2+m2*l1^2, B=m2*l1*lc2, D=I2+m2*lc2^2.
% trace(Lambda)=(A/l1^2+D/l2^2-2*B*cos(q2)^2/(l1*l2))/sin(q2)^2.
% det(Lambda)=(A*D-B^2*cos(q2)^2)/(l1^2*l2^2*sin(q2)^2).
% lambda_max <= traceUpper; lambda_min >= detLower/traceUpper.
domain = struct('q2Min',70*pi/180,'q2Max',115*pi/180, ...
    'xMin',-0.04,'xMax',0.16);
A = r.I1+r.m1*r.lc1^2+r.m2*r.l1^2;
B = r.m2*r.l1*r.lc2;
D = r.I2+r.m2*r.lc2^2;
cmax = max(abs(cos([domain.q2Min,domain.q2Max])));
smin2 = 1-cmax^2;
massMax = (A/r.l1^2+D/r.l2^2)/smin2;
detLower = (A*D-B^2*cmax^2)/(r.l1^2*r.l2^2);
massMin = detLower/massMax;
chi = (massMax-massMin)/(2*sqrt(massMax*massMin));
rhoMin = cos(2*c.delta)-chi*sin(2*c.delta);
assert(massMin>0 && rhoMin>0,'Angular sector cannot certify positive contact.');
assert(abs(options.initialAngleDeg*pi/180)<=c.delta, ...
    'Initial effective angle must be inside the +/-6 degree sector.');
certificate = struct('massMin',massMin,'massMax',massMax,'chi',chi, ...
    'rhoMin',rhoMin,'forceMin',rhoMin*c.fd, ...
    'alphaZ',c.gamma*rhoMin*c.fd*cos(c.delta),'domain',domain);

% The first case is the requested scenario. Opposite sign, invariant
% manifold (nonzero momentum), and regulation from rest are cross-checks.
cases = struct('name',{'positive_tilt','negative_tilt','on_manifold','at_rest'}, ...
    'angleDeg',{options.initialAngleDeg,-abs(options.initialAngleDeg),0,options.initialAngleDeg}, ...
    'velocity',{options.initialVelocity,options.initialVelocity,0.02,0}, ...
    'targetX',{0.08,0.08,0.08,0});
results = struct('source','TCST26E3.md / TCST26 / IAC 1', ...
    'options',options,'robot',r,'plant',plant,'controller',c, ...
    'certificate',certificate,'runs',[]);
odeOptions = odeset('RelTol',1e-9,'AbsTol',[1e-11,1e-11,1e-12], ...
    'MaxStep',0.01,'Events',@(t,y) domain_events(t,y,r,plant,c,domain));
tRequested = linspace(0,options.duration,ceil(options.duration/0.002)+1);
fprintf('\nIAC 1: horizontal rigid contact, independent 2R model\n');
fprintf('Certified f >= %.6f N; alpha_z = %.6f 1/s\n', ...
    certificate.forceMin,certificate.alphaZ);
for k = 1:numel(cases)
    spec = cases(k);
    reference = plant.p0+[spec.targetX;0];
    [~,~,initialModel] = measured_state(0,spec.velocity,r,plant);
    % hat_theta(0)=bar_theta(0)+gamma*Tc'*Lambda(0)*dp(0).
    hat0 = spec.angleDeg*pi/180+c.gamma*c.Tc'*initialModel.Lambda* ...
        (plant.T*spec.velocity);
    y0 = [0;spec.velocity;hat0];
    rhs = @(t,y) closed_loop(t,y,r,plant,c,reference);
    [t,y,te,~,ie] = ode45(rhs,tRequested,y0,odeOptions);
    assert(isempty(te),'Contact/sector/robot domain event %d at t=%.6g.', ...
        first_or_zero(ie),first_or_zero(te));
    assert(abs(t(end)-options.duration)<1e-10,'ODE ended prematurely.');
    run = postprocess(t,y,r,plant,c,reference,certificate);
    run.name = spec.name;
    run.spec = spec;
    run.initialState = y0;
    run.checks = verify_run(run,r,plant,c,certificate);
    results.runs = [results.runs;run];
    fprintf('%-14s z: %+.3f -> %+.3e deg, e: %+.3e m, f: %.9f N, min f: %.6f N\n', ...
        run.name,run.z(1)*180/pi,run.z(end)*180/pi, ...
        run.error(end),run.force(end),min(run.force));
end

if options.saveResults
    if ~exist(options.outputDir,'dir'), mkdir(options.outputDir); end
    for k = 1:numel(results.runs)
        run = results.runs(k);
        signals = table(run.t,run.y(:,1),run.y(:,2),run.y(:,3),run.beta, ...
            run.barTheta,run.z,run.error,run.force,run.tau(:,1),run.tau(:,2), ...
            'VariableNames',{'time_s','x_t_m','dx_t_mps','hat_theta_rad', ...
            'beta_rad','bar_theta_rad','z_rad','motion_error_m','force_N', ...
            'tau1_Nm','tau2_Nm'});
        writetable(signals,fullfile(options.outputDir,[run.name,'.csv']));
    end
    save(fullfile(options.outputDir,'iac1_results.mat'),'results');
    write_summary(results);
end
if options.makePlots, make_plots(results); end
fprintf('All %d cases passed the numerical verification checks.\n\n',numel(cases));
end

function dy = closed_loop(~,y,r,plant,c,reference)
[p,velocity,model] = measured_state(y(1),y(2),r,plant);
cmd = controller_command(p,velocity,y(3),model,c,reference);
[acceleration,force] = contact_plant(cmd.F,model,plant.n);
hatDot = estimator_update(cmd,force,velocity,model,c);
dy = [plant.T'*velocity;plant.T'*acceleration;hatDot];
end

function cmd = controller_command(p,velocity,hatTheta,model,c,reference)
% Only measured p, velocity, internal state, known model and reference.
cmd.beta = -c.gamma*c.Tc'*model.Lambda*velocity;
cmd.barTheta = hatTheta+cmd.beta;
cmd.nbar = rotation(cmd.barTheta)*c.n0;
cmd.up = c.kp*(reference-p)-c.kv*velocity;
cmd.F = model.eta+model.Lambda*cmd.up-cmd.nbar*c.fd;
cmd.tau = model.J'*cmd.F;
end

function hatDot = estimator_update(cmd,force,velocity,model,c)
% Scalar force sensor only; no true normal, z, acceleration or force derivative.
gp = cmd.F-model.eta+model.LambdaDot*velocity;
hatDot = c.gamma*c.Tc'*(gp+cmd.nbar*force);
end

function [acceleration,force] = contact_plant(F,model,n)
% KKT system for Lambda*a+eta=F+n*f, n'*a=0.
% Do not clip a negative reaction: that would conceal invalid contact.
solution = [model.Lambda,-n;n',0]\[F-model.eta;0];
acceleration = solution(1:2);
force = solution(3);
end

function [p,velocity,model] = measured_state(x,v,r,plant)
% Minimal-coordinate rigid plant: no normal drift and no Baumgarte gains.
p = plant.p0+plant.T*x;
velocity = plant.T*v;
q = inverse_kinematics(p,r);
J = jacobian(q,r);
dq = J\velocity;
model = robot_model(q,dq,r);
end

function q = inverse_kinematics(p,r)
cos2 = (p'*p-r.l1^2-r.l2^2)/(2*r.l1*r.l2);
assert(abs(cos2)<1,'Outside the nonsingular reachable workspace.');
q2 = acos(cos2); % fixed elbow branch, 0<q2<pi
q1 = atan2(p(2),p(1))-atan2(r.l2*sin(q2),r.l1+r.l2*cos(q2));
q = [q1;q2];
end

function J = jacobian(q,r)
J = [-r.l1*sin(q(1))-r.l2*sin(sum(q)),-r.l2*sin(sum(q)); ...
      r.l1*cos(q(1))+r.l2*cos(sum(q)), r.l2*cos(sum(q))];
end

function model = robot_model(q,dq,r)
J = jacobian(q,r);
Jdot = [-r.l1*cos(q(1))*dq(1)-r.l2*cos(sum(q))*sum(dq), ...
        -r.l2*cos(sum(q))*sum(dq); ...
        -r.l1*sin(q(1))*dq(1)-r.l2*sin(sum(q))*sum(dq), ...
        -r.l2*sin(sum(q))*sum(dq)];
A = r.I1+r.m1*r.lc1^2+r.m2*r.l1^2;
B = r.m2*r.l1*r.lc2;
D = r.I2+r.m2*r.lc2^2;
M = [A+D+2*B*cos(q(2)),D+B*cos(q(2));D+B*cos(q(2)),D];
Mdot = -B*sin(q(2))*dq(2)*[2,1;1,0];
b = B*sin(q(2));
C = [-b*dq(2),-b*sum(dq);b*dq(1),0];
G = [(r.m1*r.lc1+r.m2*r.l1)*r.g*cos(q(1))+ ...
    r.m2*r.lc2*r.g*cos(sum(q));r.m2*r.lc2*r.g*cos(sum(q))];
K = J\eye(2);
Kdot = -K*Jdot*K;
Lambda = K'*M*K;
LambdaDot = Kdot'*M*K+K'*Mdot*K+K'*M*Kdot;
eta = J'\(C*dq+G)-Lambda*Jdot*dq;
model = struct('q',q,'dq',dq,'J',J,'Jdot',Jdot,'M',M,'C',C,'G',G, ...
    'Lambda',Lambda,'LambdaDot',LambdaDot,'eta',eta);
end

function [value,isterminal,direction] = domain_events(~,y,r,plant,c,domain)
[p,velocity,model] = measured_state(y(1),y(2),r,plant);
cmd = controller_command(p,velocity,y(3),model,c,p);
[~,force] = contact_plant(cmd.F,model,plant.n);
% up is tangent for every admissible reference, so the reaction is
% independent of which such reference is supplied to this event check.
value = [force;domain.xMax-y(1);y(1)-domain.xMin; ...
    model.q(2)-domain.q2Min;domain.q2Max-model.q(2); ...
    c.delta-abs(cmd.barTheta-c.thetaC)];
isterminal = ones(size(value));
direction = -ones(size(value));
end

function run = postprocess(t,y,r,plant,c,reference,cert)
N = numel(t);
run = struct('t',t,'y',y,'reference',reference);
fields = {'beta','barTheta','z','error','force','rho','rhoFormula', ...
    'zDot','zDotTheory','momentumResidual','plantResidual','jointResidual', ...
    'motionResidual','q2','minEig','maxEig','detJ','gap','normalVelocity'};
for k = 1:numel(fields), run.(fields{k}) = zeros(N,1); end
run.p = zeros(N,2);
run.tau = zeros(N,2);
trueTheta = atan2(-plant.n(1),plant.n(2));
for i = 1:N
    [p,velocity,m] = measured_state(y(i,1),y(i,2),r,plant);
    cmd = controller_command(p,velocity,y(i,3),m,c,reference);
    [a,f] = contact_plant(cmd.F,m,plant.n);
    hd = estimator_update(cmd,f,velocity,m,c);
    momentumDot = m.LambdaDot*velocity+m.Lambda*a;
    gp = cmd.F-m.eta+m.LambdaDot*velocity;
    z = cmd.barTheta-trueTheta;
    lt = plant.T'*m.Lambda*plant.T;
    coupling = (plant.T'*m.Lambda*plant.n)/lt;
    ddq = m.J\(a-m.Jdot*m.dq);
    run.p(i,:) = p';
    run.tau(i,:) = cmd.tau';
    run.beta(i) = cmd.beta;
    run.barTheta(i) = cmd.barTheta;
    run.z(i) = z;
    run.error(i) = plant.T'*(reference-p);
    run.force(i) = f;
    run.rho(i) = f/c.fd;
    run.rhoFormula(i) = cos(z)+coupling*sin(z);
    run.zDot(i) = hd-c.gamma*c.Tc'*momentumDot;
    run.zDotTheory(i) = -c.gamma*f*(sin(cmd.barTheta-c.thetaC)- ...
        sin(trueTheta-c.thetaC));
    run.momentumResidual(i) = norm(momentumDot-gp-plant.n*f,inf);
    run.plantResidual(i) = norm(m.Lambda*a+m.eta-cmd.F-plant.n*f,inf);
    run.jointResidual(i) = norm(m.M*ddq+m.C*m.dq+m.G-cmd.tau-m.J'*plant.n*f,inf);
    run.motionResidual(i) = plant.T'*(a-cmd.up)-c.fd*sin(z)/lt;
    eigenvalues = eig(m.Lambda);
    run.minEig(i) = min(eigenvalues);
    run.maxEig(i) = max(eigenvalues);
    run.q2(i) = m.q(2);
    run.detJ(i) = det(m.J);
    run.gap(i) = plant.n'*(p-plant.p0);
    run.normalVelocity(i) = plant.n'*velocity;
end
run.angleEnvelope = abs(run.z(1))*exp(-cert.alphaZ*t);
end

function checks = verify_run(run,r,plant,c,cert)
checks = struct();
checks.maxPlantResidual = max(run.plantResidual);
checks.maxJointResidual = max(run.jointResidual);
checks.maxMomentumResidual = max(run.momentumResidual);
checks.maxAngleIdentityResidual = max(abs(run.zDot-run.zDotTheory));
checks.maxForceIdentityResidual = max(abs(run.rho-run.rhoFormula));
checks.maxMotionIdentityResidual = max(abs(run.motionResidual));
checks.maxGap = max(abs(run.gap));
checks.maxNormalVelocity = max(abs(run.normalVelocity));
checks.minForce = min(run.force);
checks.maxEnvelopeExcess = max(abs(run.z)-run.angleEnvelope);
% Independent finite difference of Lambda along a nonzero-velocity model.
[~,~,m] = measured_state(0,0.07,r,plant);
epsilon = 1e-5;
mp = robot_model(m.q+epsilon*m.dq,m.dq,r);
mm = robot_model(m.q-epsilon*m.dq,m.dq,r);
checks.lambdaDotFiniteDifference = norm((mp.Lambda-mm.Lambda)/(2*epsilon)-m.LambdaDot,inf);
% Central difference of the INTEGRATED effective angle, excluding endpoints.
dt = run.t(2)-run.t(1);
numericZdot = (run.z(3:end)-run.z(1:end-2))/(2*dt);
checks.integratedAngleDerivativeResidual = max(abs(numericZdot-run.zDotTheory(2:end-1)));
assert(checks.maxPlantResidual<1e-9 && checks.maxJointResidual<1e-9);
assert(checks.maxMomentumResidual<1e-9 && checks.maxAngleIdentityResidual<1e-9);
assert(checks.maxForceIdentityResidual<1e-10 && checks.maxMotionIdentityResidual<1e-9);
assert(checks.lambdaDotFiniteDifference<1e-7);
assert(checks.integratedAngleDerivativeResidual<1e-4);
assert(checks.maxGap<1e-12 && checks.maxNormalVelocity<1e-12);
assert(checks.minForce>=cert.forceMin-1e-8);
assert(checks.maxEnvelopeExcess<2e-9);
assert(all(abs(run.barTheta-c.thetaC)<=c.delta+1e-9));
assert(all(run.minEig>=cert.massMin) && all(run.maxEig<=cert.massMax));
assert(all(run.q2>cert.domain.q2Min & run.q2<cert.domain.q2Max));
assert(all(run.y(:,1)>cert.domain.xMin & run.y(:,1)<cert.domain.xMax));
assert(abs(run.z(end))<1e-6,'Effective angle did not converge.');
assert(abs(run.error(end))<1e-6,'Motion error did not converge.');
assert(abs(run.y(end,2))<1e-6,'Tangential velocity did not converge.');
assert(abs(run.force(end)-c.fd)<1e-5,'Contact force did not converge.');
if strcmp(run.name,'on_manifold')
    assert(max(abs(run.z))<2e-9,'I&I manifold was not invariant.');
    assert(abs(run.beta(1))>1e-4,'Manifold check requires nonzero correction.');
end
checks.passed = true;
end

function make_plots(results)
o = results.options;
r = results.runs(1);
c = results.controller;
f = figure('Name','IAC 1: horizontal surface','Color','w','Visible',o.visible, ...
    'Position',[100,100,1200,800]);
tiledlayout(2,3,'TileSpacing','compact');
nexttile; plot(r.t,[r.y(:,3),r.beta,r.barTheta]*180/pi,'LineWidth',1.2);
yline(0,'k:'); grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
legend('hat theta','beta','bar theta','true theta'); title('Implemented angle estimate');
nexttile; semilogy(r.t,max(abs(r.z),1e-15)*180/pi,'LineWidth',1.3); hold on;
semilogy(r.t,max(r.angleEnvelope,1e-15)*180/pi,'--','LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('|z| (deg)'); legend('Actual','Certified envelope');
title('Effective angle convergence');
nexttile; plot(r.t,r.force,'LineWidth',1.3); hold on;
yline(c.fd,'k--'); yline(results.certificate.forceMin,'r:'); grid on;
xlabel('Time (s)'); ylabel('Force (N)'); legend('Reaction','Desired','Certified minimum');
title('Scalar normal force');
nexttile; plot(r.t,1000*r.error,'LineWidth',1.3); grid on;
xlabel('Time (s)'); ylabel('Motion error (mm)'); title('Tangential regulation');
nexttile; hold on; colors = lines(numel(results.runs));
for k = 1:numel(results.runs)
    plot(results.runs(k).t,results.runs(k).z*180/pi, ...
        'Color',colors(k,:),'LineWidth',1.2,'DisplayName',results.runs(k).name);
end
grid on; xlabel('Time (s)'); ylabel('z (deg)'); legend('Interpreter','none');
title('Cross-check scenarios');
nexttile; plot(r.t,r.tau,'LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('Torque (N m)'); legend('tau 1','tau 2');
title('Actuator torques');
sgtitle(sprintf('IAC 1 | true horizontal surface | initial effective tilt %+.1f deg', ...
    results.runs(1).spec.angleDeg));
if o.saveResults
    exportgraphics(f,fullfile(o.outputDir,'iac1_summary.png'),'Resolution',180);
    savefig(f,fullfile(o.outputDir,'iac1_summary.fig'));
end
end

function write_summary(results)
path = fullfile(results.options.outputDir,'verification.txt');
fid = fopen(path,'w');
assert(fid>=0,'Could not open verification output.');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'IAC 1 independent ODE verification\nSource: %s\n',results.source);
fprintf(fid,'ODE state: [x_t, dx_t, hat_theta]; exact active rigid contact.\n');
fprintf(fid,'True surface: y=0.4 m, n=[0;1]. Sector: +/-6 deg.\n');
fprintf(fid,'Analytic inertia bounds: [%.9g, %.9g] kg\n', ...
    results.certificate.massMin,results.certificate.massMax);
fprintf(fid,'Certified force minimum: %.9g N; alpha_z: %.9g /s\n', ...
    results.certificate.forceMin,results.certificate.alphaZ);
for k = 1:numel(results.runs)
    run = results.runs(k);
    fprintf(fid,'\n%s\n',run.name);
    fprintf(fid,'Initial/final z: %.9g / %.9g deg\n',run.z(1)*180/pi,run.z(end)*180/pi);
    fprintf(fid,'Final motion error: %.9g m; velocity: %.9g m/s\n',run.error(end),run.y(end,2));
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
