function results = nac1_simulation(options)
%NAC1_SIMULATION FIC 1 with a fixed, inaccurate surface-normal estimate.
%   results = nac1_simulation;
%   results = nac1_simulation(struct('visible','off'));
%   No calls to FIC/IAC implementations or settings. Base MATLAB only.
%   ODE state: [x_t; dx_t]. The normal estimate never changes.
%   Main case: regulation, kf=0, fixed +5 deg estimate, same scene as IAC 1.
%   Use fic1_nac1_iac1_simulation for the separate three-controller runner.
if nargin==0, options=struct(); end
defaults = struct('makePlots',true,'saveResults',true,'visible','on', ...
    'outputDir',fullfile(fileparts(mfilename('fullpath')),'results','nac1'), ...
    'initialAngleDeg',5,'initialVelocity',0,'duration',6);
assert(isstruct(options)&&isscalar(options),'options must be a scalar struct.');
names = fieldnames(options);
for i=1:numel(names)
    assert(isfield(defaults,names{i}),'Unknown option: %s',names{i});
    defaults.(names{i}) = options.(names{i});
end
options=defaults;
validateattributes(options.initialAngleDeg,{'numeric'},{'real','finite','scalar','>=',-6,'<=',6});
validateattributes(options.initialVelocity,{'numeric'},{'real','finite','scalar'});
validateattributes(options.duration,{'numeric'},{'real','finite','scalar','>=',6});

robot = struct('l1',0.6,'l2',0.5,'m1',2,'m2',1.5, ...
    'lc1',0.3,'lc2',0.25,'I1',2*0.6^2/12,'I2',1.5*0.5^2/12,'g',9.81);
plant = struct('p0',[0.6;0.4],'n',[0;1],'T',[1;0], ...
    'xMin',-0.04,'xMax',0.16,'q2Min',70*pi/180,'q2Max',115*pi/180);
% Controller parameters contain no true normal or tangent.
controller = struct('kp',25,'kv',10,'fd',10, ...
    'fixedAngle',options.initialAngleDeg*pi/180);
controller.nHat = [-sin(controller.fixedAngle);cos(controller.fixedAngle)];
task = struct('origin',[0.6;0.4],'axis',[1;0],'targetX',0.08);
time = linspace(0,options.duration,ceil(options.duration/0.002)+1)';
odeOptions = odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.01);
results = struct('controller','NAC 1','options',options,'robot',robot, ...
    'plant',plant,'control',controller,'task',task,'odeOptions',odeOptions, ...
    'source','TCST26E3.md / FIC 1, with fixed estimated normal','cases',[]);
fprintf('\nNAC 1: fixed normal estimate %+.3f deg, true horizontal surface\n',options.initialAngleDeg);
mode = 'regulation';
for kf=[0,2]
    rhs = @(t,y) plant_ode(t,y,robot,plant,controller,task,mode,kf);
    [t,y] = ode45(rhs,time,[0;options.initialVelocity],odeOptions);
    run = postprocess(t,y,robot,plant,controller,task,mode,kf);
    run.checks = verify_run(run,robot,plant,controller,task);
    results.cases = [results.cases;run];
    fprintf('%-10s kf=%g: final e=%+.6f mm, f=%.9f N; predicted e_inf=%+.6f mm\n', ...
        mode,kf,run.error(end)*1000,run.force(end),run.equilibrium.error*1000);
end
results.summary = struct2table([results.cases.metrics]);
results.summary = addvars(results.summary,string({results.cases.mode})', ...
    [results.cases.kf]','Before',1,'NewVariableNames',{'mode','kf'});
results.allChecksPassed = true;
if options.saveResults
    if ~exist(options.outputDir,'dir'), mkdir(options.outputDir); end
    writetable(results.summary,fullfile(options.outputDir,'summary.csv'));
    for i=1:numel(results.cases)
        run=results.cases(i);
        signals=table(run.t,run.y(:,1),run.y(:,2),run.error,run.force,run.uf, ...
            run.angle,run.tau(:,1),run.tau(:,2),'VariableNames', ...
            {'time_s','x_t_m','dx_t_mps','motion_error_m','force_N','force_command_N', ...
            'fixed_angle_rad','tau1_Nm','tau2_Nm'});
        writetable(signals,fullfile(options.outputDir,sprintf('%s_kf_%g.csv',run.mode,run.kf)));
    end
    save(fullfile(options.outputDir,'nac1_results.mat'),'results');
    write_report(results);
end
if options.makePlots, make_plots(results); end
fprintf('Both NAC 1 regulation cases passed dynamics, force, and bias checks.\n');
end

function dy = plant_ode(t,y,r,p,c,task,mode,kf)
a=evaluate(t,y,r,p,c,task,mode,kf);
dy=[y(2);p.T'*a.acceleration];
end

function a = evaluate(t,y,r,p,c,task,mode,kf)
assert(y(1)>p.xMin && y(1)<p.xMax,'NAC1:Region','Tangential domain exceeded.');
position=p.p0+p.T*y(1);
velocity=p.T*y(2);
m=robot_model(position,velocity,r,p);
[pd,dpd,ddpd]=reference(t,task,mode);
cmd=controller_command(position,velocity,pd,dpd,ddpd,m,c,kf);
% F = F0 + nHat*kf*f. Solve the instantaneous scalar-force feedback loop
% together with the ORIGINAL joint dynamics and true rigid constraint.
Jn=p.n'*m.J;
K=[m.M,-m.J'*(p.n+kf*c.nHat);Jn,0];
solution=K\[m.J'*cmd.F0-m.C*m.dq-m.G;-p.n'*m.Jdotdq];
qdd=solution(1:2);
f=solution(3);
assert(f>0,'NAC1:ContactLoss','Calculated reaction is not compressive.');
uf=c.fd+kf*(c.fd-f);
F=cmd.F0+c.nHat*kf*f;
tau=m.J'*F;
acceleration=m.J*qdd+m.Jdotdq;
z=c.fixedAngle-atan2(-p.n(1),p.n(2));
lt=p.T'*m.Lambda*p.T;
rho=cos(z)+(p.T'*m.Lambda*p.n)/lt*sin(z);
expectedForce=(1+kf)*rho/(1+kf*rho)*c.fd;
a=struct('position',position,'velocity',velocity,'pd',pd,'dpd',dpd, ...
    'acceleration',acceleration,'force',f,'uf',uf,'tau',tau, ...
    'error',p.T'*(pd-position),'angle',c.fixedAngle,'rho',rho,'q',m.q, ...
    'plantResidual',norm(m.M*qdd+m.C*m.dq+m.G-tau-m.J'*p.n*f,inf), ...
    'cartesianResidual',norm(m.Lambda*acceleration+m.eta-F-p.n*f,inf), ...
    'forceResidual',f-expectedForce, ...
    'motionResidual',p.T'*(acceleration-cmd.up)-uf*sin(z)/lt, ...
    'gap',p.n'*(position-p.p0),'normalVelocity',p.n'*velocity, ...
    'normalAcceleration',p.n'*acceleration);
end

function cmd=controller_command(position,velocity,pd,dpd,ddpd,m,c,kf)
% Exact FIC 1 Cartesian motion law; only n is replaced by fixed nHat.
% No true geometry or adaptive state enters this function.
cmd.up=ddpd+c.kv*(dpd-velocity)+c.kp*(pd-position);
cmd.F0=m.eta+m.Lambda*cmd.up-c.nHat*(1+kf)*c.fd;
end

function [pd,dpd,ddpd]=reference(~,task,mode)
assert(strcmp(mode,'regulation'),'NAC1:Reference','Only the matched regulation task is supported.');
pd=task.origin+task.axis*task.targetX;
dpd=zeros(2,1);
ddpd=zeros(2,1);
end

function m=robot_model(position,velocity,r,p)
cos2=(position'*position-r.l1^2-r.l2^2)/(2*r.l1*r.l2);
assert(abs(cos2)<1,'NAC1:Workspace','Unreachable or singular position.');
q2=acos(cos2);
assert(q2>p.q2Min && q2<p.q2Max,'NAC1:Region','Elbow domain exceeded.');
q1=atan2(position(2),position(1))-atan2(r.l2*sin(q2),r.l1+r.l2*cos(q2));
q=[q1;q2];
J=[-r.l1*sin(q1)-r.l2*sin(q1+q2),-r.l2*sin(q1+q2); ...
    r.l1*cos(q1)+r.l2*cos(q1+q2),r.l2*cos(q1+q2)];
dq=J\velocity;
A=r.I1+r.m1*r.lc1^2+r.m2*r.l1^2;
B=r.m2*r.l1*r.lc2;
D=r.I2+r.m2*r.lc2^2;
M=[A+D+2*B*cos(q2),D+B*cos(q2);D+B*cos(q2),D];
b=B*sin(q2);
C=[-b*dq(2),-b*sum(dq);b*dq(1),0];
G=[(r.m1*r.lc1+r.m2*r.l1)*r.g*cos(q1)+r.m2*r.lc2*r.g*cos(q1+q2); ...
    r.m2*r.lc2*r.g*cos(q1+q2)];
Jdotdq=-[r.l1*cos(q1)*dq(1)^2+r.l2*cos(q1+q2)*sum(dq)^2; ...
    r.l1*sin(q1)*dq(1)^2+r.l2*sin(q1+q2)*sum(dq)^2];
Lambda=J'\(M/J);
eta=J'\(C*dq+G)-Lambda*Jdotdq;
m=struct('q',q,'dq',dq,'J',J,'M',M,'C',C,'G',G, ...
    'Jdotdq',Jdotdq,'Lambda',Lambda,'eta',eta);
end

function eq=equilibrium(r,p,c,task,kf)
% Offline prediction: true geometry is used here only for verification.
% Variable inertia makes the regulation equilibrium an implicit equation.
xstar=fzero(@balance,[p.xMin+1e-8,p.xMax-1e-8]);
[~,uf,f]=balance(xstar);
eq=struct('x',xstar,'error',task.targetX-xstar,'force',f,'uf',uf);
    function [value,uf,f]=balance(x)
        m=robot_model(p.p0+p.T*x,[0;0],r,p);
        z=c.fixedAngle-atan2(-p.n(1),p.n(2));
        lt=p.T'*m.Lambda*p.T;
        rho=cos(z)+(p.T'*m.Lambda*p.n)/lt*sin(z);
        uf=(1+kf)*c.fd/(1+kf*rho);
        f=rho*uf;
        value=c.kp*(task.targetX-x)+uf*sin(z)/lt;
    end
end

function run=postprocess(t,y,r,p,c,task,mode,kf)
run=struct('mode',mode,'kf',kf,'t',t,'y',y);
for i=1:numel(t)
    a=evaluate(t(i),y(i,:)',r,p,c,task,mode,kf);
    if i==1
        fields=fieldnames(a);
        for j=1:numel(fields)
            run.(fields{j})=zeros(numel(t),numel(a.(fields{j})));
        end
    end
    for j=1:numel(fields), run.(fields{j})(i,:)=a.(fields{j})(:)'; end
end
run.equilibrium=equilibrium(r,p,c,task,kf);
metrics=struct('finalError',run.error(end),'finalForce',run.force(end), ...
    'minForce',min(run.force),'maxForceError',max(abs(run.force-c.fd)), ...
    'tailRmsError',sqrt(mean(run.error(t>=t(end)-2).^2)), ...
    'predictedRegulationError',run.equilibrium.error, ...
    'predictedRegulationForce',run.equilibrium.force);
run.metrics=metrics;
end

function checks=verify_run(run,r,p,c,task)
checks=struct('maxJointResidual',max(run.plantResidual), ...
    'maxCartesianResidual',max(run.cartesianResidual), ...
    'maxForceIdentityResidual',max(abs(run.forceResidual)), ...
    'maxMotionIdentityResidual',max(abs(run.motionResidual)), ...
    'maxGap',max(abs(run.gap)),'maxNormalVelocity',max(abs(run.normalVelocity)), ...
    'maxNormalAcceleration',max(abs(run.normalAcceleration)));
values=cell2mat(struct2cell(checks));
assert(all(isfinite(run.y),'all') && all(values<1e-9),'NAC1:Dynamics','Dynamics check failed.');
assert(all(run.force>0) && all(run.angle==c.fixedAngle));
assert(all(run.y(:,1)>p.xMin & run.y(:,1)<p.xMax));
assert(all(run.q(:,2)>p.q2Min & run.q(:,2)<p.q2Max));
if strcmp(run.mode,'regulation')
    checks.regulationEquilibriumError=abs(run.error(end)-run.equilibrium.error);
    checks.regulationEquilibriumForceError=abs(run.force(end)-run.equilibrium.force);
    assert(checks.regulationEquilibriumError<1e-6);
    assert(checks.regulationEquilibriumForceError<1e-5 && abs(run.y(end,2))<1e-6);
end
if c.fixedAngle==0
    % Zero mismatch must recover the independent analytical FIC 1 target.
    e0=[run.error(1);p.T'*(run.dpd(1,:)'-run.velocity(1,:)')];
    A=[0,1;-c.kp,-c.kv];
    exact=zeros(numel(run.t),1);
    for i=1:numel(run.t), v=expm(A*run.t(i))*e0; exact(i)=v(1); end
    checks.zeroAngleAnalyticError=max(abs(run.error-exact));
    assert(checks.zeroAngleAnalyticError<1e-7 && max(abs(run.force-c.fd))<1e-8);
end
% Verify the equilibrium prediction comes from the full joint plant as well.
a=evaluate(0,[run.equilibrium.x;0],r,p,c,task,'regulation',run.kf);
checks.equilibriumAcceleration=norm(a.acceleration,inf);
assert(checks.equilibriumAcceleration<1e-9);
checks.passed=true;
end

function make_plots(results)
o=results.options; c=results.control;
a=results.cases(1); b=results.cases(2);
f=figure('Name','NAC 1: fixed inaccurate normal','Color','w','Visible',o.visible, ...
    'Position',[100,100,1200,800]);
tiledlayout(2,3,'TileSpacing','compact');
nexttile; plot(a.t,[a.y(:,1),b.y(:,1)],'LineWidth',1.3); hold on;
yline(results.task.targetX,'k--'); grid on; xlabel('Time (s)'); ylabel('x_t (m)');
legend('k_f=0','k_f=2','Desired'); title('Regulation with fixed geometry error');
nexttile; plot(a.t,1000*[a.error,b.error],'LineWidth',1.3); hold on;
yline(a.equilibrium.error*1000,'b:'); yline(b.equilibrium.error*1000,'r:');
grid on; xlabel('Time (s)'); ylabel('Motion error (mm)'); title('Persistent motion bias');
legend('k_f=0','k_f=2','Predicted, k_f=0','Predicted, k_f=2');
nexttile; plot(a.t,[a.force,b.force],'LineWidth',1.3); hold on;
yline(c.fd,'k--'); grid on; xlabel('Time (s)'); ylabel('Force (N)');
legend('k_f=0','k_f=2','Desired'); title('Normal contact force');
nexttile; plot(a.t,a.angle*180/pi,'LineWidth',1.3); hold on; yline(0,'k--');
grid on; xlabel('Time (s)'); ylabel('Angle (deg)'); title('Constant normal estimate');
legend('Fixed estimate','True normal');
nexttile; plot(a.t,a.tau,'LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('Torque (N m)'); legend('tau 1','tau 2'); title('Main-case torques');
nexttile; hold on;
origin=results.plant.p0;
lineX=origin(1)+[-0.04,0.16];
plot(lineX,origin(2)+[0,0],'k--','LineWidth',1.2);
plot(lineX,origin(2)+tan(c.fixedAngle)*(lineX-origin(1)),'r:','LineWidth',1.2);
plot(a.position(:,1),a.position(:,2),'b','LineWidth',1.5);
plot(a.pd(1,1),a.pd(1,2),'ko','MarkerFaceColor','k');
plot(a.position(end,1),a.position(end,2),'bs','MarkerFaceColor','b');
axis equal; grid on; xlabel('p_x (m)'); ylabel('p_y (m)');
legend('True surface','Fixed estimated orientation','Actual path','Desired','Final');
title('Geometry and final position bias');
sgtitle(sprintf('NAC 1 | horizontal surface | fixed estimated normal %+.1f deg',o.initialAngleDeg));
if o.saveResults
    exportgraphics(f,fullfile(o.outputDir,'nac1_summary.png'),'Resolution',180);
    savefig(f,fullfile(o.outputDir,'nac1_summary.fig'));
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
for i=1:numel(results.cases)
    run=results.cases(i);
    fprintf(fid,'\n%s, kf=%g\nFinal error %.12g m; final force %.12g N; min force %.12g N\n', ...
        run.mode,run.kf,run.error(end),run.force(end),min(run.force));
    if strcmp(run.mode,'regulation')
        fprintf(fid,'Predicted equilibrium error %.12g m; force %.12g N\n', ...
            run.equilibrium.error,run.equilibrium.force);
    end
    fields=fieldnames(run.checks);
    for j=1:numel(fields), fprintf(fid,'%s: %.12g\n',fields{j},run.checks.(fields{j})); end
end
fprintf(fid,'\nAll checks passed. Established frictionless rigid contact and ideal known model only.\n');
end
