function results = fic1_nac1_iac1_simulation(options)
%FIC1_NAC1_IAC1_SIMULATION Run and compare three independent controllers.
%   results = fic1_nac1_iac1_simulation;
%   results = fic1_nac1_iac1_simulation(struct('visible','off'));
%   This entry point executes FIC 1, NAC 1 and IAC 1 afresh, checks their
%   common conditions, and compares their MAIN regulation cases with kf=0.
%   For NAC 1 alone, use nac1_simulation. Controllers remain independent.
%   All outputs go to results/fic1_nac1_iac1/, including per-controller data.
if nargin==0, options=struct(); end
defaults=struct('makePlots',true,'visible','on','outputDir', ...
    fullfile(fileparts(mfilename('fullpath')),'results','fic1_nac1_iac1'));
assert(isstruct(options)&&isscalar(options),'options must be a scalar struct.');
names=fieldnames(options);
for k=1:numel(names)
    assert(isfield(defaults,names{k}),'Unknown option: %s',names{k});
    defaults.(names{k})=options.(names{k});
end
options=defaults;
if ~exist(options.outputDir,'dir'), mkdir(options.outputDir); end
fprintf('\nFIC 1 / NAC 1 / IAC 1: fresh independent simulations\n');
% Suppress individual figures; this entry point creates comparison figures.
% Explicit subdirectories preserve the standalone runners' output folders.
fic=fic1_simulation(false,fullfile(options.outputDir,'fic1'));
nac=nac1_simulation(struct('makePlots',false,'outputDir', ...
    fullfile(options.outputDir,'nac1')));
iac=iac1_simulation(struct('makePlots',false,'outputDir', ...
    fullfile(options.outputDir,'iac1')));
checks=check_conditions(fic,nac,iac);
a=fic.cases(1); b=nac.cases(1); d=iac.runs(1);
trueAngle=atan2(-iac.plant.n(1),iac.plant.n(2));
primary(1)=struct('name','FIC 1','t',a.t,'x',a.z(:,1), ...
    'error',a.ep*fic.config.T,'force',a.f, ...
    'angle',repmat(trueAngle,size(a.t)),'tau',a.tau);
primary(2)=struct('name','NAC 1','t',b.t,'x',b.y(:,1), ...
    'error',b.error,'force',b.force,'angle',b.angle,'tau',b.tau);
primary(3)=struct('name','IAC 1','t',d.t,'x',d.y(:,1), ...
    'error',d.error,'force',d.force,'angle',d.barTheta,'tau',d.tau);
fd=fic.config.fd;
metrics=repmat(struct('finalMotionError_m',0,'finalForce_N',0, ...
    'finalForceError_N',0,'minForce_N',0,'finalAngleError_deg',0),1,numel(primary));
for k=1:numel(primary)
    r=primary(k);
    metrics(k)=struct('finalMotionError_m',r.error(end), ...
        'finalForce_N',r.force(end),'finalForceError_N',r.force(end)-fd, ...
        'minForce_N',min(r.force),'finalAngleError_deg',(r.angle(end)-trueAngle)*180/pi);
    signals=table(r.t,r.x,r.error,r.force,r.angle,r.tau(:,1),r.tau(:,2), ...
        'VariableNames',{'time_s','x_t_m','motion_error_m','force_N', ...
        'normal_angle_rad','tau1_Nm','tau2_Nm'});
    writetable(signals,fullfile(options.outputDir, ...
        sprintf('%s_primary.csv',lower(strrep(r.name,' ','')))));
end
summary=struct2table(metrics);
summary=addvars(summary,string({primary.name})','Before',1,'NewVariableNames','controller');
results=struct('options',options,'fic1',fic,'nac1',nac,'iac1',iac, ...
    'primary',primary,'summary',summary,'checks',checks,'allChecksPassed',true);
writetable(summary,fullfile(options.outputDir,'summary.csv'));
save(fullfile(options.outputDir,'fic1_nac1_iac1_results.mat'),'results');
write_report(results);
if options.makePlots, plot_comparison(results); end
disp(summary);
fprintf('All comparison condition checks passed. Results: %s\n',options.outputDir);
end

function checks=check_conditions(fic,nac,iac)
assert(fic.allChecksPassed && nac.allChecksPassed && ...
    all(arrayfun(@(run) run.checks.passed,iac.runs)));
fields=fieldnames(iac.robot);
for j=1:numel(fields)
    assert(isequal(nac.robot.(fields{j}),iac.robot.(fields{j})) && ...
        isequal(nac.robot.(fields{j}),fic.config.(fields{j})), ...
        'Comparison:Robot','Robot parameter mismatch: %s.',fields{j});
end
assert(isequal(nac.plant.n,iac.plant.n) && isequal(nac.plant.n,fic.config.n));
assert(isequal(nac.plant.T,iac.plant.T) && isequal(nac.plant.T,fic.config.T));
assert(isequal(nac.plant.p0,iac.plant.p0) && isequal(nac.plant.p0,fic.config.p0));
for name={'kp','kv','fd'}
    key=name{1};
    assert(nac.control.(key)==fic.config.(key) && nac.control.(key)==iac.controller.(key));
end
assert(nac.options.initialVelocity==fic.config.initialVelocity && ...
    nac.options.initialVelocity==iac.options.initialVelocity);
domain=iac.certificate.domain;
for name={'xMin','xMax','q2Min','q2Max'}
    key=name{1};
    assert(nac.plant.(key)==domain.(key) && nac.plant.(key)==fic.config.(key));
end
a=fic.cases(1); b=nac.cases(1); d=iac.runs(1);
assert(strcmp(a.mode,'regulation') && strcmp(b.mode,'regulation') && a.kf==0 && b.kf==0);
assert(isequal(a.t,b.t) && isequal(b.t,d.t),'Comparison:Time','Output grids differ.');
assert(max(vecnorm(a.pd-b.pd,2,2))<1e-14 && ...
    max(vecnorm(b.pd-d.reference',2,2))<1e-14,'Comparison:Reference','References differ.');
assert(norm(a.p(1,:)-b.position(1,:))<1e-14 && norm(b.position(1,:)-d.p(1,:))<1e-14);
assert(norm(a.dp(1,:)-b.velocity(1,:))<1e-14 && ...
    norm(b.velocity(1,:)'-iac.plant.T*d.y(1,2))<1e-14);
assert(abs(nac.control.fixedAngle-d.barTheta(1))<1e-14);
checks.initialNacIacTorqueDifference=norm(b.tau(1,:)-d.tau(1,:));
checks.initialNacIacForceDifference=abs(b.force(1)-d.force(1));
assert(checks.initialNacIacTorqueDifference<1e-10 && checks.initialNacIacForceDifference<1e-10);
checks.identicalPhysicalConditions=true;
checks.identicalReferenceAndOutputTimes=true;
checks.identicalInitialNacIacNormal=true;
checks.passed=true;
end

function plot_comparison(results)
o=results.options;
r=results.primary;
t=r(1).t;
fig=figure('Name','FIC 1 / NAC 1 / IAC 1 comparison','Color','w', ...
    'Position',[100,100,1100,750],'Visible',o.visible);
tiledlayout(2,2,'TileSpacing','compact');
nexttile; plot(t,[r.x],'LineWidth',1.4); hold on;
yline(results.nac1.task.targetX,'k:'); grid on;
xlabel('Time (s)'); ylabel('x_t (m)'); title('Same regulation task');
legend('FIC 1','NAC 1','IAC 1','Desired');
nexttile; plot(t,1000*[r.error],'LineWidth',1.4); grid on;
xlabel('Time (s)'); ylabel('Position error (mm)'); title('Motion error');
legend('FIC 1','NAC 1','IAC 1');
nexttile; plot(t,[r.force],'LineWidth',1.4); hold on;
yline(results.fic1.config.fd,'k:'); grid on;
xlabel('Time (s)'); ylabel('Force (N)'); title('Actual scalar contact force');
legend('FIC 1','NAC 1','IAC 1','Desired');
nexttile; plot(t,[r.angle]*180/pi,'LineWidth',1.4); grid on;
xlabel('Time (s)'); ylabel('Normal angle (deg)'); title('True, fixed and adaptive normal');
legend('FIC 1: true','NAC 1: fixed','IAC 1: effective');
sgtitle('FIC 1 / NAC 1 / IAC 1 | identical physical conditions | k_f=0');
exportgraphics(fig,fullfile(o.outputDir,'comparison.png'),'Resolution',180);
savefig(fig,fullfile(o.outputDir,'comparison.fig'));
end

function write_report(results)
fid=fopen(fullfile(results.options.outputDir,'verification.txt'),'w');
assert(fid>=0); cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'FIC 1 / NAC 1 / IAC 1 comparison, freshly integrated independent implementations.\n');
fprintf(fid,'Main regulation case only: horizontal surface, target displacement 0.08 m, kf=0.\n');
fprintf(fid,'Matched robot, geometry, initial states, gains, reference, operating domain and output times.\n');
fprintf(fid,'NAC 1 and IAC 1 share the same initial normal estimate.\n');
fprintf(fid,'Initial NAC/IAC torque difference: %.12g N m\n',results.checks.initialNacIacTorqueDifference);
fprintf(fid,'Initial NAC/IAC force difference: %.12g N\n',results.checks.initialNacIacForceDifference);
for k=1:numel(results.primary)
    r=results.primary(k);
    fprintf(fid,'%s: final motion error %.12g m, final force %.12g N\n',r.name,r.error(end),r.force(end));
end
fprintf(fid,'All constituent checks and comparison condition checks passed.\n');
end
