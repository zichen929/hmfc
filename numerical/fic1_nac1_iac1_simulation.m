function results = fic1_nac1_iac1_simulation(options)
%FIC1_NAC1_IAC1_SIMULATION Run and compare three independent controllers.
%   results = fic1_nac1_iac1_simulation;
%   results = fic1_nac1_iac1_simulation(struct('visible','off'));
%   Set initialAngleDeg to choose the common NAC/IAC initial estimate.
%   This entry point executes FIC 1, NAC 1 and IAC 1 afresh, checks their
%   common conditions, and compares their MAIN regulation cases with kf=0.
%   For NAC 1 alone, use nac1_simulation. Controllers remain independent.
%   All outputs go to numerical/results/fic1_nac1_iac1/, including per-controller data.
if nargin==0, options=struct(); end
common=hmfc_parameters();
defaults=struct('makePlots',true,'saveResults',true,'visible','on','initialAngleDeg',5, ...
    'duration',common.duration,'initialVelocity',common.initialVelocity, ...
    'sectorHalfWidthDeg',6,'sectorMarginDeg',1e-6, ...
    'allowNacOutsideOperatingRegion',false,'outputDir', ...
    fullfile(hmfc_numerical_root(),'results','fic1_nac1_iac1'));
options=hmfc_simulation_options(options,defaults);
validateattributes(options.initialAngleDeg,{'numeric'},{'real','finite','scalar'});
if options.saveResults && ~exist(options.outputDir,'dir'), mkdir(options.outputDir); end
fprintf('\nFIC 1 / NAC 1 / IAC 1: fresh independent simulations\n');
% Suppress individual figures; this entry point creates comparison figures.
% Explicit subdirectories preserve the standalone runners' output folders.
child=struct('makePlots',false,'saveResults',options.saveResults, ...
    'visible',options.visible,'duration',options.duration,'initialVelocity',options.initialVelocity);
ficOptions=child; ficOptions.controllerIds=1;
ficOptions.outputDir=fullfile(options.outputDir,'fic1');
fic=fic_simulation(ficOptions);
iacOptions=child; iacOptions.initialAngleDeg=options.initialAngleDeg;
iacOptions.sectorHalfWidthDeg=options.sectorHalfWidthDeg;
iacOptions.sectorMarginDeg=options.sectorMarginDeg;
iacOptions.outputDir=fullfile(options.outputDir,'iac1');
iac=iac1_simulation(iacOptions);
nacOptions=child; nacOptions.initialAngleDeg=options.initialAngleDeg;
nacOptions.initialAngleLimitDeg=iac.certificate.sectorHalfWidthUsedDeg;
nacOptions.enforceOperatingRegion=~options.allowNacOutsideOperatingRegion;
nacOptions.outputDir=fullfile(options.outputDir,'nac1');
nac=nac1_simulation(nacOptions);
checks=check_conditions(fic,nac,iac);
% Common run fields allow collecting the primary cases without signal adapters.
runs=hmfc_append_runs(fic.runs(1),nac.runs(1));
runs=hmfc_append_runs(runs,iac.runs(1));
trueAngle=atan2(-iac.plant.n(1),iac.plant.n(2));
fd=fic.controller.fd;
metrics=repmat(struct('finalMotionError_m',0,'finalForce_N',0, ...
    'finalForceError_N',0,'minForce_N',0,'finalAngleError_deg',0, ...
    'stayedInOperatingRegion',true),1,numel(runs));
for k=1:numel(runs)
    r=runs(k);
    metrics(k)=struct('finalMotionError_m',r.error(end), ...
        'finalForce_N',r.force(end),'finalForceError_N',r.force(end)-fd, ...
        'minForce_N',min(r.force),'finalAngleError_deg',(r.angle(end)-trueAngle)*180/pi, ...
        'stayedInOperatingRegion',r.operatingRegionSatisfied);
    if options.saveResults
        signals=table(r.t,r.x,r.error,r.force,r.angle,r.tau(:,1),r.tau(:,2), ...
        'VariableNames',{'time_s','x_t_m','motion_error_m','force_N', ...
        'normal_angle_rad','tau1_Nm','tau2_Nm'});
        writetable(signals,fullfile(options.outputDir, ...
            sprintf('%s_primary.csv',lower(strrep(r.controller,' ','')))));
    end
end
summary=struct2table(metrics);
summary=addvars(summary,string({runs.controller})','Before',1,'NewVariableNames','controller');
results=struct('options',options,'fic1',fic,'nac1',nac,'iac1',iac, ...
    'runs',runs,'summary',summary,'checks',checks,'allChecksPassed',true, ...
    'allStayedInOperatingRegion',all([runs.operatingRegionSatisfied]));
if options.saveResults
    writetable(summary,fullfile(options.outputDir,'summary.csv'));
    save(fullfile(options.outputDir,'fic1_nac1_iac1_results.mat'),'results');
    write_report(results);
end
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
        isequal(nac.robot.(fields{j}),fic.robot.(fields{j})), ...
        'Comparison:Robot','Robot parameter mismatch: %s.',fields{j});
end
assert(isequal(nac.plant.n,iac.plant.n) && isequal(nac.plant.n,fic.plant.n));
assert(isequal(nac.plant.T,iac.plant.T) && isequal(nac.plant.T,fic.plant.T));
assert(isequal(nac.plant.p0,iac.plant.p0) && isequal(nac.plant.p0,fic.plant.p0));
for name={'kp','kv','fd'}
    key=name{1};
    assert(nac.controller.(key)==fic.controller.(key) && nac.controller.(key)==iac.controller.(key));
end
assert(nac.options.initialVelocity==fic.options.initialVelocity && ...
    nac.options.initialVelocity==iac.options.initialVelocity);
domain=iac.certificate.domain;
for name={'xMin','xMax','q2Min','q2Max'}
    key=name{1};
    assert(nac.plant.(key)==domain.(key) && nac.plant.(key)==fic.plant.(key));
end
a=fic.runs(1); b=nac.runs(1); d=iac.runs(1);
assert(strcmp(a.mode,'regulation') && strcmp(b.mode,'regulation') && a.kf==0 && b.kf==0);
assert(isequal(a.t,b.t) && isequal(b.t,d.t),'Comparison:Time','Output grids differ.');
assert(max(vecnorm(a.reference-b.reference,2,2))<1e-14 && ...
    max(vecnorm(b.reference-d.reference,2,2))<1e-14,'Comparison:Reference','References differ.');
assert(norm(a.position(1,:)-b.position(1,:))<1e-14 && norm(b.position(1,:)-d.position(1,:))<1e-14);
assert(norm(a.velocity(1,:)-b.velocity(1,:))<1e-14 && ...
    norm(b.velocity(1,:)'-iac.plant.T*d.state(1,2))<1e-14);
assert(abs(nac.controller.fixedAngle-d.barTheta(1))<1e-14);
checks.initialNacIacTorqueDifference=norm(b.tau(1,:)-d.tau(1,:));
checks.initialNacIacForceDifference=abs(b.force(1)-d.force(1));
assert(checks.initialNacIacTorqueDifference<1e-10 && checks.initialNacIacForceDifference<1e-10);
checks.identicalPhysicalConditions=true;
checks.identicalReferenceAndOutputTimes=true;
checks.identicalInitialNacIacNormal=true;
checks.nacStayedInOperatingRegion=b.operatingRegionSatisfied;
checks.passed=true;
end

function plot_comparison(results)
o=results.options;
r=results.runs;
t=r(1).t;
fig=figure('Name','FIC 1 / NAC 1 / IAC 1 comparison','Color','w', ...
    'Position',[100,100,1100,750],'Visible',o.visible);
hmfc_latex_figure(fig);
tiledlayout(2,2,'TileSpacing','compact');
nexttile; plot(t,[r.x],'LineWidth',1.4); hold on;
yline(results.nac1.task.targetX,'k:'); grid on;
xlabel('$t$ (s)'); ylabel('$x_t$ (m)'); title('Same regulation task');
legend('FIC 1','NAC 1','IAC 1','Desired');
if ~results.allStayedInOperatingRegion
    yline(results.nac1.plant.xMax,'r--','Certificate-region upper limit','HandleVisibility','off');
end
nexttile; plot(t,1000*[r.error],'LineWidth',1.4); grid on;
xlabel('$t$ (s)'); ylabel('Position error (mm)'); title('Motion error');
legend('FIC 1','NAC 1','IAC 1');
nexttile; plot(t,[r.force],'LineWidth',1.4); hold on;
yline(results.fic1.controller.fd,'k:'); grid on;
xlabel('$t$ (s)'); ylabel('Force (N)'); title('Actual scalar contact force');
legend('FIC 1','NAC 1','IAC 1','Desired');
nexttile; plot(t,[r.angle]*180/pi,'LineWidth',1.4); grid on;
xlabel('$t$ (s)'); ylabel('Normal angle (${}^\circ$)'); title('True, fixed and adaptive normal');
legend('FIC 1: true','NAC 1: fixed','IAC 1: effective');
titleText=sprintf('FIC 1 / NAC 1 / IAC 1 | $k_f=0$ | initial NAC/IAC angle $%+.1f^\\circ$',o.initialAngleDeg);
if ~results.allStayedInOperatingRegion
    titleText=sprintf('%s\nIAC sector $\\pm %.6f^\\circ$ | NAC leaves the original certificate region', ...
        titleText,results.iac1.certificate.sectorHalfWidthUsedDeg);
end
sgtitle(titleText,'Interpreter','latex');
hmfc_latex_figure(fig);
if o.saveResults
    exportgraphics(fig,fullfile(o.outputDir,'comparison.png'),'Resolution',180);
    savefig(fig,fullfile(o.outputDir,'comparison.fig'));
end
end

function write_report(results)
fid=fopen(fullfile(results.options.outputDir,'verification.txt'),'w');
assert(fid>=0); cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'FIC 1 / NAC 1 / IAC 1 comparison, separately integrated controllers with a shared robot model.\n');
fprintf(fid,'Main regulation case only: horizontal surface, target displacement 0.08 m, kf=0.\n');
fprintf(fid,'Matched robot, geometry, initial states, gains, reference, operating domain and output times.\n');
fprintf(fid,'NAC 1 and IAC 1 share the same initial normal estimate.\n');
fprintf(fid,'Initial NAC/IAC normal angle: %+.12g deg\n',results.options.initialAngleDeg);
fprintf(fid,'IAC sector half-width: %.12g deg; open supremum: %.12g deg\n', ...
    results.iac1.certificate.sectorHalfWidthUsedDeg,results.iac1.certificate.sectorHalfWidthSupremumDeg);
fprintf(fid,'Initial NAC/IAC torque difference: %.12g N m\n',results.checks.initialNacIacTorqueDifference);
fprintf(fid,'Initial NAC/IAC force difference: %.12g N\n',results.checks.initialNacIacForceDifference);
for k=1:numel(results.runs)
    r=results.runs(k);
    fprintf(fid,'%s: final motion error %.12g m, final force %.12g N\n',r.controller,r.error(end),r.force(end));
end
fprintf(fid,'All dynamics checks and comparison condition checks passed.\n');
if ~results.allStayedInOperatingRegion
    fprintf(fid,'NAC 1 left the ORIGINAL certificate region at approximately %.9g s.\n', ...
        results.nac1.runs(1).firstRegionExitTime);
    fprintf(fid,'Its exact physical model was continued in reachable nonsingular configurations.\n');
    fprintf(fid,'NAC 1 is not certified to stay in that region. FIC 1 and IAC 1 remained inside.\n');
end
end
