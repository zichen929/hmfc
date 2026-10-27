function results = fic1_simscape_simulation(options)
%FIC1_SIMSCAPE_SIMULATION Run native Multibody FIC1 and compare to the ODE.
% results = fic1_simscape_simulation(struct('makePlots',false));
% Four cases: regulation/tracking, kf=0/2. Requires Simscape Multibody.
if nargin==0, options=struct(); end
root=fileparts(mfilename('fullpath'));
defaults=struct('makePlots',true,'visible','off','rebuild',false, ...
    'outputDir',fullfile(root,'results','fic1_simscape'));
assert(isstruct(options)&&isscalar(options),'Expected a scalar options struct.');
for f=fieldnames(options)'
    assert(isfield(defaults,f{1}),'Unknown option: %s',f{1});
    defaults.(f{1})=options.(f{1});
end
options=defaults;
if ~exist(options.outputDir,'dir'), mkdir(options.outputDir); end
modelFile=fullfile(root,'hmfc_fic1_simscape.slx');
if options.rebuild || ~isfile(modelFile)
    modelFile=build_hmfc_simscape(root);
end
load_system(modelFile); [~,mdl]=fileparts(modelFile);
c=hmfc_simscape_parameters();
baseline=fic1_simulation(false,fullfile(options.outputDir,'ode_reference'));
for f={'l1','l2','m1','m2','lc1','lc2','I1','I2','g','kp','kv','fd','p0','n','T','q0'}
    assert(isequal(c.(f{1}),baseline.config.(f{1})),'Parameter mismatch: %s',f{1});
end
results=struct('config',c,'options',options,'modelFile',modelFile,'cases',[]);
t=(0:c.outputStep:c.duration)';
for k=1:numel(baseline.cases)
    ref=baseline.cases(k);
    mode=double(strcmp(ref.mode,'tracking'));
    input=Simulink.SimulationInput(mdl);
    input=input.setVariable('c',c,'Workspace',mdl);
    input=input.setVariable('mode',mode,'Workspace',mdl);
    input=input.setVariable('kf',ref.kf,'Workspace',mdl);
    input=input.setModelParameter('StopTime',num2str(c.duration), ...
        'OutputOption','SpecifiedOutputTimes','OutputTimes',mat2str(t',15));
    out=sim(input);
    r=struct('mode',ref.mode,'kf',ref.kf,'t',t);
    for f={'q','dq','qdd','force','p','pd','tau','uf'}
        ts=out.get([f{1},'_log']);
        [sampleTime,index]=unique(ts.Time,'last');
        if size(ts.Data,1)==numel(ts.Time)
            values=reshape(ts.Data,numel(ts.Time),[]);
        else
            values=reshape(ts.Data,[],numel(ts.Time))';
        end
        r.(f{1})=interp1(sampleTime,values(index,:),t,'linear');
    end
    r.f=r.force(:,2); r.ep=r.pd-r.p;
    r.x=(r.p-c.p0')*c.T;
    r.velocity=zeros(numel(t),2); r.dynamicResidual=zeros(numel(t),2);
    for j=1:numel(t)
        [M,C,G,J]=joint_model(r.q(j,:)',r.dq(j,:)',c);
        r.velocity(j,:)=(J*r.dq(j,:)')';
        r.dynamicResidual(j,:)=(M*r.qdd(j,:)'+C*r.dq(j,:)'+G ...
            -r.tau(j,:)'-J'*c.n*r.f(j))';
    end
    r.checks=struct( ...
        'maxPositionVsODE_m',max(vecnorm(r.p-ref.p,2,2)), ...
        'maxVelocityVsODE_mps',max(vecnorm(r.velocity-ref.dp,2,2)), ...
        'maxJointVsODE_rad',max(vecnorm(r.q-ref.q,2,2)), ...
        'maxTorqueVsODE_Nm',max(vecnorm(r.tau-ref.tau,2,2)), ...
        'maxForceVsODE_N',max(abs(r.f-ref.f)), ...
        'maxForceError_N',max(abs(r.f-c.fd)), ...
        'minForce_N',min(r.f), ...
        'maxGap_m',max(abs((r.p-c.p0')*c.n)), ...
        'maxNormalVelocity_mps',max(abs(r.velocity*c.n)), ...
        'maxTangentialReaction_N',max(abs(r.force(:,1))), ...
        'maxJointDynamicResidual_Nm',max(vecnorm(r.dynamicResidual,2,2)), ...
        'finalMotionError_m',norm(r.ep(end,:)));
    z=r.checks;
    assert(z.maxPositionVsODE_m<2e-6 && z.maxVelocityVsODE_mps<2e-5, ...
        'FIC1Simscape:Motion','Motion differs from the ODE reference.');
    assert(z.maxJointVsODE_rad<1e-5 && z.maxTorqueVsODE_Nm<1e-3, ...
        'FIC1Simscape:Robot','Joint state or torque differs from the ODE reference.');
    assert(z.maxForceError_N<1e-3 && z.maxForceVsODE_N<1e-3 && z.minForce_N>0, ...
        'FIC1Simscape:Force','Force tracking or compressive-contact check failed.');
    assert(z.maxGap_m<1e-8 && z.maxNormalVelocity_mps<1e-7 ...
        && z.maxTangentialReaction_N<1e-7,'FIC1Simscape:Constraint','Contact constraint failed.');
    assert(z.maxJointDynamicResidual_Nm<1e-4,'FIC1Simscape:Dynamics', ...
        'Multibody acceleration/force does not match the original joint dynamics.');
    assert(all(r.q(:,2)>c.q2Min & r.q(:,2)<c.q2Max) ...
        && all(r.x>c.xMin & r.x<c.xMax),'Operating region violated.');
    r.checks.passed=true;
    if isempty(results.cases), results.cases=r; else, results.cases(end+1)=r; end
    data=array2table([t,r.q,r.dq,r.qdd,r.p,r.pd,r.f,r.tau], ...
        'VariableNames',{'time_s','q1_rad','q2_rad','dq1_radps','dq2_radps', ...
        'qdd1_radps2','qdd2_radps2','px_m','py_m','pdx_m','pdy_m','force_N','tau1_Nm','tau2_Nm'});
    writetable(data,fullfile(options.outputDir,sprintf('%s_kf%d.csv',r.mode,r.kf)));
    fprintf('Simscape %-10s kf=%g: max position vs ODE %.3e m, force error %.3e N\n', ...
        r.mode,r.kf,z.maxPositionVsODE_m,z.maxForceError_N);
end
results.summary=struct2table([results.cases.checks]);
results.summary=addvars(results.summary,string({results.cases.mode})', ...
    [results.cases.kf]','Before',1,'NewVariableNames',{'mode','kf'});
results.allChecksPassed=true;
writetable(results.summary,fullfile(options.outputDir,'summary.csv'));
save(fullfile(options.outputDir,'fic1_simscape_results.mat'),'results');
fid=fopen(fullfile(options.outputDir,'verification.txt'),'w');
assert(fid>=0); cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'Native Simscape Multibody 2R arm / FIC1\nMATLAB %s\n',version);
fprintf(fid,'Rigid maintained-contact branch: pin-slot constraint, no penalty spring.\n');
fprintf(fid,'Sensor direction: Base on Follower, resolved in Base (world axes).\n');
fprintf(fid,'Four cases passed motion, force, contact, region and joint-dynamics checks.\n');
fprintf(fid,'%s\n',evalc('disp(results.summary)'));
if options.makePlots, plot_fic1_simscape_results(results,baseline); end
fprintf('All four native Simscape FIC1 cases passed. Results: %s\n',options.outputDir);
end

function [M,C,G,J]=joint_model(q,dq,c)
A=c.I1+c.m1*c.lc1^2+c.m2*c.l1^2; B=c.m2*c.l1*c.lc2;
D=c.I2+c.m2*c.lc2^2;
M=[A+D+2*B*cos(q(2)),D+B*cos(q(2));D+B*cos(q(2)),D];
b=B*sin(q(2)); C=[-b*dq(2),-b*sum(dq);b*dq(1),0];
G=[(c.m1*c.lc1+c.m2*c.l1)*c.g*cos(q(1))+c.m2*c.lc2*c.g*cos(sum(q)); ...
    c.m2*c.lc2*c.g*cos(sum(q))];
J=[-c.l1*sin(q(1))-c.l2*sin(sum(q)),-c.l2*sin(sum(q)); ...
    c.l1*cos(q(1))+c.l2*cos(sum(q)),c.l2*cos(sum(q))];
end

