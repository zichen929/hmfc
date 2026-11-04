function tests = test_numerical_interface
%TEST_NUMERICAL_INTERFACE Public contract, physical equivalence and output control.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
    {root,fullfile(root,'shared')}));
testCase.TestData.outputRoot = tempname;
options = struct('makePlots',false,'saveResults',false, ...
    'outputDir',testCase.TestData.outputRoot);
ficOptions = options; ficOptions.controllerIds = 1;
testCase.TestData.fic = fic_simulation(ficOptions);
testCase.TestData.nac = nac1_simulation(options);
testCase.TestData.iac = iac1_simulation(options);
end

function testSharedContract(testCase)
results = {testCase.TestData.fic,testCase.TestData.nac,testCase.TestData.iac};
topFields = {'name','options','robot','plant','controller','task','odeOptions', ...
    'runs','summary','checks','allChecksPassed'};
runFields = {'controller','name','mode','kf','t','state','initialState','x', ...
    'position','velocity','acceleration','reference','referenceVelocity', ...
    'referenceAcceleration','positionError','velocityError','error', ...
    'force','forceCommand','angle','tau','q','dq','ddq', ...
    'jointResidual','cartesianResidual','metrics','checks', ...
    'operatingRegionSatisfied','firstRegionExitTime'};
for k = 1:numel(results)
    result = results{k};
    verifyTrue(testCase,all(isfield(result,topFields)));
    verifyFalse(testCase,isfield(result,'cases') || isfield(result,'config'));
    verifyTrue(testCase,result.allChecksPassed && result.checks.passed);
    verifyEqual(testCase,height(result.summary),numel(result.runs));
    verifyTrue(testCase,ismember('firstRegionExitTime',result.summary.Properties.VariableNames));
    verifyFalse(testCase,ismember('firstRegionExitTime_s',result.summary.Properties.VariableNames));
    verifyFalse(testCase,isfield(result.controller,'l1'));
    if k>1
        verifyFalse(testCase,isfield(result.controller,'n') || isfield(result.controller,'T'));
    end
    for j = 1:numel(result.runs)
        run = result.runs(j);
        verifyTrue(testCase,all(isfield(run,runFields)));
        verifyFalse(testCase,any(isfield(run,{'qdd','dynamicResidual','plantResidual','minEig','maxEig', ...
            'exactEp','exactEv','errorResidual'})));
        verifyFalse(testCase,isfield(run.metrics,'maxContactPositionError'));
        verifyFalse(testCase,any(isfield(run.metrics,{'maxAnalyticError','firstRegionExitTime_s'})));
        verifyFalse(testCase,any(isfield(run.checks,{'maxContactPositionError','maxForceIdentityResidual'})));
        verifyTrue(testCase,run.checks.passed);
        verifyTrue(testCase,all(isfield(run.checks,{'maxJointResidual','maxCartesianResidual', ...
            'maxGap','maxNormalVelocity','maxNormalAcceleration','operatingRegionSatisfied'})));
        verifySize(testCase,run.t,[3001,1]);
        verifySize(testCase,run.state,[3001,2+(k==3)]);
        for name = {'position','velocity','acceleration','reference','referenceVelocity', ...
                'referenceAcceleration','tau','q','dq','ddq','jointResidual','cartesianResidual'}
            verifySize(testCase,run.(name{1}),[3001,2]);
        end
        verifyEqual(testCase,run.initialState,run.state(1,:)');
        verifyEqual(testCase,run.x,run.state(:,1));
        verifyEqual(testCase,run.error,(run.reference-run.position)*result.plant.T,'AbsTol',1e-14);
        verifyEqual(testCase,run.position,result.plant.p0'+run.x*result.plant.T','AbsTol',1e-14);
        verifyEqual(testCase,run.velocity,run.state(:,2)*result.plant.T','AbsTol',1e-14);
        verifyEqual(testCase,run.metrics.finalError,run.error(end));
        verifyEqual(testCase,run.metrics.finalForce,run.force(end));
        verifyEqual(testCase,run.metrics.firstRegionExitTime,run.firstRegionExitTime);
        verifyEqual(testCase,run.checks.maxJointResidual,max(vecnorm(run.jointResidual,inf,2)));
        verifyEqual(testCase,run.checks.maxCartesianResidual,max(vecnorm(run.cartesianResidual,inf,2)));
        verifyTrue(testCase,run.operatingRegionSatisfied);
        if k~=2
            verifyTrue(testCase,all(isfield(run,{'minLambda','maxLambda'})));
            verifySize(testCase,run.minLambda,[3001,1]);
            verifySize(testCase,run.maxLambda,[3001,1]);
        end
        if k==1
            verifyTrue(testCase,isfield(run.metrics,'maxGap'));
            verifyTrue(testCase,all(isfield(run, ...
                {'analyticPositionError','analyticVelocityError','errorDynamicsResidual'})));
            for name = {'analyticPositionError','analyticVelocityError','errorDynamicsResidual'}
                verifySize(testCase,run.(name{1}),[3001,2]);
            end
            verifyTrue(testCase,isfield(run.metrics,'maxAnalyticPositionError'));
            verifyTrue(testCase,isfield(run.checks,'maxAnalyticPositionError'));
        elseif k==2
            verifyTrue(testCase,isfield(run,'forceResidual'));
            verifyTrue(testCase,isfield(run.checks,'maxForceResidual'));
            verifyFalse(testCase,isfield(run.checks,'maxForceRatioResidual'));
            verifyLessThan(testCase,run.checks.maxForceResidual,1e-9);
        else
            verifyTrue(testCase,isfield(run,'forceRatioResidual'));
            verifySize(testCase,run.forceRatioResidual,[3001,1]);
            verifyTrue(testCase,isfield(run.checks,'maxForceRatioResidual'));
            verifyFalse(testCase,isfield(run.checks,'maxForceResidual'));
            verifyLessThan(testCase,run.checks.maxForceRatioResidual,1e-10);
        end
    end
end
verifyFalse(testCase,isfolder(testCase.TestData.outputRoot));
end

function testEstimatorAndJointConstraint(testCase)
result = testCase.TestData.iac;
for k = 1:numel(result.runs)
    run = result.runs(k);
    verifyEqual(testCase,run.hatTheta,run.state(:,3));
    verifyTrue(testCase,all(isfield(run,{'dz','dzTheory'})));
    verifyFalse(testCase,any(isfield(run,{'zDot','zDotTheory'})));
    verifyEqual(testCase,run.angle,run.hatTheta+run.beta,'AbsTol',1e-15);
    verifyEqual(testCase,run.angleError,run.angle,'AbsTol',1e-15);
    verifyLessThan(testCase,run.checks.maxJointResidual,1e-9);
    verifyLessThan(testCase,max(abs(run.acceleration*result.plant.n)),1e-9);
end
verifyLessThan(testCase,max(abs(result.runs(3).angleError)),2e-9);
verifyGreaterThan(testCase,abs(result.runs(3).beta(1)),1e-4);
end

function testZeroMismatchRecoversFic(testCase)
options = struct('makePlots',false,'saveResults',false,'initialAngleDeg',0);
nac = nac1_simulation(options);
iac = iac1_simulation(options);
fic = testCase.TestData.fic.runs(1);
for run = {nac.runs(1),iac.runs(1)}
    verifyEqual(testCase,run{1}.t,fic.t);
    verifyEqual(testCase,run{1}.position,fic.position,'AbsTol',1e-9);
    verifyEqual(testCase,run{1}.force,fic.force,'AbsTol',1e-9);
    verifyEqual(testCase,run{1}.tau,fic.tau,'AbsTol',1e-8);
end
end

function testStandardAndWideComparisonWithoutSaving(testCase)
options = struct('makePlots',false,'saveResults',false,'outputDir',tempname);
standard = fic1_nac1_iac1_simulation(options);
verifyTrue(testCase,standard.allChecksPassed && standard.allStayedInOperatingRegion);
verifyEqual(testCase,numel(standard.runs),3);
verifyEqual(testCase,standard.runs(2).angle(1),standard.runs(3).angle(1),'AbsTol',1e-14);
verifyFalse(testCase,isfolder(options.outputDir));
wide = fic1_nac1_iac1_max_angle_simulation(options);
verifyTrue(testCase,wide.allChecksPassed);
verifyFalse(testCase,wide.allStayedInOperatingRegion);
verifyFalse(testCase,wide.nac1.runs(1).operatingRegionSatisfied);
verifyEqual(testCase,wide.nac1.runs(1).firstRegionExitTime,0.4,'AbsTol',0.002);
verifyEqual(testCase,wide.nac1.runs(1).metrics.firstRegionExitTime, ...
    wide.nac1.runs(1).firstRegionExitTime);
verifyTrue(testCase,wide.iac1.runs(1).operatingRegionSatisfied);
verifyFalse(testCase,isfolder(options.outputDir));
end

function testPlotsWithoutSaving(testCase)
cleanup = onCleanup(@() close('all','force'));
options = struct('makePlots',true,'saveResults',false,'visible','off','outputDir',tempname);
ficOptions = options; ficOptions.controllerIds = [1,3];
fic_simulation(ficOptions);
nac1_simulation(options);
iac1_simulation(options);
verifyFalse(testCase,isfolder(options.outputDir));
figures = findall(groot,'Type','figure');
verifyNotEmpty(testCase,figures);
verifyTrue(testCase,all(strcmp(get(figures,'Visible'),'off')));
end

function testCommonSavedSignals(testCase)
root = tempname;
cleanup = onCleanup(@() remove_output(root));
options = struct('makePlots',false,'saveResults',true,'outputDir',root);
runners = {@fic_simulation,@nac1_simulation,@iac1_simulation};
for k = 1:numel(runners)
    child = options; child.outputDir = fullfile(root,sprintf('controller%d',k));
    if k==1, child.controllerIds = 1; end
    result = runners{k}(child);
    summary = readtable(fullfile(child.outputDir,'summary.csv'));
    verifyEqual(testCase,height(summary),numel(result.runs));
    signals = readtable(fullfile(child.outputDir,[result.runs(1).name,'.csv']));
    common = {'time_s','x_t_m','dx_t_mps','motion_error_m','force_N', ...
        'force_command_N','normal_angle_rad','tau1_Nm','tau2_Nm'};
    verifyEqual(testCase,signals.Properties.VariableNames(1:9),common);
    verifyEqual(testCase,signals.force_N,result.runs(1).force,'AbsTol',1e-12);
end
end

function testDesiredReferenceDerivatives(testCase)
% Check that dpd and ddpd represent derivatives of pd for the tracking case.
run = testCase.TestData.fic.runs(3);
pd = run.reference; dpd = run.referenceVelocity; ddpd = run.referenceAcceleration;
timeStep = run.t(2)-run.t(1);
dpdNumeric = (pd(3:end,:)-pd(1:end-2,:))/(2*timeStep);
ddpdNumeric = (dpd(3:end,:)-dpd(1:end-2,:))/(2*timeStep);
verifyEqual(testCase,dpd(2:end-1,:),dpdNumeric,'AbsTol',1e-7);
verifyEqual(testCase,ddpd(2:end-1,:),ddpdNumeric,'AbsTol',1e-7);
end

function testInvalidSharedOptions(testCase)
for runner = {@fic_simulation,@nac1_simulation,@iac1_simulation}
    verifyError(testCase,@() runner{1}(1),'HMFC:Options');
    verifyError(testCase,@() runner{1}(struct('saveResults',[])),'HMFC:Options');
    verifyError(testCase,@() runner{1}(struct('unknown',true)),'HMFC:Options');
end
end

function remove_output(root)
if isfolder(root), rmdir(root,'s'); end
end
