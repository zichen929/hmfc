function tests = test_simscape_project
%TEST_SIMSCAPE_PROJECT Check relocated setup, results and physical comparison.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
simscapeRoot = fileparts(fileparts(mfilename('fullpath')));
root = fileparts(simscapeRoot);
testCase.TestData.root = root;
testCase.TestData.simscapeRoot = simscapeRoot;
testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
    {simscapeRoot,fullfile(root,'numerical'),fullfile(root,'numerical','shared')}));
end

function testInitializationFromAnyFolder(testCase)
oldPath = path;
cleanup = onCleanup(@() path(oldPath));
testCase.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
workingFolder = pwd;
previousFolders = string(strsplit(path,pathsep));
simscapeRoot = setup_simscape();
verifyEqual(testCase,simscapeRoot,testCase.TestData.simscapeRoot);
verifyEqual(testCase,hmfc_simscape_root(),simscapeRoot);
verifyEqual(testCase,pwd,workingFolder);
verifyEqual(testCase,which('fic1_simscape_simulation'), ...
    fullfile(simscapeRoot,'fic1_simscape_simulation.m'));
verifyEqual(testCase,which('fic_simulation'), ...
    fullfile(testCase.TestData.root,'numerical','fic_simulation.m'));
initializedPath = path;
setup_simscape();
verifyEqual(testCase,path,initializedPath);
newFolders = setdiff(string(strsplit(path,pathsep)),previousFolders);
for excluded = {'tests','docs','results','cache','codegen'}
    verifyFalse(testCase,any(newFolders==string(fullfile(simscapeRoot,excluded{1}))));
end
if ~isempty(which('Simulink.fileGenControl'))
    config = Simulink.fileGenControl('getConfig');
    verifyEqual(testCase,char(config.CacheFolder),fullfile(simscapeRoot,'cache'));
    verifyEqual(testCase,char(config.CodeGenFolder),fullfile(simscapeRoot,'codegen'));
end
end

function testPhysicalComparisonAndSavedPlots(testCase)
assumeFalse(testCase,isempty(which('sm_lib')),'Simscape Multibody is not installed.');
assumeFalse(testCase,bdIsLoaded('hmfc_fic1_simscape'), ...
    'Close the existing physical model before running the integration test.');
testCase.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
outputDir = tempname;
cleanup = onCleanup(@() remove_test_outputs(outputDir));
setup_simscape();
result = fic1_simscape_simulation(struct('visible','off','outputDir',outputDir));
verifyTrue(testCase,result.allChecksPassed);
verifyEqual(testCase,numel(result.cases),4);
verifyTrue(testCase,isfield(result.cases,'ddq'));
verifyFalse(testCase,isfield(result.cases,'qdd'));
verifyEqual(testCase,result.modelFile, ...
    fullfile(testCase.TestData.simscapeRoot,'hmfc_fic1_simscape.slx'));
verifyTrue(testCase,all(arrayfun(@(r) r.checks.passed,result.cases)));
for name = {'fic1_simscape_results.mat','summary.csv','verification.txt', ...
        'regulation.png','regulation.fig','tracking.png','tracking.fig', ...
        fullfile('ode_reference','fic1_results.mat')}
    verifyTrue(testCase,isfile(fullfile(outputDir,name{1})));
end
signals = readtable(fullfile(outputDir,'regulation_kf0.csv'));
verifyTrue(testCase,all(ismember({'ddq1_radps2','ddq2_radps2','pxd_m','pyd_m'}, ...
    signals.Properties.VariableNames)));
verifyEqual(testCase,get_param('hmfc_fic1_simscape/ddq log','VariableName'),'ddq_log');
saved = load(fullfile(outputDir,'fic1_simscape_results.mat'));
verifyEqual(testCase,saved.results,result);
checks = [result.cases.checks];
verifyLessThan(testCase,max([checks.maxPositionVsODE_m]),2e-6);
end

function remove_test_outputs(outputDir)
if bdIsLoaded('hmfc_fic1_simscape'), close_system('hmfc_fic1_simscape',0); end
if isfolder(outputDir), rmdir(outputDir,'s'); end
end
