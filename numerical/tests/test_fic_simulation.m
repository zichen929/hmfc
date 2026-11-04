function tests = test_fic_simulation
%TEST_FIC_SIMULATION Exercise controller selection and saved result layouts.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
    {root,fullfile(root,'shared')}));
testCase.TestData.outputRoot = tempname;
mkdir(testCase.TestData.outputRoot);
end

function teardownOnce(testCase)
rmdir(testCase.TestData.outputRoot,'s');
end

function testAllControllers(testCase)
outputDir = fullfile(testCase.TestData.outputRoot,'all');
results = fic_simulation(struct('controllerIds',1:4,'makePlots',false,'outputDir',outputDir));
verifyTrue(testCase,results.allChecksPassed);
verifyEqual(testCase,height(results.summary),16);
verifyEqual(testCase,numel(results.runs),16);
verifyEqual(testCase,height(results.comparisonSummary),24);
for id = 1:4
    name = sprintf('fic%d',id);
    run = results.(name);
    verifyEqual(testCase,run.controller.id,id);
    verifyEqual(testCase,numel(run.runs),4);
    verifyTrue(testCase,run.allChecksPassed);
    verifyTrue(testCase,isfile(fullfile(outputDir,name,[name,'_results.mat'])));
end
verifyTrue(testCase,isfile(fullfile(outputDir,'comparison','summary.csv')));
saved = load(fullfile(outputDir,'fic1234_results.mat'));
verifyEqual(testCase,saved.results,results);
verifyEmpty(testCase,dir(fullfile(outputDir,'**','*.png')));
end

function testNonconsecutiveColumnSelection(testCase)
outputDir = fullfile(testCase.TestData.outputRoot,'subset');
results = fic_simulation(struct('controllerIds',[4;2],'makePlots',false,'outputDir',outputDir));
verifyTrue(testCase,results.allChecksPassed);
verifyTrue(testCase,isfield(results,'fic4') && isfield(results,'fic2'));
verifyFalse(testCase,isfield(results,'fic1') || isfield(results,'fic3'));
verifyFalse(testCase,isfolder(fullfile(outputDir,'fic1')));
verifyFalse(testCase,isfolder(fullfile(outputDir,'fic3')));
verifyEqual(testCase,height(results.summary),8);
verifyEqual(testCase,height(results.comparisonSummary),4);
verifyEqual(testCase,string(results.summary.controller), ...
    [repmat("FIC 4",4,1);repmat("FIC 2",4,1)]);
verifyEqual(testCase,string(results.comparisonSummary.controllerA),repmat("FIC 4",4,1));
verifyEqual(testCase,string(results.comparisonSummary.controllerB),repmat("FIC 2",4,1));
verifyTrue(testCase,isfile(fullfile(outputDir,'fic42_results.mat')));
end

function testSingleController(testCase)
outputDir = fullfile(testCase.TestData.outputRoot,'single');
results = fic_simulation(struct('controllerIds',3,'makePlots',false,'outputDir',outputDir));
verifyTrue(testCase,results.allChecksPassed);
verifyEqual(testCase,results.controller.id,3);
verifyEqual(testCase,numel(results.runs),4);
verifyEqual(testCase,height(results.summary),4);
verifyFalse(testCase,isfield(results,'fic3') || isfield(results,'comparison'));
verifyFalse(testCase,isfolder(fullfile(outputDir,'comparison')));
saved = load(fullfile(outputDir,'fic3_results.mat'));
verifyEqual(testCase,saved.results,results);
end

function testInvalidSelectionsDoNotCreateOutputs(testCase)
outputDir = fullfile(testCase.TestData.outputRoot,'invalid');
selections = {[],0,5,1.5,[1 1],[1 5],NaN,Inf,1+1i,[1 2;3 4],'1',true};
for index = 1:numel(selections)
    verifyError(testCase,@() fic_simulation(struct('controllerIds',selections{index},'makePlots',false,'outputDir',outputDir)), ...
        'FIC:Controller');
end
verifyFalse(testCase,isfolder(outputDir));
end

function testInvalidPlotOptionsDoNotCreateOutputs(testCase)
outputDir = fullfile(testCase.TestData.outputRoot,'invalid-plots');
options = {[],[true false],2,NaN,1+1i,'off'};
for index = 1:numel(options)
    verifyError(testCase,@() fic_simulation(struct('controllerIds',1,'makePlots',options{index},'outputDir',outputDir)),'HMFC:Options');
end
verifyFalse(testCase,isfolder(outputDir));
end
