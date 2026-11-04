function tests = test_project_setup
%TEST_PROJECT_SETUP Verify repository and numerical setup from any folder.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
numericalRoot = fileparts(fileparts(mfilename('fullpath')));
root = fileparts(numericalRoot);
testCase.TestData.root = root;
testCase.TestData.numericalRoot = numericalRoot;
testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
    {root,numericalRoot,fullfile(numericalRoot,'shared')}));
end

function testInitializationIsIndependentOfWorkingFolder(testCase)
oldPath = path;
cleanup = onCleanup(@() path(oldPath));
testCase.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
workingFolder = pwd;
root = setup_project();
verifyEqual(testCase,root,testCase.TestData.root);
verifyEqual(testCase,pwd,workingFolder);
verifyEqual(testCase,which('fic_simulation'), ...
    fullfile(root,'numerical','fic_simulation.m'));
verifyEqual(testCase,which('fic1_simscape_simulation'), ...
    fullfile(root,'simscape','fic1_simscape_simulation.m'));
verifyEqual(testCase,which('hmfc_parameters'), ...
    fullfile(root,'numerical','shared','hmfc_parameters.m'));
verifyEqual(testCase,hmfc_project_root(),root);
verifyEqual(testCase,hmfc_numerical_root(),testCase.TestData.numericalRoot);
initializedPath = path;
setup_project();
verifyEqual(testCase,path,initializedPath);
end

function testNumericalInitializationAndPaths(testCase)
oldPath = path;
cleanup = onCleanup(@() path(oldPath));
testCase.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
workingFolder = pwd;
previousFolders = string(strsplit(path,pathsep));
numericalRoot = setup_numerical();
verifyEqual(testCase,numericalRoot,testCase.TestData.numericalRoot);
verifyEqual(testCase,pwd,workingFolder);
initializedPath = path;
setup_numerical();
verifyEqual(testCase,path,initializedPath);
folders = setdiff(string(strsplit(path,pathsep)),previousFolders);
for excluded = {'tests','docs','results'}
    verifyFalse(testCase,any(folders==string(fullfile(numericalRoot,excluded{1}))));
end
verifyEqual(testCase,which('run_numerical_tests'), ...
    fullfile(numericalRoot,'run_numerical_tests.m'));
end

function testSharedInitialStateMatchesContactPoint(testCase)
c = hmfc_parameters();
q1 = c.q0(1); q2 = c.q0(2);
position = [c.l1*cos(q1)+c.l2*cos(q1+q2); ...
            c.l1*sin(q1)+c.l2*sin(q1+q2)];
verifyEqual(testCase,position,c.p0,'AbsTol',1e-14);
verifyEqual(testCase,c.setpoint,c.p0+c.T*c.targetX);
verifyEqual(testCase,c.n'*c.T,0,'AbsTol',1e-14);
verifyEqual(testCase,norm(c.n),1,'AbsTol',1e-14);
verifyEqual(testCase,norm(c.T),1,'AbsTol',1e-14);
end
