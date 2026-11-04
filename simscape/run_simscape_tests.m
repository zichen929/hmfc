function results = run_simscape_tests()
%RUN_SIMSCAPE_TESTS Check setup and run the four-case physical ODE comparison.
%   The physical test requires Simulink, Simscape and Simscape Multibody.
simscapeRoot = setup_simscape();
results = runtests(fullfile(simscapeRoot,'tests'));
assertSuccess(results);
end
