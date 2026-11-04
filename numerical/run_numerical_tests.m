function results = run_numerical_tests()
%RUN_NUMERICAL_TESTS Run all numerical and project-layout checks from any folder.
numericalRoot = setup_numerical();
results = runtests(fullfile(numericalRoot,'tests'));
assertSuccess(results);
end
