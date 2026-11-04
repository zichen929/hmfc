function summary = hmfc_run_summary(runs)
%HMFC_RUN_SUMMARY Common summary columns followed by controller-specific metrics.
summary = struct2table([runs.metrics]);
summary = addvars(summary,string({runs.controller})',string({runs.name})', ...
    string({runs.mode})',[runs.kf]','Before',1, ...
    'NewVariableNames',{'controller','name','mode','kf'});
end
