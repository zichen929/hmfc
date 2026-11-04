function runs = hmfc_append_runs(runs,additional)
%HMFC_APPEND_RUNS Collect independent runs, retaining optional diagnostics.
if isempty(runs), runs = additional(:); return; end
names = union(fieldnames(runs),fieldnames(additional),'stable');
for k = 1:numel(names)
    key = names{k};
    if ~isfield(runs,key), [runs.(key)] = deal([]); end
    if ~isfield(additional,key), [additional.(key)] = deal([]); end
end
runs = [orderfields(runs,names);orderfields(additional(:),names)];
end
