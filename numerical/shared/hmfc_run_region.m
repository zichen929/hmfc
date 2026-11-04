function run = hmfc_run_region(run,plant)
%HMFC_RUN_REGION Report the original certificate region on the output grid.
inside = run.state(:,1)>plant.xMin & run.state(:,1)<plant.xMax ...
    & run.q(:,2)>plant.q2Min & run.q(:,2)<plant.q2Max;
run.operatingRegionSatisfied = all(inside);
run.firstRegionExitTime = NaN;
firstOutside = find(~inside,1);
if ~isempty(firstOutside), run.firstRegionExitTime = run.t(firstOutside); end
end
