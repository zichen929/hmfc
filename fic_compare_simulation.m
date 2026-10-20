function results = fic_compare_simulation(controllerIds,makePlots,outputDir)
%FIC_COMPARE_SIMULATION Common runner for two to four controllers.
%   Use fic12_simulation, fic123_simulation, or fic1234_simulation.
if ~exist(outputDir,'dir'), mkdir(outputDir); end
comparisonDir = fullfile(outputDir,'comparison');
if ~exist(comparisonDir,'dir'), mkdir(comparisonDir); end
controllerResults = cell(1,numel(controllerIds));
results.allChecksPassed = true;
for controllerIndex = 1:numel(controllerIds)
    controllerId = controllerIds(controllerIndex);
    % Every controller performs its own integration on the same physical plant.
    run = fic_simulation(controllerId,makePlots, ...
        fullfile(outputDir,sprintf('fic%d',controllerId)));
    controllerResults{controllerIndex} = run;
    results.(sprintf('fic%d',controllerId)) = run;
    results.allChecksPassed = results.allChecksPassed && run.allChecksPassed;
    controllerSummary = addvars(run.summary, ...
        repmat(string(run.controller),height(run.summary),1), ...
        'Before',1,'NewVariableNames','controller');
    if controllerIndex == 1
        results.summary = controllerSummary;
    else
        results.summary = [results.summary;controllerSummary];
    end
end

pairs = nchoosek(1:numel(controllerIds),2);
for caseIndex = 1:numel(controllerResults{1}.cases)
    matchedCases = cell(1,numel(controllerIds));
    for controllerIndex = 1:numel(controllerIds)
        assert(numel(controllerResults{controllerIndex}.cases) ...
            == numel(controllerResults{1}.cases), ...
            'FIC:ComparisonCases','The controllers produced different case counts.');
        matchedCases{controllerIndex} = controllerResults{controllerIndex}.cases(caseIndex);
    end
    for pairIndex = 1:size(pairs,1)
        a = matchedCases{pairs(pairIndex,1)};
        b = matchedCases{pairs(pairIndex,2)};
        assert(strcmp(a.mode,b.mode) && a.kf == b.kf && isequal(a.t,b.t), ...
            'FIC:ComparisonInputs','Compared cases must share mode, gain, and times.');
        comparison = compareCase(a,b);
        if ~isfield(results,'comparison')
            results.comparison = comparison;
        else
            results.comparison(end+1) = comparison;
        end
        fprintf('%s vs %s | %-10s kf=%g: max |delta p|=%.3e m, max |delta tau|=%.3e N m\n', ...
            a.controller,b.controller,a.mode,a.kf, ...
            comparison.maxPositionDifference,comparison.maxTorqueDifference);
    end
    if makePlots
        plotSideBySide(matchedCases,controllerResults{1}.config,comparisonDir);
    end
end
results.comparisonSummary = struct2table(results.comparison);
writetable(results.comparisonSummary,fullfile(comparisonDir,'summary.csv'));
writetable(results.summary,fullfile(outputDir,'summary.csv'));
if makePlots
    for pairIndex = 1:size(pairs,1)
        if size(pairs,1) == 1
            name = 'equivalence';
        else
            name = sprintf('equivalence_%d_%d', ...
                controllerIds(pairs(pairIndex,1)),controllerIds(pairs(pairIndex,2)));
        end
        plotEquivalence(controllerResults{pairs(pairIndex,1)}.cases, ...
            controllerResults{pairs(pairIndex,2)}.cases,comparisonDir,name);
    end
end
fileTag = ['fic',sprintf('%d',controllerIds)];
save(fullfile(outputDir,[fileTag,'_results.mat']),'results');
fprintf('All %d controllers and pairwise equivalence checks passed. Results: %s\n', ...
    numel(controllerIds),outputDir);
end

function comparison = compareCase(a,b)
comparison.controllerA = a.controller;
comparison.controllerB = b.controller;
comparison.mode = a.mode;
comparison.kf = a.kf;
comparison.maxReferencePositionDifference = max(vecnorm(a.pd-b.pd,2,2));
comparison.maxReferenceVelocityDifference = max(vecnorm(a.dpd-b.dpd,2,2));
comparison.maxStateDifference = max(vecnorm(a.z-b.z,2,2));
comparison.maxPositionDifference = max(vecnorm(a.p-b.p,2,2));
comparison.maxVelocityDifference = max(vecnorm(a.dp-b.dp,2,2));
comparison.maxAccelerationDifference = max(vecnorm(a.pdd-b.pdd,2,2));
comparison.maxCommandDifference = max(vecnorm(a.up-b.up,2,2));
comparison.maxForceCommandDifference = max(abs(a.uf-b.uf));
comparison.maxForceDifference = max(abs(a.f-b.f));
comparison.maxTorqueDifference = max(vecnorm(a.tau-b.tau,2,2));
residuals = [comparison.maxReferencePositionDifference, ...
    comparison.maxReferenceVelocityDifference,comparison.maxStateDifference, ...
    comparison.maxPositionDifference,comparison.maxVelocityDifference, ...
    comparison.maxAccelerationDifference,comparison.maxCommandDifference, ...
    comparison.maxForceCommandDifference,comparison.maxForceDifference, ...
    comparison.maxTorqueDifference];
assert(all(isfinite(residuals)) && all(residuals < 1e-7), ...
    'FIC:Equivalence','Compared controllers differ beyond numerical tolerance.');
end

function plotSideBySide(cases,c,outputDir)
a = cases{1};
comparisonTitle = strjoin(cellfun(@(r) r.controller,cases,'UniformOutput',false),' vs ');
columnCount = numel(cases);
fig = figure('Name',sprintf('%s - %s - kf=%g',comparisonTitle,a.mode,a.kf), ...
    'Color','w','Position',[100,60,640*columnCount,1100]);
if isprop(fig,'Theme'), set(fig,'Theme','light'); end
tl = tiledlayout(fig,4,columnCount,'TileSpacing','compact','Padding','compact');
title(tl,sprintf('%s | %s | k_f=%g | identical plant, gains, and reference',comparisonTitle,a.mode,a.kf));
axesHandles = gobjects(4,columnCount);
for column = 1:columnCount
    r = cases{column};
    axesHandles(1,column) = nexttile(column);
    plot(r.t,(r.pd-c.p0')*c.T,'k--',r.t,r.z(:,1),'b-','LineWidth',1.4);
    grid on; xlabel('t (s)'); ylabel('x_t (m)');
    title([r.controller,' | Tangential position']);
    legend('Reference','Actual','Location','best');

    axesHandles(2,column) = nexttile(columnCount+column);
    semilogy(r.t,max(vecnorm(r.ep,2,2),1e-16),'b-', ...
        r.t,max(vecnorm(r.exactEp,2,2),1e-16),'k--','LineWidth',1.2);
    grid on; xlabel('t (s)'); ylabel('||e_p|| (m)');
    title([r.controller,' | Position error']);
    legend('ODE','Analytical','Location','best');

    axesHandles(3,column) = nexttile(2*columnCount+column);
    plot(r.t,r.f,'b-','LineWidth',1.4); hold on;
    yline(c.fd,'k--'); ylim([c.fd-1,c.fd+1]);
    grid on; xlabel('t (s)'); ylabel('f (N)');
    title([r.controller,' | Normal contact force']);
    legend('Actual','Desired','Location','best');

    axesHandles(4,column) = nexttile(3*columnCount+column);
    plot(r.t,r.tau(:,1),'b-',r.t,r.tau(:,2),'r-','LineWidth',1.2);
    grid on; xlabel('t (s)'); ylabel('Joint torque (N m)');
    title([r.controller,' | Applied torques']);
    legend('\tau_1','\tau_2','Location','best');
end
% Matched rows share limits so the visual comparison uses the same scales.
for row = 1:4
    limits = zeros(columnCount,2);
    for column = 1:columnCount
        limits(column,:) = ylim(axesHandles(row,column));
    end
    sharedLimits = [min(limits(:,1)),max(limits(:,2))];
    linkaxes(axesHandles(row,:),'xy');
    ylim(axesHandles(row,1),sharedLimits);
end
name = sprintf('%s_kf_%g',a.mode,a.kf);
exportgraphics(fig,fullfile(outputDir,[name,'.png']),'Resolution',160);
savefig(fig,fullfile(outputDir,[name,'.fig']));
end

function plotEquivalence(fic1Cases,fic2Cases,outputDir,name)
comparisonTitle = [fic1Cases(1).controller,' vs ',fic2Cases(1).controller];
fig = figure('Name',[comparisonTitle,' - numerical differences'], ...
    'Color','w','Position',[120,80,1250,850]);
if isprop(fig,'Theme'), set(fig,'Theme','light'); end
tl = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
title(tl,[comparisonTitle,' | numerical differences (display floor: 10^{-16})']);
labels = cell(1,numel(fic1Cases));
for i = 1:numel(fic1Cases)
    a = fic1Cases(i); b = fic2Cases(i);
    labels{i} = sprintf('%s, k_f=%g',a.mode,a.kf);
    series = {vecnorm(a.p-b.p,2,2),vecnorm(a.up-b.up,2,2), ...
        abs(a.f-b.f),vecnorm(a.tau-b.tau,2,2)};
    for panel = 1:4
        nexttile(panel); hold on;
        semilogy(a.t,max(series{panel},1e-16),'LineWidth',1.2);
        set(gca,'YScale','log'); ylim([1e-16,1e-6]);
        grid on; xlabel('t (s)');
    end
end
titles = {'Position difference norm (m)','Acceleration command difference norm (m/s^2)', ...
    'Contact force difference (N)','Joint torque difference norm (N m)'};
for panel = 1:4
    nexttile(panel); title(titles{panel});
    yline(1e-7,'k--','Check tolerance');
end
nexttile(1); legend(labels,'Location','best');
exportgraphics(fig,fullfile(outputDir,[name,'.png']),'Resolution',160);
savefig(fig,fullfile(outputDir,[name,'.fig']));
end
