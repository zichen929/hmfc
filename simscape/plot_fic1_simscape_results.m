function plot_fic1_simscape_results(results,baseline)
%PLOT_FIC1_SIMSCAPE_RESULTS Plot recorded physical results without rerunning.
% plot_fic1_simscape_results(results) loads the saved ODE benchmark.
if nargin<2
    data=load(fullfile(results.options.outputDir,'ode_reference','fic1_results.mat'),'results');
    baseline=data.results;
end
for im=1:2
    fig=figure('Visible',results.options.visible,'Color','w','Position',[100,100,1100,800]);
    tiledlayout(3,2);
    a=results.cases(2*im-1); ref=baseline.runs(2*im-1);
    nexttile; plot(a.t,a.p(:,1),'b',a.t,a.pd(:,1),'k--',ref.t,ref.position(:,1),'r:');
    ylabel('x (m)'); legend('Simscape','Reference','ODE'); grid on;
    nexttile; plot(a.t,a.ep); ylabel('Position error (m)'); legend('x','y'); grid on;
    nexttile; hold on;
    for k=2*im-1:2*im
        r=results.cases(k); plot(r.t,r.f,'DisplayName',sprintf('kf=%g',r.kf));
    end
    yline(results.config.fd,'k--','DisplayName','Desired');
    ylim(results.config.fd+[-0.5,0.5]); ylabel('Reaction force (N)'); legend; grid on;
    nexttile; plot(a.t,a.tau); ylabel('Joint torque (N m)'); legend('1','2'); grid on;
    nexttile; plot(a.t,a.p-ref.position); ylabel('Simscape - ODE (m)'); xlabel('Time (s)'); grid on;
    nexttile; plot(a.t,a.dynamicResidual); ylabel('Joint residual (N m)'); xlabel('Time (s)'); grid on;
    sgtitle(sprintf('Native Simscape FIC1: %s',a.mode));
    exportgraphics(fig,fullfile(results.options.outputDir,[a.mode,'.png']),'Resolution',160);
    savefig(fig,fullfile(results.options.outputDir,[a.mode,'.fig']));
    if strcmp(results.options.visible,'off'), close(fig); end
end
end
