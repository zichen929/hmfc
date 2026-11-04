function hmfc_latex_figure(fig)
%HMFC_LATEX_FIGURE Use LaTeX for existing text and future figure children.
%   Apply before creating axes so intermediate layout updates also use LaTeX.
%   Reapply before export to include reference-line labels and other charts.
set(fig,'DefaultTextInterpreter','latex', ...
    'DefaultAxesTickLabelInterpreter','latex','DefaultLegendInterpreter','latex');
set(findall(fig,'-property','Interpreter'),'Interpreter','latex');
set(findall(fig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex');
end
