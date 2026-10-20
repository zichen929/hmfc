function results = fic1234_simulation(makePlots, outputDir)
%FIC1234_SIMULATION Run FIC 1, 2, 3, 4 with individual and four-column results.
%   results = fic1234_simulation() displays and saves all sixteen cases.
%   results = fic1234_simulation(false) disables plots.
%   results = fic1234_simulation(true, outputDir) selects the output root.
%   Outputs: fic1/, fic2/, fic3/, fic4/, comparison/, and fic1234_results.mat.
%   All six controller pairs are checked for all four matched cases.
if nargin < 1, makePlots = true; end
if nargin < 2
    outputDir = fullfile(fileparts(mfilename('fullpath')), 'results', 'fic1234');
end
results = fic_compare_simulation(1:4, makePlots, outputDir);
end
