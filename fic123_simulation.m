function results = fic123_simulation(makePlots, outputDir)
%FIC123_SIMULATION Run FIC 1, 2, 3 with individual and three-column results.
%   results = fic123_simulation() displays and saves all twelve cases.
%   results = fic123_simulation(false) disables plots.
%   results = fic123_simulation(true, outputDir) selects the output root.
%   Outputs: fic1/, fic2/, fic3/, comparison/, and fic123_results.mat.
%   Each matched case has three columns; all three controller pairs are checked.
if nargin < 1, makePlots = true; end
if nargin < 2
    outputDir = fullfile(fileparts(mfilename('fullpath')), 'results', 'fic123');
end
results = fic_compare_simulation([1,2,3],makePlots,outputDir);
end
