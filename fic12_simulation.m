function results = fic12_simulation(makePlots, outputDir)
%FIC12_SIMULATION Run FIC 1 and FIC 2 with individual and side-by-side results.
%   The original two-controller entry point and output layout are preserved.
%   Run fic123_simulation to include FIC 3.
if nargin < 1, makePlots = true; end
if nargin < 2
    outputDir = fullfile(fileparts(mfilename('fullpath')), 'results', 'fic12');
end
results = fic_compare_simulation([1,2],makePlots,outputDir);
end
