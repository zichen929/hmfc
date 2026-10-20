function results = fic3_simulation(makePlots, outputDir)
%FIC3_SIMULATION Run FIC 3 intrinsic-error feedback on the rigid-contact arm.
%   results = fic3_simulation() displays and saves the FIC 3 results.
%   results = fic3_simulation(false) disables plots.
%   results = fic3_simulation(true, outputDir) selects the output directory.
%   Run fic123_simulation for all three controllers and comparison figures.
if nargin < 1, makePlots = true; end
if nargin < 2
    outputDir = fullfile(fileparts(mfilename('fullpath')), 'results', 'fic3');
end
results = fic_simulation(3, makePlots, outputDir);
end
