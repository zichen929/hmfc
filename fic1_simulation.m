function results = fic1_simulation(makePlots, outputDir)
%FIC1_SIMULATION Run FIC 1 with Cartesian references on the rigid-contact arm.
%   results = fic1_simulation() displays and saves the FIC 1 results.
%   results = fic1_simulation(false) disables plots.
%   results = fic1_simulation(true, outputDir) selects the output directory.
%   Run fic12_simulation to generate both controllers and comparison figures.
if nargin < 1, makePlots = true; end
if nargin < 2
    outputDir = fullfile(fileparts(mfilename('fullpath')), 'results', 'fic1');
end
results = fic_simulation(1, makePlots, outputDir);
end
