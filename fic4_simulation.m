function results = fic4_simulation(makePlots, outputDir)
%FIC4_SIMULATION Run FIC 4: Cartesian references with intrinsic-error feedback.
%   The controller converts pd, dpd, ddpd to xtd, dxtd, ddxtd and then uses
%   scalar intrinsic errors, as specified in TCST26E3.md.
%   results = fic4_simulation() displays and saves the FIC 4 results.
%   results = fic4_simulation(false) disables plots.
%   results = fic4_simulation(true, outputDir) selects the output directory.
%   Run fic1234_simulation for all four controllers and comparison figures.
if nargin < 1, makePlots = true; end
if nargin < 2
    outputDir = fullfile(fileparts(mfilename('fullpath')), 'results', 'fic4');
end
results = fic_simulation(4, makePlots, outputDir);
end
