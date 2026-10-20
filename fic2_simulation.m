function results = fic2_simulation(makePlots, outputDir)
%FIC2_SIMULATION Run FIC 2 with intrinsic references on the rigid-contact arm.
%   The controller converts xtd, dxtd, ddxtd to pd, dpd, ddpd before applying
%   the Cartesian feedback law, as specified in TCST26E3-EN.md.
%   results = fic2_simulation() displays and saves the FIC 2 results.
%   results = fic2_simulation(false) disables plots.
%   results = fic2_simulation(true, outputDir) selects the output directory.
if nargin < 1, makePlots = true; end
if nargin < 2
    outputDir = fullfile(fileparts(mfilename('fullpath')), 'results', 'fic2');
end
results = fic_simulation(2, makePlots, outputDir);
end
