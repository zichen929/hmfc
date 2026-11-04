function projectRoot = setup_project()
%SETUP_PROJECT Initialize HMFC numerical and physical simulation sources.
%   Opening hmfc.prj runs this function. It preserves pwd and is repeatable.
%   For one simulation family use setup_numerical or setup_simscape.
%   Initialization does not require the optional Simscape toolboxes.
projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot,fullfile(projectRoot,'simscape'));
setup_simscape();
end
