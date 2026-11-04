function numericalRoot = setup_numerical()
%SETUP_NUMERICAL Initialize the base-MATLAB numerical simulations from any folder.
%   Adds only numerical sources and shared helpers; does not change pwd.
numericalRoot = fileparts(mfilename('fullpath'));
addpath(numericalRoot,fullfile(numericalRoot,'shared'));
end
