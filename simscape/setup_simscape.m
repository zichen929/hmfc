function simscapeRoot = setup_simscape()
%SETUP_SIMSCAPE Initialize physical sources and their numerical dependencies.
%   Does not load a model, require Simscape, or change the current folder.
%   When Simulink is available, its cache and generated code remain here.
simscapeRoot = fileparts(mfilename('fullpath'));
projectRoot = fileparts(simscapeRoot);
addpath(simscapeRoot,fullfile(projectRoot,'numerical'));
setup_numerical();
if ~isempty(which('Simulink.fileGenControl'))
    Simulink.fileGenControl('set', ...
        'CacheFolder',fullfile(simscapeRoot,'cache'), ...
        'CodeGenFolder',fullfile(simscapeRoot,'codegen'), ...
        'createDir',true);
end
end
