function projectRoot = hmfc_project_root()
%HMFC_PROJECT_ROOT Resolve the repository root independently of pwd.
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
