function numericalRoot = hmfc_numerical_root()
%HMFC_NUMERICAL_ROOT Resolve numerical sources, docs, tests and result root.
numericalRoot = fileparts(fileparts(mfilename('fullpath')));
end
