function options = hmfc_simulation_options(options,defaults)
%HMFC_SIMULATION_OPTIONS Validate shared runner options before any side effects.
assert(isstruct(options) && isscalar(options),'HMFC:Options', ...
    'options must be a scalar struct.');
for name = fieldnames(options)'
    key = name{1};
    assert(isfield(defaults,key),'HMFC:Options','Unknown option: %s',key);
    defaults.(key) = options.(key);
end
options = defaults;
for name = {'makePlots','saveResults'}
    key = name{1};
    value = options.(key);
    assert((islogical(value) || isnumeric(value)) && isreal(value) ...
        && isscalar(value) && ismember(value,[0,1]),'HMFC:Options', ...
        '%s must be true or false.',key);
    options.(key) = logical(value);
end
assert((ischar(options.visible) && isrow(options.visible)) ...
    || (isstring(options.visible) && isscalar(options.visible)), ...
    'HMFC:Options','visible must be on or off.');
options.visible = validatestring(options.visible,{'on','off'});
assert((ischar(options.outputDir) && (isrow(options.outputDir) || isempty(options.outputDir))) ...
    || (isstring(options.outputDir) && isscalar(options.outputDir) ...
    && ~ismissing(options.outputDir)), 'HMFC:Options','outputDir must be a path.');
options.outputDir = char(options.outputDir);
if isfield(options,'duration')
    validateattributes(options.duration,{'numeric'},{'real','finite','scalar','>=',6});
    validateattributes(options.initialVelocity,{'numeric'},{'real','finite','scalar'});
end
end
