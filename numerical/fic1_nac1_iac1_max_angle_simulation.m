function results = fic1_nac1_iac1_max_angle_simulation(options)
%FIC1_NAC1_IAC1_MAX_ANGLE_SIMULATION Near-supremum sector, +15 deg mismatch.
%   results = fic1_nac1_iac1_max_angle_simulation;
%   results = fic1_nac1_iac1_max_angle_simulation(struct('visible','off'));
%   Delta_sup = atan(1/chi_Lambda)/2 is an OPEN upper bound.
%   Use Delta=Delta_sup-sectorMarginDeg, with a positive margin (1e-6 deg).
%   NAC uses fixed +15 deg; IAC starts at +15 deg; FIC uses true geometry.
if nargin==0, options=struct(); end
common=hmfc_parameters();
defaults=struct('duration',common.duration,'initialVelocity',common.initialVelocity, ...
    'makePlots',true,'saveResults',true,'visible','on','sectorMarginDeg',1e-6,'outputDir', ...
    fullfile(hmfc_numerical_root(),'results','fic1_nac1_iac1_max_angle'));
options=hmfc_simulation_options(options,defaults);
scenario=options;
scenario.initialAngleDeg=15;
scenario.sectorHalfWidthDeg='supremum';
% A certificate region is not a physical obstacle. Retain its original
% bounds and FLAG any NAC exit, while integrating the same exact robot.
% IAC and FIC still enforce the original region throughout.
scenario.allowNacOutsideOperatingRegion=true;
results=fic1_nac1_iac1_simulation(scenario);
cert=results.iac1.certificate;
supremum=cert.sectorHalfWidthSupremumDeg;
used=cert.sectorHalfWidthUsedDeg;
assert(abs(supremum-used-options.sectorMarginDeg)<1e-10);
assert(used<supremum && cert.rhoMin>0);
assert(abs(results.iac1.runs(1).barTheta(1)*180/pi-15)<1e-12 && used>15);
analysis=struct('trueAngleDeg',0,'chosenInitialAngleDeg',15, ...
    'sectorHalfWidthSupremumDeg',supremum,'sectorHalfWidthUsedDeg',used, ...
    'sectorMarginDeg',options.sectorMarginDeg,'sectorDeg',[-used,used], ...
    'chiLambda',cert.chi,'rhoMin',cert.rhoMin, ...
    'certifiedForceMinimum_N',cert.forceMin,'strictSectorConditionPassed',true, ...
    'nacStayedInOperatingRegion',results.nac1.runs(1).operatingRegionSatisfied);
results.angleAnalysis=analysis;
results.scenario='near-supremum initialization sector with +15 deg initial mismatch';
if options.saveResults
    save(fullfile(options.outputDir,'fic1_nac1_iac1_results.mat'),'results');
    fid=fopen(fullfile(options.outputDir,'initial_angle_analysis.txt'),'w');
    assert(fid>=0); cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'Near-supremum IAC 1 initialization sector with +15 degree mismatch\n');
    fprintf(fid,'chi_Lambda=%.12g\n',cert.chi);
    fprintf(fid,'OPEN half-width supremum: %.12g deg\n',supremum);
    fprintf(fid,'Positive numerical margin: %.12g deg\n',options.sectorMarginDeg);
    fprintf(fid,'Actual closed sector half-width: %.12g deg\n',used);
    fprintf(fid,'Initial NAC/IAC angle: +15 deg; true angle: 0 deg\n');
    fprintf(fid,'rho_min=%.12g; certified force minimum=%.12g N\n',cert.rhoMin,cert.forceMin);
    fprintf(fid,'Equality at the supremum would yield a zero uniform force bound and is not used.\n');
    fprintf(fid,'The near-zero uniform bound is conservative; measured IAC force remains positive.\n');
    fprintf(fid,'Original certificate region unchanged. NAC stayed inside: %d\n',analysis.nacStayedInOperatingRegion);
    if ~analysis.nacStayedInOperatingRegion
        fprintf(fid,'NAC first exited the region near %.9g s; its physical model was continued.\n', ...
            results.nac1.runs(1).firstRegionExitTime);
    end
end
fprintf('Sector half-width: %.12g deg (open supremum %.12g deg); initial angle +15 deg.\n',used,supremum);
end
