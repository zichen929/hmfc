function modelFile = build_hmfc_simscape(outputDir)
%BUILD_HMFC_SIMSCAPE Build native, torque-driven 2R Multibody FIC1 model.
% modelFile = build_hmfc_simscape(); then open_system(modelFile).
% No ODE, inverse kinematics, prescribed motion, or penalty contact in plant.
root=fileparts(mfilename('fullpath'));
if nargin==0, outputDir=root; end
assert(~isempty(which('sm_lib')),'HMFC:MissingMultibody', ...
    'Install Simscape and Simscape Multibody for this MATLAB release.');
if ~exist(outputDir,'dir'), mkdir(outputDir); end
mdl='hmfc_fic1_simscape';
if bdIsLoaded(mdl)
    assert(strcmp(get_param(mdl,'Dirty'),'off'),'Save or close the modified model before rebuilding.');
    close_system(mdl,0);
end
load_system('sm_lib'); load_system('nesl_utility');
new_system(mdl);
c=hmfc_parameters();
w=get_param(mdl,'ModelWorkspace');
assignin(w,'c',c); assignin(w,'mode',0); assignin(w,'kf',0);
set_param(mdl,'InitFcn', ...
    'assignin(get_param(bdroot,''ModelWorkspace''),''c'',hmfc_parameters());');
set_param(mdl,'Solver','ode15s','StopTime','c.duration','MaxStep','0.005', ...
    'RelTol','1e-9','AbsTol','1e-10','ReturnWorkspaceOutputs','on', ...
    'AlgebraicLoopSolver','LineSearch','AlgebraicLoopMsg','warning');
% Keep physical simulation independent from any prescribed trajectory.
plant=[mdl,'/2R Arm - Simscape Multibody'];
add_block('simulink/Ports & Subsystems/Subsystem',plant,'Position',[510,170,760,350]);
delete_line(plant,'In1/1','Out1/1');
delete_block([plant,'/In1']); delete_block([plant,'/Out1']);
build_plant(plant);
ctrl=[mdl,'/FIC1 Cartesian Controller'];
add_block('simulink/User-Defined Functions/MATLAB Function',ctrl,'Position',[230,180,410,320]);
chart=find(sfroot,'-isa','Stateflow.EMChart','Path',ctrl);
chart.Script=fileread(fullfile(root,'fic1_simscape_controller.m'));
for name={'c','mode','kf'}
    d=find(chart,'-isa','Stateflow.Data','Name',name{1}); d.Scope='Parameter';
end
add_block('simulink/Sources/Clock',[mdl,'/Time'],'Position',[60,160,90,190]);
add_block('simulink/Signal Routing/Selector',[mdl,'/Normal Force'], ...
    'Position',[820,375,875,415],'NumberOfDimensions','1', ...
    'IndexOptionArray',{'Index vector (dialog)'},'IndexParamArray',{'2'},'InputPortWidth','3');
line(mdl,'Time/1','FIC1 Cartesian Controller/1');
line(mdl,'FIC1 Cartesian Controller/1','2R Arm - Simscape Multibody/1');
line(mdl,'2R Arm - Simscape Multibody/1','FIC1 Cartesian Controller/2');
line(mdl,'2R Arm - Simscape Multibody/2','FIC1 Cartesian Controller/3');
line(mdl,'2R Arm - Simscape Multibody/4','Normal Force/1');
line(mdl,'Normal Force/1','FIC1 Cartesian Controller/4');
logs={'q','dq','ddq','force','p','pd','tau','uf'};
sources={'2R Arm - Simscape Multibody/1','2R Arm - Simscape Multibody/2', ...
    '2R Arm - Simscape Multibody/3','2R Arm - Simscape Multibody/4', ...
    'FIC1 Cartesian Controller/2','FIC1 Cartesian Controller/3', ...
    'FIC1 Cartesian Controller/1','FIC1 Cartesian Controller/4'};
for j=1:numel(logs)
    dest=[mdl,'/',logs{j},' log'];
    add_block('simulink/Sinks/To Workspace',dest,'Position',[1040,85+50*j,1165,110+50*j], ...
        'VariableName',[logs{j},'_log'],'SaveFormat','Timeseries','MaxDataPoints','inf');
    line(mdl,sources{j},[logs{j},' log/1']);
end
add_block('simulink/Sinks/Scope',[mdl,'/Motion Force Torque Scope'], ...
    'NumInputPorts','4','Position',[820,180,875,300]);
line(mdl,'FIC1 Cartesian Controller/2','Motion Force Torque Scope/1');
line(mdl,'FIC1 Cartesian Controller/3','Motion Force Torque Scope/2');
line(mdl,'Normal Force/1','Motion Force Torque Scope/3');
line(mdl,'FIC1 Cartesian Controller/1','Motion Force Torque Scope/4');
add_block('simulink/Logic and Bit Operations/Compare To Constant',[mdl,'/Compressive Contact'], ...
    'relop','>','const','0','Position',[820,470,920,510]);
add_block('simulink/Model Verification/Assertion',[mdl,'/Contact Must Stay Positive'], ...
    'Position',[965,470,1005,510]);
line(mdl,'Normal Force/1','Compressive Contact/1');
line(mdl,'Compressive Contact/1','Contact Must Stay Positive/1');
a=Simulink.Annotation(mdl,sprintf(['FIC1 / exact 2R arm / maintained rigid contact\n', ...
    'Model workspace: c = physical parameters; mode = 0 regulation / 1 tracking; kf = 0 or 2.\n', ...
    'Positive reaction is checked. Constraint is bilateral; separation and impacts are outside scope.']));
a.Position=[50,30];
set_param(mdl,'Location',[50,50,1400,800]);
Simulink.BlockDiagram.arrangeSystem(mdl);
% Multibody viewer is optional; headless simulation must not launch windows.
try
    set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off');
catch
    % Older Multibody releases can omit this optional viewer preference.
end
modelFile=fullfile(outputDir,[mdl,'.slx']);
save_system(mdl,modelFile);
fprintf('Built native Simscape Multibody model: %s\n',modelFile);
end

function build_plant(s)
add_block('simulink/Sources/In1',[s,'/tau'],'Port','1','Position',[30,370,60,390]);
add_block('simulink/Signal Routing/Demux',[s,'/Torques'],'Outputs','2','Position',[100,345,105,415]);
line(s,'tau/1','Torques/1');
add_block('sm_lib/Frames and Transforms/World Frame',[s,'/World'],'Position',[50,70,90,110]);
add_block('sm_lib/Utilities/Mechanism Configuration',[s,'/Gravity'], ...
    'GravityVector','[0 -c.g 0]','Position',[50,150,140,190]);
add_block('nesl_utility/Solver Configuration',[s,'/Solver'],'Position',[50,225,140,265]);
phys(s,'World','RConn',1,'Gravity','RConn',1);
phys(s,'World','RConn',1,'Solver','RConn',1);
for j=1:2
    x=210+300*(j-1); joint=sprintf('Joint %d',j);
    add_block('sm_lib/Joints/Revolute Joint',[s,'/',joint],'Position',[x,70,x+90,140], ...
        'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion', ...
        'PositionTargetSpecify','on','PositionTargetValue',sprintf('c.q0(%d)',j), ...
        'PositionTargetValueUnits','rad','PositionTargetPriority','High', ...
        'VelocityTargetSpecify','on','VelocityTargetValue',sprintf('c.dq0(%d)',j),'VelocityTargetValueUnits','rad/s', ...
        'SensePosition','on','SenseVelocity','on','SenseAcceleration','on');
    trans(s,sprintf('COM %d',j),sprintf('[c.lc%d 0 0]',j),[x+110,170,x+180,210]);
    trans(s,sprintf('Tip %d',j),sprintf('[c.l%d 0 0]',j),[x+110,70,x+180,110]);
    add_block('sm_lib/Body Elements/Brick Solid',[s,sprintf('/Link %d',j)], ...
        'Position',[x+200,170,x+260,230], ...
        'BrickDimensions',sprintf('[c.l%d 0.025 0.025]',j), ...
        'InertiaType','Custom','Mass',sprintf('c.m%d',j), ...
        'CenterOfMass','[0 0 0]','MomentsOfInertia',sprintf('[0 c.I%d c.I%d]',j,j), ...
        'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor',color(j));
    phys(s,joint,'RConn',1,sprintf('COM %d',j),'LConn',1);
    phys(s,joint,'RConn',1,sprintf('Tip %d',j),'LConn',1);
    phys(s,sprintf('COM %d',j),'RConn',1,sprintf('Link %d',j),'RConn',1);
    psin(s,sprintf('Torque %d',j),'N*m',[x,330,x+70,370]);
    line(s,sprintf('Torques/%d',j),sprintf('Torque %d/1',j));
    phys(s,sprintf('Torque %d',j),'RConn',1,joint,'LConn',2);
    for k=1:3
        sensorNames={'q','dq','ddq'}; units={'rad','rad/s','rad/s^2'};
        nm=sprintf('%s%d',sensorNames{k},j);
        psout(s,nm,units{k},[x+100,270+80*k,x+170,310+80*k]);
        phys(s,joint,'RConn',k+1,nm,'LConn',1);
    end
end
phys(s,'World','RConn',1,'Joint 1','LConn',1);
phys(s,'Tip 1','RConn',1,'Joint 2','LConn',1);
trans(s,'Surface Origin','[c.p0(1) c.p0(2) 0]',[170,650,260,690]);
phys(s,'World','RConn',1,'Surface Origin','LConn',1);
add_block('sm_lib/Joints/Pin Slot Joint',[s,'/Rigid Frictionless Contact'], ...
    'Position',[760,80,900,160],'SenseConstraintForce','on', ...
    'CompositeWrenchDir','BaseOnFollower','CompositeWrenchFrame','BaseFrame');
phys(s,'Surface Origin','RConn',1,'Rigid Frictionless Contact','LConn',1);
phys(s,'Tip 2','RConn',1,'Rigid Frictionless Contact','RConn',1);
psout(s,'Contact Force','N',[950,170,1030,210]);
phys(s,'Rigid Frictionless Contact','RConn',2,'Contact Force','LConn',1);
for k=1:3
    labels={'q','dq','ddq'}; nm=labels{k};
    add_block('simulink/Signal Routing/Mux',[s,'/',nm,' vector'],'Inputs','2', ...
        'Position',[1000,300+80*k,1005,350+80*k]);
    line(s,[nm,'1/1'],[nm,' vector/1']); line(s,[nm,'2/1'],[nm,' vector/2']);
    add_block('simulink/Sinks/Out1',[s,'/',nm],'Port',num2str(k), ...
        'Position',[1120,310+80*k,1150,330+80*k]);
    line(s,[nm,' vector/1'],[nm,'/1']);
end
add_block('simulink/Sinks/Out1',[s,'/force'],'Port','4','Position',[1120,180,1150,200]);
line(s,'Contact Force/1','force/1');
a=Simulink.Annotation(s,sprintf(['Native physical plant: world XY plane, gravity -Y, revolute axes +Z.\n', ...
    'Links have exact uniform-rod COM inertia [0 I I]; brick geometry is visual only.\n', ...
    'Pin-slot permits X translation and Z rotation; reaction is sensed in world axes.']));
a.Position=[180,750];
end

function trans(s,name,offset,position)
add_block('sm_lib/Frames and Transforms/Rigid Transform',[s,'/',name], ...
    'TranslationMethod','Cartesian','TranslationCartesianOffset',offset,'Position',position);
end
function psin(s,name,unit,position)
add_block('nesl_utility/Simulink-PS Converter',[s,'/',name], ...
    'Unit',unit,'FilteringAndDerivatives','provide','UdotUserProvided','0','Position',position);
end
function psout(s,name,unit,position)
add_block('nesl_utility/PS-Simulink Converter',[s,'/',name],'Unit',unit,'Position',position);
end
function phys(s,a,sideA,indexA,b,sideB,indexB)
drawnow;
pa=get_param([s,'/',a],'PortHandles'); pb=get_param([s,'/',b],'PortHandles');
add_line(s,pa.(sideA)(indexA),pb.(sideB)(indexB),'autorouting','on');
end
function line(s,a,b)
add_line(s,a,b,'autorouting','on');
end
function c=color(j)
if j==1, c='[0.1 0.4 0.8]'; else, c='[0.9 0.45 0.1]'; end
end
