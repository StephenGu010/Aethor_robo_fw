function modelPaths = build_adrc_models(outputDirectory)
%BUILD_ADRC_MODELS Rebuild controller, toy plant, and offline validation SLX.
% Public Simulink APIs only; all arithmetic/controller state is scalar single.
% Models are newly created under outputDirectory; existing loaded models fail.
arguments
    outputDirectory (1,:) char
end
assert(isfolder(outputDirectory), 'Output directory must already exist.');
modelNames = {'adrc_controller', 'adrc_plant', 'adrc_validation'};
for modelIndex = 1:numel(modelNames)
    assert(~bdIsLoaded(modelNames{modelIndex}), ...
        'Close the previous diagnostic model before rebuilding: %s', modelNames{modelIndex});
end
load_system('simulink');
parameters = adrc_simulation_parameters;
buildController(modelNames{1});
buildPlant(modelNames{2});
save_system(modelNames{1}, fullfile(outputDirectory, [modelNames{1} '.slx']));
save_system(modelNames{2}, fullfile(outputDirectory, [modelNames{2} '.slx']));
buildValidation(modelNames{3}, parameters);
modelPaths = struct;
for modelIndex = 1:numel(modelNames)
    modelName = modelNames{modelIndex};
    % All blocks in this newly created scope belong to this rebuild. Arrange
    % them before saving so the delivered SLX is readable as well as executable.
    Simulink.BlockDiagram.arrangeSystem(modelName);
    set_param(modelName, 'SimulationCommand', 'update');
    modelPaths.(modelName) = fullfile(outputDirectory, [modelName '.slx']);
    save_system(modelName, modelPaths.(modelName));
end
end

function configureDiscreteModel(modelName)
%CONFIGUREDISCRETEMODEL Set the common deterministic four-millisecond clock.
new_system(modelName);
set_param(modelName, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.004', 'StartTime', '0', 'StopTime', '6', ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', ...
    'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'off', ...
    'AlgebraicLoopMsg', 'error', 'DefaultParameterBehavior', 'Inlined');
end

function buildController(modelName)
%BUILDCONTROLLER Form the frozen prediction/correction ESO and tracking PI.
configureDiscreteModel(modelName);
inputNames = {'mode','reference_rad_s','velocity_rad_s','prev_sent_nm', ...
    'b0','wc_rad_s','wo_rad_s','reset','new_sample'};
for inputIndex = 1:numel(inputNames)
    inputType = 'single';
    if ismember(inputIndex, [1 8 9]), inputType = 'uint8'; end
    addInput(modelName, inputNames{inputIndex}, inputIndex, inputType);
end
constant(modelName, 'Ts', 'single(0.004)');
constant(modelName, 'One', 'single(1)');
constant(modelName, 'Zero', 'single(0)');
constant(modelName, 'Two', 'single(2)');
unitDelay(modelName, 'Z1Memory');
unitDelay(modelName, 'Z2Memory');
unitDelay(modelName, 'IntegralMemory');
unitDelay(modelName, 'PreviousRaw');
% Observer poles and controller rate use exp(-bandwidth*Ts), never Euler gains.
product(modelName, 'ObserverTime', {'wo_rad_s','Ts'}, '**');
gain(modelName, 'NegativeObserverTime', 'single(-1)', 'ObserverTime');
mathBlock(modelName, 'ObserverPole', 'exp', 'NegativeObserverTime');
product(modelName, 'PoleSquared', {'ObserverPole','ObserverPole'}, '**');
sumBlock(modelName, 'L1', '+-', {'One','PoleSquared'});
sumBlock(modelName, 'OneMinusPole', '+-', {'One','ObserverPole'});
product(modelName, 'L2', {'OneMinusPole','OneMinusPole','Ts'}, '**/');
product(modelName, 'ControllerTime', {'wc_rad_s','Ts'}, '**');
gain(modelName, 'NegativeControllerTime', 'single(-1)', 'ControllerTime');
mathBlock(modelName, 'ControllerPole', 'exp', 'NegativeControllerTime');
sumBlock(modelName, 'OneMinusControllerPole', '+-', {'One','ControllerPole'});
product(modelName, 'Kc', {'OneMinusControllerPole','Ts'}, '*/');
product(modelName, 'StateAdvance', {'Ts','Z2Memory'}, '**');
product(modelName, 'InputAdvance', {'b0','Ts','prev_sent_nm'}, '***');
sumBlock(modelName, 'Z1Prediction', '+++', {'Z1Memory','StateAdvance','InputAdvance'});
sumBlock(modelName, 'Innovation', '+-', {'velocity_rad_s','Z1Prediction'});
product(modelName, 'Z1Correction', {'L1','Innovation'}, '**');
product(modelName, 'Z2Correction', {'L2','Innovation'}, '**');
sumBlock(modelName, 'Z1Corrected', '++', {'Z1Prediction','Z1Correction'});
sumBlock(modelName, 'Z2Corrected', '++', {'Z2Memory','Z2Correction'});
switchBlock(modelName, 'Z1SampleGate', 'Z1Corrected', 'new_sample', 'Z1Prediction');
switchBlock(modelName, 'Z2SampleGate', 'Z2Corrected', 'new_sample', 'Z2Memory');
switchBlock(modelName, 'Z1Reset', 'velocity_rad_s', 'reset', 'Z1SampleGate');
switchBlock(modelName, 'Z2Reset', 'Zero', 'reset', 'Z2SampleGate');
wire(modelName, 'Z1Reset', 'Z1Memory', 1);
wire(modelName, 'Z2Reset', 'Z2Memory', 1);
sumBlock(modelName, 'ObserverError', '+-', {'reference_rad_s','Z1Reset'});
product(modelName, 'ObserverFeedback', {'Kc','ObserverError'}, '**');
sumBlock(modelName, 'DisturbanceCompensation', '+-', {'ObserverFeedback','Z2Reset'});
product(modelName, 'LadrcRaw', {'DisturbanceCompensation','b0'}, '*/');
% PI design on the ideal integrator: Kp=2*Kc/b0, Ki=Kc^2/b0, Kaw=Kc.
sumBlock(modelName, 'PiError', '+-', {'reference_rad_s','velocity_rad_s'});
product(modelName, 'PiKp', {'Two','Kc','b0'}, '**/');
product(modelName, 'PiKi', {'Kc','Kc','b0'}, '**/');
product(modelName, 'PiProportional', {'PiKp','PiError'}, '**');
product(modelName, 'IntegralErrorAdvance', {'Ts','PiKi','PiError'}, '***');
switchBlock(modelName, 'IntegralSampleGate', 'IntegralErrorAdvance', 'new_sample', 'Zero');
sumBlock(modelName, 'ActuatorTrackingError', '+-', {'prev_sent_nm','PreviousRaw'});
product(modelName, 'AntiWindupAdvance', {'Ts','Kc','ActuatorTrackingError'}, '***');
sumBlock(modelName, 'IntegralAdvance', '+++', {'IntegralMemory','IntegralSampleGate','AntiWindupAdvance'});
switchBlock(modelName, 'IntegralReset', 'Zero', 'reset', 'IntegralAdvance');
wire(modelName, 'IntegralReset', 'IntegralMemory', 1);
sumBlock(modelName, 'PiRaw', '++', {'PiProportional','IntegralReset'});
add_block('simulink/Logic and Bit Operations/Compare To Constant', ...
    [modelName '/IsPi'], 'relop', '==', 'const', 'uint8(1)');
wire(modelName, 'mode', 'IsPi', 1);
switchBlock(modelName, 'RawSelection', 'PiRaw', 'IsPi', 'LadrcRaw');
switchBlock(modelName, 'RawMemoryReset', 'Zero', 'reset', 'RawSelection');
wire(modelName, 'RawMemoryReset', 'PreviousRaw', 1);
addOutput(modelName, 'torque_raw_nm', 1, 'RawSelection');
addOutput(modelName, 'z1', 2, 'Z1Reset');
addOutput(modelName, 'z2', 3, 'Z2Reset');
set_param(modelName, 'SystemTargetFile', 'ert.tlc', 'TargetLang', 'C', ...
    'GenCodeOnly', 'on', 'GenerateReport', 'off', 'GenerateSampleERTMain', 'off', ...
    'CodeInterfacePackaging', 'Nonreusable function', ...
    'SupportNonFinite', 'on', 'SupportContinuousTime', 'off', ...
    'MatFileLogging', 'off', 'ProdHWDeviceType', 'ARM Compatible->ARM Cortex-M');
end

function buildPlant(modelName)
%BUILDPLANT Build an explicit synthetic first-order velocity/position object.
configureDiscreteModel(modelName);
% Model-reference defaults combine outputs/update and mark every input as
% direct feedthrough. Split only the plant schedule; controller keeps step().
set_param(modelName, 'CombineOutputUpdateFcns', 'off', ...
    'ModelReferenceMinAlgLoopOccurrences', 'on');
names = {'torque_nm','disturbance_rad_s2','actual_b0','damping','reset'};
for inputIndex = 1:numel(names)
    inputType = 'single';
    if inputIndex == 5, inputType = 'uint8'; end
    addInput(modelName, names{inputIndex}, inputIndex, inputType);
end
constant(modelName, 'Ts', 'single(0.004)');
constant(modelName, 'Zero', 'single(0)');
unitDelay(modelName, 'VelocityMemory');
unitDelay(modelName, 'PositionMemory');
switchBlock(modelName, 'VelocityReset', 'Zero', 'reset', 'VelocityMemory');
switchBlock(modelName, 'PositionReset', 'Zero', 'reset', 'PositionMemory');
product(modelName, 'AccelerationInput', {'actual_b0','torque_nm'}, '**');
product(modelName, 'DampingAcceleration', {'damping','VelocityReset'}, '**');
sumBlock(modelName, 'Acceleration', '+-+', {'AccelerationInput','DampingAcceleration','disturbance_rad_s2'});
product(modelName, 'VelocityAdvance', {'Ts','Acceleration'}, '**');
sumBlock(modelName, 'NextVelocity', '++', {'VelocityReset','VelocityAdvance'});
product(modelName, 'PositionAdvance', {'Ts','VelocityReset'}, '**');
sumBlock(modelName, 'NextPosition', '++', {'PositionReset','PositionAdvance'});
switchBlock(modelName, 'NextVelocityReset', 'Zero', 'reset', 'NextVelocity');
switchBlock(modelName, 'NextPositionReset', 'Zero', 'reset', 'NextPosition');
wire(modelName, 'NextVelocityReset', 'VelocityMemory', 1);
wire(modelName, 'NextPositionReset', 'PositionMemory', 1);
% Plant outputs are stored states. Synchronous reset affects the next state,
% so the Model block has no artificial reset-to-output direct feedthrough.
addOutput(modelName, 'velocity_rad_s', 1, 'VelocityMemory');
addOutput(modelName, 'position_rad', 2, 'PositionMemory');
end

function buildValidation(modelName, parameters)
%BUILDVALIDATION Connect plant/controller with common external actuator limits.
configureDiscreteModel(modelName);
names = {'mode','reference_rad_s','disturbance_rad_s2','measurement_noise', ...
    'new_sample','reset'};
for inputIndex = 1:numel(names)
    inputType = 'single';
    if ismember(inputIndex, [1 5 6]), inputType = 'uint8'; end
    addInput(modelName, names{inputIndex}, inputIndex, inputType);
end
workspace = get_param(modelName, 'ModelWorkspace');
assignin(workspace, 'scenarioB0', parameters.b0);
assignin(workspace, 'scenarioDamping', parameters.damping);
assignin(workspace, 'scenarioDelay', uint8(0));
constant(modelName, 'B0', sprintf('single(%.9g)', parameters.b0));
constant(modelName, 'Wc', sprintf('single(%.9g)', parameters.wc));
constant(modelName, 'Wo', sprintf('single(%.9g)', parameters.wo));
constant(modelName, 'ActualB0', 'scenarioB0');
constant(modelName, 'Damping', 'scenarioDamping');
constant(modelName, 'DelaySteps', 'scenarioDelay', 'uint8');
constant(modelName, 'Zero', 'single(0)');
add_block('simulink/Ports & Subsystems/Model', [modelName '/Controller'], ...
    'ModelName', 'adrc_controller', 'SimulationMode', 'Normal', 'Position', [100 100 420 420]);
add_block('simulink/Ports & Subsystems/Model', [modelName '/Plant'], ...
    'ModelName', 'adrc_plant', 'SimulationMode', 'Normal', 'Position', [100 500 420 700]);
unitDelay(modelName, 'PreviousSent');
unitDelay(modelName, 'MeasurementDelay1');
unitDelay(modelName, 'MeasurementDelay2');
wire(modelName, 'Plant/1', 'MeasurementDelay1', 1);
wire(modelName, 'MeasurementDelay1', 'MeasurementDelay2', 1);
add_block('simulink/Logic and Bit Operations/Compare To Constant', ...
    [modelName '/DelayIsTwo'], 'relop', '==', 'const', 'uint8(2)');
wire(modelName, 'DelaySteps', 'DelayIsTwo', 1);
switchBlock(modelName, 'SelectDelayed', 'MeasurementDelay2', 'DelayIsTwo', 'MeasurementDelay1');
switchBlock(modelName, 'SelectMeasurement', 'SelectDelayed', 'DelaySteps', 'Plant/1');
sumBlock(modelName, 'NoisyMeasurement', '++', {'SelectMeasurement','measurement_noise'});
gain(modelName, 'VelocityToCounts', sprintf('single(%.9g)', 1/parameters.velocityQuantum), 'NoisyMeasurement');
add_block('simulink/Math Operations/Rounding Function', [modelName '/VelocityCounts'], 'Operator', 'round');
wire(modelName, 'VelocityToCounts', 'VelocityCounts', 1);
gain(modelName, 'VelocityQuantized', sprintf('single(%.9g)', parameters.velocityQuantum), 'VelocityCounts');
controllerInputs = {'mode','reference_rad_s','VelocityQuantized','PreviousSent','B0','Wc','Wo','reset','new_sample'};
for inputIndex = 1:numel(controllerInputs)
    wire(modelName, controllerInputs{inputIndex}, 'Controller', inputIndex);
end
sumBlock(modelName, 'TorqueDifference', '+-', {'Controller/1','PreviousSent'});
add_block('simulink/Discontinuities/Saturation', [modelName '/TorqueSlew'], ...
    'UpperLimit', sprintf('single(%.9g)', parameters.torqueSlew*parameters.sampleTime), ...
    'LowerLimit', sprintf('single(%.9g)', -parameters.torqueSlew*parameters.sampleTime));
wire(modelName, 'TorqueDifference', 'TorqueSlew', 1);
sumBlock(modelName, 'SlewedTorque', '++', {'PreviousSent','TorqueSlew'});
add_block('simulink/Discontinuities/Saturation', [modelName '/TorqueAmplitude'], ...
    'UpperLimit', sprintf('single(%.9g)', parameters.torqueLimit), ...
    'LowerLimit', sprintf('single(%.9g)', -parameters.torqueLimit));
wire(modelName, 'SlewedTorque', 'TorqueAmplitude', 1);
gain(modelName, 'TorqueToCounts', sprintf('single(%.9g)', 1/parameters.torqueQuantum), 'TorqueAmplitude');
add_block('simulink/Math Operations/Rounding Function', [modelName '/TorqueCounts'], 'Operator', 'round');
wire(modelName, 'TorqueToCounts', 'TorqueCounts', 1);
gain(modelName, 'QuantizedTorque', sprintf('single(%.9g)', parameters.torqueQuantum), 'TorqueCounts');
switchBlock(modelName, 'SentTorque', 'Zero', 'reset', 'QuantizedTorque');
wire(modelName, 'SentTorque', 'PreviousSent', 1);
plantInputs = {'SentTorque','disturbance_rad_s2','ActualB0','Damping','reset'};
for inputIndex = 1:numel(plantInputs), wire(modelName, plantInputs{inputIndex}, 'Plant', inputIndex); end
outputNames = {'torque_raw_nm','z1','z2','velocity_rad_s','position_rad','torque_sent_nm','velocity_measured','prev_sent_nm'};
outputSources = {'Controller/1','Controller/2','Controller/3','Plant/1','Plant/2','SentTorque','VelocityQuantized','PreviousSent'};
for outputIndex = 1:numel(outputNames)
    addOutput(modelName, outputNames{outputIndex}, outputIndex, outputSources{outputIndex});
end
end

function addInput(modelName, name, portNumber, dataType)
%ADDINPUT Declare a scalar discrete root port with an explicit wire type.
add_block('simulink/Sources/In1', [modelName '/' name], 'Port', num2str(portNumber), ...
    'OutDataTypeStr', dataType, 'PortDimensions', '1', 'SampleTime', '0.004');
end

function addOutput(modelName, name, portNumber, source)
%ADDOUTPUT Expose one scalar single output in the public generated interface.
add_block('simulink/Sinks/Out1', [modelName '/' name], 'Port', num2str(portNumber), 'OutDataTypeStr', 'single');
wire(modelName, source, name, 1);
end

function constant(modelName, name, expression, dataType)
%CONSTANT Create a typed parameter, including model-workspace scenario values.
if nargin < 4, dataType = 'single'; end
add_block('simulink/Sources/Constant', [modelName '/' name], ...
    'Value', expression, 'OutDataTypeStr', dataType);
end

function unitDelay(modelName, name)
%UNITDELAY Allocate one explicit static single-precision state.
add_block('simulink/Discrete/Unit Delay', [modelName '/' name], ...
    'SampleTime', '0.004', 'InitialCondition', 'single(0)');
end

function sumBlock(modelName, name, signs, sources)
%SUMBLOCK Connect an ordered scalar sum without implicit double constants.
add_block('simulink/Math Operations/Sum', [modelName '/' name], 'Inputs', signs, 'OutDataTypeStr', 'single');
for inputIndex = 1:numel(sources), wire(modelName, sources{inputIndex}, name, inputIndex); end
end

function product(modelName, name, sources, operators)
%PRODUCT Connect scalar multiplication/division with single result type.
add_block('simulink/Math Operations/Product', [modelName '/' name], 'Inputs', operators, 'OutDataTypeStr', 'single');
for inputIndex = 1:numel(sources), wire(modelName, sources{inputIndex}, name, inputIndex); end
end

function gain(modelName, name, expression, source)
%GAIN Apply a named single-precision constant scaling operation.
add_block('simulink/Math Operations/Gain', [modelName '/' name], 'Gain', expression, 'OutDataTypeStr', 'single');
wire(modelName, source, name, 1);
end

function mathBlock(modelName, name, operation, source)
%MATHBLOCK Add the exponential used to map continuous bandwidth to a pole.
add_block('simulink/Math Operations/Math Function', [modelName '/' name], 'Operator', operation);
wire(modelName, source, name, 1);
end

function switchBlock(modelName, name, whenTrue, condition, whenFalse)
%SWITCHBLOCK Choose exactly one data branch from a nonzero Boolean/uint8 flag.
add_block('simulink/Signal Routing/Switch', [modelName '/' name], ...
    'Criteria', 'u2 ~= 0', 'OutDataTypeStr', 'single');
wire(modelName, whenTrue, name, 1);
wire(modelName, condition, name, 2);
wire(modelName, whenFalse, name, 3);
end

function wire(modelName, source, destination, destinationPort)
%WIRE Connect known newly-created block ports; source defaults to port one.
if ~contains(source, '/'), source = [source '/1']; end
add_line(modelName, source, [destination '/' num2str(destinationPort)], 'autorouting', 'on');
end
