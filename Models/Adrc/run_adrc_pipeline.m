function report = run_adrc_pipeline(outputDirectory)
%RUN_ADRC_PIPELINE Rebuild SLX, execute MIL, generate C, and replay generated C.
% Entire pipeline is offline; synthetic parameters cannot qualify hardware.
arguments
    outputDirectory (1,:) char
end
sourceDirectory = fileparts(mfilename('fullpath'));
ownedModelNames = {'adrc_validation','adrc_controller','adrc_plant'};
for modelIndex = 1:numel(ownedModelNames)
    assert(~bdIsLoaded(ownedModelNames{modelIndex}), ...
        'Pipeline refuses to alter an already loaded model: %s', ownedModelNames{modelIndex});
end
modelCleanup = onCleanup(@()closeOwnedModels(ownedModelNames));
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
originalDirectory = pwd;
directoryCleanup = onCleanup(@()cd(originalDirectory));
cd(outputDirectory);
outputDirectory = pwd;
addpath(sourceDirectory);
addpath(outputDirectory);
report = struct('passed', false, 'stage', 'starting', 'matlab', version, ...
    'verificationHashesMatch', false, ...
    'provenance', 'offline_synthetic_example_not_motor_measurement');
reportFile = fullfile(outputDirectory, 'pipeline_report.json');
writeJson(reportFile, report);
try
    assert(usejava('jvm'), 'SHA-256 verification requires a JVM-enabled MATLAB session.');
    sourceNames = {'build_adrc_models.m','run_adrc_pipeline.m', ...
        'adrc_simulation_parameters.m','adrc_generated_replay.c'};
    sourcePaths = cellfun(@(name)fullfile(sourceDirectory,name), sourceNames, 'UniformOutput', false);
    report.verificationSources = hashFileManifest(sourcePaths);
    writeJson(reportFile, report);
    parameters = adrc_simulation_parameters;
    report.parameters = parameters;
    save(fullfile(outputDirectory, 'simulation_parameters.mat'), 'parameters');
    Simulink.fileGenControl('set', 'CacheFolder', fullfile(outputDirectory, 'cache'), ...
        'CodeGenFolder', fullfile(outputDirectory, 'generated'), 'createDir', true);
    report.stage = 'building_models'; writeJson(reportFile, report);
    report.models = build_adrc_models(outputDirectory);
    report.stage = 'generating_controller_c'; writeJson(reportFile, report);
    slbuild('adrc_controller');
    report.generatedBuild = RTW.getBuildDir('adrc_controller');
    artifactPaths = verificationArtifactPaths(report.models, report.generatedBuild.BuildDirectory);
    report.verificationArtifacts = hashFileManifest(artifactPaths);
    report.stage = 'controller_mil'; writeJson(reportFile, report);
    [report.controllerMil, replayVectors] = verifyController(parameters);
    vectorFile = fullfile(outputDirectory, 'controller_vectors.csv');
    writematrix(replayVectors, vectorFile);
    report.stage = 'closed_loop_mil'; writeJson(reportFile, report);
    [report.closedLoopMil, traces] = verifyClosedLoop(parameters);
    save(fullfile(outputDirectory, 'closed_loop_traces.mat'), 'traces', 'parameters', '-v7.3');
    for traceIndex = 1:numel(traces)
        controllerInputs = zeros(size(traces(traceIndex).inputs,1),9);
        controllerInputs(:,1:2) = double(traces(traceIndex).inputs(:,1:2));
        controllerInputs(:,3:4) = double(traces(traceIndex).outputs(:,7:8));
        controllerInputs(:,5:7) = repmat(double([parameters.b0 parameters.wc parameters.wo]), size(controllerInputs,1), 1);
        controllerInputs(:,8) = double(traces(traceIndex).inputs(:,6));
        controllerInputs(:,9) = double(traces(traceIndex).inputs(:,5));
        replayVectors = [replayVectors; controllerInputs double(traces(traceIndex).outputs(:,1:3))]; %#ok<AGROW>
    end
    writematrix(replayVectors, vectorFile);
    report.stage = 'generated_c_replay'; writeJson(reportFile, report);
    report.cReplay = replayGeneratedCode(sourceDirectory, outputDirectory, report.generatedBuild.BuildDirectory, vectorFile);
    report.stage = 'verifying_hashes'; writeJson(reportFile, report);
    assertManifestUnchanged(report.verificationSources, hashFileManifest(sourcePaths));
    currentArtifactPaths = verificationArtifactPaths(report.models, report.generatedBuild.BuildDirectory);
    assertManifestUnchanged(report.verificationArtifacts, hashFileManifest(currentArtifactPaths));
    report.verificationHashesMatch = true;
    report.passed = report.verificationHashesMatch && report.controllerMil.passed && ...
        report.closedLoopMil.passed && report.cReplay.passed;
    report.stage = 'completed';
    writeJson(reportFile, report);
    assert(report.passed, 'Offline ADRC verification did not pass.');
    disp('ADRC_PIPELINE_COMPLETED');
catch pipelineException
    report.passed = false;
    report.error = getReport(pipelineException, 'extended', 'hyperlinks', 'off');
    writeJson(reportFile, report);
    rethrow(pipelineException);
end
end

function paths = verificationArtifactPaths(modelPaths, buildDirectory)
%VERIFICATIONARTIFACTPATHS Enumerate all three models and generated C/headers.
modelNames = {'adrc_controller','adrc_plant','adrc_validation'};
paths = cellfun(@(name)modelPaths.(name), modelNames, 'UniformOutput', false);
cSources = dir(fullfile(buildDirectory, '*.c'));
headers = dir(fullfile(buildDirectory, '*.h'));
assert(~isempty(cSources) && ~isempty(headers), 'Generated C and header evidence must both exist.');
generatedFiles = [cSources; headers];
for fileIndex = 1:numel(generatedFiles)
    assert(~generatedFiles(fileIndex).isdir, 'Generated evidence must be a file.');
    paths{end+1} = fullfile(buildDirectory, generatedFiles(fileIndex).name); %#ok<AGROW>
end
end

function manifest = hashFileManifest(paths)
%HASHFILEMANIFEST Record deterministic absolute paths and SHA-256 fingerprints.
assert(~isempty(paths), 'Verification manifest cannot be empty.');
manifest = repmat(struct('path', '', 'sha256', ''), numel(paths), 1);
for fileIndex = 1:numel(paths)
    file = javaObject('java.io.File', paths{fileIndex});
    assert(file.isFile(), 'Verification file is missing: %s', paths{fileIndex});
    manifest(fileIndex).path = char(file.getCanonicalPath());
    manifest(fileIndex).sha256 = fileSha256(manifest(fileIndex).path);
end
[~, sortedIndices] = sort({manifest.path});
manifest = manifest(sortedIndices);
assert(numel(unique({manifest.path})) == numel(manifest), 'Verification paths must be unique.');
end

function fingerprint = fileSha256(path)
%FILESHA256 Stream binary bytes through the public Java SHA-256 implementation.
[handle, message] = fopen(path, 'rb');
assert(handle ~= -1, 'Cannot read verification file: %s (%s)', path, message);
fileCleanup = onCleanup(@()fclose(handle));
digest = javaMethod('getInstance', 'java.security.MessageDigest', 'SHA-256');
assert(fseek(handle, 0, 'eof') == 0, 'Cannot seek verification file: %s', path);
remainingBytes = ftell(handle);
assert(remainingBytes >= 0 && fseek(handle, 0, 'bof') == 0, 'Cannot size verification file: %s', path);
while remainingBytes > 0
    requestedBytes = min(1024*1024, remainingBytes);
    bytes = fread(handle, requestedBytes, '*uint8');
    [readMessage, readError] = ferror(handle);
    assert(numel(bytes) == requestedBytes && readError == 0, ...
        'Cannot completely read verification file: %s (%s)', path, readMessage);
    digest.update(typecast(bytes, 'int8'));
    remainingBytes = remainingBytes - numel(bytes);
end
digestBytes = typecast(int8(digest.digest()), 'uint8');
fingerprint = lower(reshape(dec2hex(digestBytes, 2).', 1, []));
assert(numel(fingerprint) == 64, 'Unexpected SHA-256 digest length.');
end

function assertManifestUnchanged(expected, actual)
%ASSERTMANIFESTUNCHANGED Reject removed/added files and any changed byte content.
assert(isequal({expected.path}, {actual.path}), 'Verification file set changed during the pipeline.');
for fileIndex = 1:numel(expected)
    assert(strcmp(expected(fileIndex).sha256, actual(fileIndex).sha256), ...
        'Verification file changed during the pipeline: %s', expected(fileIndex).path);
end
end

function closeOwnedModels(modelNames)
%CLOSEOWNEDMODELS Close only model names verified absent before this invocation.
for modelIndex = 1:numel(modelNames)
    if bdIsLoaded(modelNames{modelIndex}), close_system(modelNames{modelIndex}, 0); end
end
end

function [summary, allVectors] = verifyController(parameters)
%VERIFYCONTROLLER Compare block model with independently expressed recurrence.
time = (0:1000)' * 0.004;
sampleCount = numel(time);
allVectors = zeros(0, 12);
maximumError = 0;
for mode = uint8([1 2])
    inputs = zeros(sampleCount, 9, 'single');
    inputs(:,1) = single(mode);
    inputs(:,2) = single(0.2*sin(2*time));
    inputs(:,3) = single(0.12*sin(1.7*time)+0.03*cos(8*time));
    inputs(:,4) = single(0.04*sin(3*time));
    inputs(:,5) = parameters.b0;
    inputs(:,6) = parameters.wc;
    inputs(:,7) = parameters.wo;
    inputs([1 501],8) = single(1);
    inputs(:,9) = single(mod((0:sampleCount-1)',7) ~= 3);
    expected = referenceRecurrence(inputs, double(parameters.sampleTime));
    actual = simulateController(time, inputs);
    normalizedError = max(abs(double(actual)-expected)./max(1,abs(expected)), [], 'all');
    maximumError = max(maximumError, normalizedError);
    assert(normalizedError <= 1e-4, 'Controller MIL recurrence mismatch for mode %d.', mode);
    assert(isequal(actual([1 501],2), inputs([1 501],3)), 'Reset must use current velocity.');
    assert(all(actual([1 501],3) == 0), 'Reset must clear disturbance state.');
    allVectors = [allVectors; double(inputs) double(actual)]; %#ok<AGROW>
end
% A fresh nonfinite measurement must reach the wrapper as nonfinite, and reset
% must recover the observer. Stale LADRC samples must not apply correction.
invalidInputs = repmat(single([2 0.1 0 0 parameters.b0 parameters.wc parameters.wo 0 1]), 6, 1);
invalidInputs([1 4 6],8) = 1;
invalidInputs(2,3) = single(NaN); invalidInputs(2,9) = 0;
invalidInputs(3,3) = single(NaN);
invalidInputs(5,3) = single(Inf);
invalidOutputs = simulateController((0:5)'*0.004, invalidInputs);
assert(all(isfinite(invalidOutputs(2,:))), 'Stale measurement incorrectly corrected the observer.');
assert(~isfinite(invalidOutputs(3,1)) && ~isfinite(invalidOutputs(5,1)), 'Nonfinite measurement must be rejected by adapter.');
assert(all(isfinite(invalidOutputs([4 6],:)), 'all'), 'Reset failed to recover finite states.');
summary = struct('passed', true, 'samples', size(allVectors,1), ...
    'max_normalized_error', maximumError, 'tolerance', 1e-4, ...
    'reset_drop_reverse_and_nonfinite_checks', true);
end

function outputs = referenceRecurrence(inputs, sampleTime)
%REFERENCERECURRENCE Double-precision oracle, independent of the block topology.
velocityEstimate = 0; disturbanceEstimate = 0; integralState = 0; previousRaw = 0;
outputs = zeros(size(inputs,1),3);
for sampleIndex = 1:size(inputs,1)
    values = double(inputs(sampleIndex,:));
    controllerRate = -expm1(-values(6)*sampleTime)/sampleTime;
    observerPole = exp(-values(7)*sampleTime);
    error = values(2)-values(3);
    if values(8) ~= 0
        velocityEstimate = values(3); disturbanceEstimate = 0; integralState = 0;
    else
        velocityEstimate = velocityEstimate + sampleTime*(disturbanceEstimate+values(5)*values(4));
        if values(9) ~= 0
            innovation = values(3)-velocityEstimate;
            velocityEstimate = velocityEstimate+(1-observerPole^2)*innovation;
            disturbanceEstimate = disturbanceEstimate+(1-observerPole)^2/sampleTime*innovation;
            integralState = integralState+sampleTime*controllerRate^2/values(5)*error;
        end
        integralState = integralState+sampleTime*controllerRate*(values(4)-previousRaw);
    end
    if values(1) == 1
        raw = 2*controllerRate/values(5)*error+integralState;
    else
        raw = (controllerRate*(values(2)-velocityEstimate)-disturbanceEstimate)/values(5);
    end
    outputs(sampleIndex,:) = [raw velocityEstimate disturbanceEstimate];
    previousRaw = raw;
    if values(8) ~= 0, previousRaw = 0; end
end
end

function outputs = simulateController(time, inputs)
%SIMULATECONTROLLER Run root inputs without MEX or hardware transport.
names = {'mode','reference_rad_s','velocity_rad_s','prev_sent_nm','b0','wc_rad_s','wo_rad_s','reset','new_sample'};
simulationInput = Simulink.SimulationInput('adrc_controller');
simulationInput = simulationInput.setExternalInput(makeDataset(time, inputs, names, [1 8 9]));
simulationInput = simulationInput.setModelParameter('StopTime', sprintf('%.12g', time(end)));
simulationOutput = sim(simulationInput);
outputs = extractOutputs(simulationOutput.yout, 3);
assert(size(outputs,1) == numel(time), 'Unexpected simulation sample count.');
end

function [summary, traces] = verifyClosedLoop(parameters)
%VERIFYCLOSEDLOOP Sweep synthetic b0 mismatch, feedback delay, and noise.
time = (0:round(parameters.duration/0.004))'*0.004;
reference = min(single(time)*parameters.referenceAcceleration, parameters.referenceAmplitude);
reverseIndex = time >= 2.5;
reference(reverseIndex) = max(parameters.referenceAmplitude-single(time(reverseIndex)-2.5)*parameters.referenceAcceleration, -parameters.referenceAmplitude);
disturbance = single(time >= 2 & time < 3)*parameters.disturbanceAcceleration;
rng(parameters.seed, 'twister');
noise = single(randn(size(time)))*parameters.noiseAmplitude;
scenarioIndex = 0;
metricTemplate = struct('mode', 0, 'b0_ratio', 0, 'delay_samples', 0, ...
    'noise_enabled', false, 'saturation_recovery_stress', false, ...
    'final_error_rad_s', 0, 'peak_speed_rad_s', 0, 'peak_raw_nm', 0, ...
    'peak_sent_nm', 0, 'rms_tracking_error_rad_s', 0);
metrics = repmat(metricTemplate, 38, 1);
traces = repmat(struct('time', [], 'inputs', [], 'outputs', [], 'metrics', metricTemplate), 38, 1);
for mode = uint8([1 2])
    for mismatch = [0.5 1 2]
        for delay = uint8([0 1 2])
            for noisy = [false true]
                scenarioIndex = scenarioIndex+1;
                inputs = zeros(numel(time),6,'single');
                inputs(:,1) = single(mode); inputs(:,2) = reference;
                inputs(:,3) = disturbance; inputs(:,4) = noise*single(noisy);
                inputs(:,5) = single(1); inputs(1,6) = single(1);
                names = {'mode','reference_rad_s','disturbance_rad_s2','measurement_noise','new_sample','reset'};
                simulationInput = Simulink.SimulationInput('adrc_validation');
                simulationInput = simulationInput.setExternalInput(makeDataset(time, inputs, names, [1 5 6]));
                simulationInput = simulationInput.setVariable('scenarioB0', parameters.b0*single(mismatch), 'Workspace', 'adrc_validation');
                simulationInput = simulationInput.setVariable('scenarioDelay', delay, 'Workspace', 'adrc_validation');
                simulationInput = simulationInput.setModelParameter('StopTime', sprintf('%.12g',time(end)));
                simulationOutput = sim(simulationInput);
                values = extractOutputs(simulationOutput.yout, 8);
                assert(all(isfinite(values),'all'), 'Nonfinite closed-loop trace in scenario %d.', scenarioIndex);
                torqueTolerance = double(parameters.torqueQuantum)*0.51+1e-6;
                assert(max(abs(values(:,6))) <= double(parameters.torqueLimit)+torqueTolerance, 'Amplitude limit failed.');
                assert(max(abs(diff(values(:,6)))) <= double(parameters.torqueSlew*parameters.sampleTime)+double(parameters.torqueQuantum)+1e-6, 'Slew limit failed.');
                finalError = abs(double(values(end,4))-double(reference(end)));
                nominal = mismatch == 1 && delay == 0 && ~noisy;
                if nominal, assert(finalError <= 0.01, 'Nominal final tracking error exceeds synthetic 0.01 rad/s band.'); end
                metrics(scenarioIndex) = struct('mode', double(mode), 'b0_ratio', mismatch, ...
                    'delay_samples', double(delay), 'noise_enabled', noisy, ...
                    'saturation_recovery_stress', false, ...
                    'final_error_rad_s', finalError, ...
                    'peak_speed_rad_s', max(abs(double(values(:,4)))), ...
                    'peak_raw_nm', max(abs(double(values(:,1)))), ...
                    'peak_sent_nm', max(abs(double(values(:,6)))), ...
                    'rms_tracking_error_rad_s', sqrt(mean((double(values(:,4))-double(reference)).^2)));
                traces(scenarioIndex).time = time;
                traces(scenarioIndex).inputs = inputs;
                traces(scenarioIndex).outputs = values;
                traces(scenarioIndex).metrics = metrics(scenarioIndex);
            end
        end
    end
end
% An intentionally abrupt reference is an algorithm stress input, not an
% allowed hardware command. Both modes see the same actuator restrictions.
for mode = uint8([1 2])
    scenarioIndex = scenarioIndex+1;
    inputs = zeros(numel(time),6,'single');
    inputs(:,1) = single(mode); inputs(:,2) = single(time > 0 & time < 2);
    inputs(:,5) = single(1); inputs(1,6) = single(1);
    simulationInput = Simulink.SimulationInput('adrc_validation');
    names = {'mode','reference_rad_s','disturbance_rad_s2','measurement_noise','new_sample','reset'};
    simulationInput = simulationInput.setExternalInput(makeDataset(time, inputs, names, [1 5 6]));
    simulationInput = simulationInput.setVariable('scenarioB0', parameters.b0, 'Workspace', 'adrc_validation');
    simulationInput = simulationInput.setVariable('scenarioDelay', uint8(0), 'Workspace', 'adrc_validation');
    simulationInput = simulationInput.setModelParameter('StopTime', sprintf('%.12g',time(end)));
    simulationOutput = sim(simulationInput);
    values = extractOutputs(simulationOutput.yout, 8);
    assert(all(isfinite(values),'all'), 'Nonfinite saturation recovery trace.');
    assert(nnz(abs(values(:,6)) >= parameters.torqueLimit-single(1e-6)) > 10, 'Stress did not exercise amplitude saturation.');
    assert(abs(values(end,4)) <= 0.02, 'Failed to recover to zero speed after saturation.');
    metrics(scenarioIndex) = struct('mode', double(mode), 'b0_ratio', 1, ...
        'delay_samples', 0, 'noise_enabled', false, 'saturation_recovery_stress', true, ...
        'final_error_rad_s', abs(double(values(end,4))), ...
        'peak_speed_rad_s', max(abs(double(values(:,4)))), ...
        'peak_raw_nm', max(abs(double(values(:,1)))), ...
        'peak_sent_nm', max(abs(double(values(:,6)))), ...
        'rms_tracking_error_rad_s', sqrt(mean((double(values(:,4))-double(inputs(:,2))).^2)));
    traces(scenarioIndex).time = time;
    traces(scenarioIndex).inputs = inputs;
    traces(scenarioIndex).outputs = values;
    traces(scenarioIndex).metrics = metrics(scenarioIndex);
end
assert(scenarioIndex == 38, 'The complete closed-loop scenario set must execute.');
summary = struct('passed', true, 'scenarios', scenarioIndex, 'metrics', metrics, ...
    'criterion', 'finite/shared limits; nominal final error <= 0.01 rad/s; saturation recovery <= 0.02 rad/s');
end

function dataset = makeDataset(time, inputs, names, integerPorts)
%MAKEDATASET Preserve typed root ports and zero-order-held sample semantics.
dataset = Simulink.SimulationData.Dataset;
for inputIndex = 1:numel(names)
    values = single(inputs(:,inputIndex));
    if ismember(inputIndex, integerPorts), values = uint8(values); end
    signal = timeseries(values, time);
    signal = setinterpmethod(signal, 'zoh');
    dataset = dataset.addElement(signal, names{inputIndex});
end
end

function values = extractOutputs(dataset, outputCount)
%EXTRACTOUTPUTS Read root outports in declared order with explicit single type.
first = dataset.getElement(1);
values = zeros(numel(first.Values.Time), outputCount, 'single');
for outputIndex = 1:outputCount
    signal = dataset.getElement(outputIndex);
    values(:,outputIndex) = single(signal.Values.Data(:));
end
end

function replay = replayGeneratedCode(sourceDirectory, outputDirectory, buildDirectory, vectorFile)
%REPLAYGENERATEDCODE Compile the emitted controller and compare each MIL row.
sources = dir(fullfile(buildDirectory, '*.c'));
assert(~isempty(sources), 'No generated C sources found.');
executable = fullfile(outputDirectory, 'adrc_generated_replay.exe');
command = 'gcc -std=c99 -O2 -Wall -Wextra -Werror -fno-fast-math -ffp-contract=off';
includePaths = {buildDirectory, fullfile(matlabroot,'extern','include'), ...
    fullfile(matlabroot,'simulink','include'), fullfile(matlabroot,'rtw','c','src')};
for pathIndex = 1:numel(includePaths), command = [command ' -I' quoted(includePaths{pathIndex})]; end %#ok<AGROW>
command = [command ' ' quoted(fullfile(sourceDirectory, 'adrc_generated_replay.c'))];
for sourceIndex = 1:numel(sources)
    command = [command ' ' quoted(fullfile(sources(sourceIndex).folder, sources(sourceIndex).name))]; %#ok<AGROW>
end
command = [command ' -lm -o ' quoted(executable)];
[compileStatus, compileOutput] = system(command);
writeText(fullfile(outputDirectory, 'gcc_replay_build.log'), compileOutput);
assert(compileStatus == 0, 'Generated C replay compilation failed: %s', compileOutput);
replayReport = fullfile(outputDirectory, 'generated_c_replay.json');
[replayStatus, replayOutput] = system([quoted(executable) ' ' quoted(vectorFile) ' ' quoted(replayReport)]);
writeText(fullfile(outputDirectory, 'gcc_replay_run.log'), replayOutput);
assert(replayStatus == 0, 'Generated C replay failed: %s (exit %d)', replayOutput, replayStatus);
replay = jsondecode(fileread(replayReport));
end

function value = quoted(value)
%QUOTED Quote a known local Windows path and reject command metacharacters.
assert(~contains(value, '"') && ~contains(value, '%') && ~contains(value, newline), 'Unsupported path characters.');
value = ['"' value '"'];
end

function writeJson(path, value)
%WRITEJSON Persist stage evidence before subsequent operations can fail.
writeText(path, jsonencode(value, PrettyPrint=true));
end

function writeText(path, value)
%WRITETEXT Save UTF-8 evidence while ensuring the handle is always closed.
[handle, message] = fopen(path, 'w', 'n', 'UTF-8');
assert(handle ~= -1, 'Cannot write evidence: %s', message);
fileCleanup = onCleanup(@()fclose(handle));
fprintf(handle, '%s', value);
end
