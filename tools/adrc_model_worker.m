function adrc_model_worker(controlDirectory, modelDirectory)
%ADRC_MODEL_WORKER Reuse one task-owned MATLAB session for offline model iteration.
% Accepts only run/stop messages under the fixed control directory; no hardware I/O.
% Model failures are recorded without restarting MATLAB's unstable startup services.
arguments
    controlDirectory (1,:) char
    modelDirectory (1,:) char
end
addpath(modelDirectory);
requestPath = fullfile(controlDirectory, 'request.json');
lastRequestId = 0;
idleTimer = tic;
writeWorkerJson(fullfile(controlDirectory, 'ready.json'), struct('ready', true, 'version', version));
while toc(idleTimer) < 900
    pause(0.5);
    if ~isfile(requestPath), continue; end
    try
        request = jsondecode(fileread(requestPath));
    catch
        continue; % A writer may still be replacing the small request file.
    end
    if ~isfield(request, 'id') || request.id <= lastRequestId, continue; end
    lastRequestId = request.id;
    idleTimer = tic;
    if strcmp(request.action, 'stop'), break; end
    assert(strcmp(request.action, 'run'), 'Unsupported worker operation.');
    outputDirectory = fullfile(controlDirectory, '..', 'models', sprintf('worker_%06d', request.id));
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    result = struct('id', request.id, 'completed', false, 'passed', false, 'output', outputDirectory, 'error', '');
    try
        clear run_adrc_pipeline build_adrc_models adrc_simulation_parameters
        report = run_adrc_pipeline(outputDirectory);
        result.passed = report.passed;
    catch jobException
        result.error = getReport(jobException, 'extended', 'hyperlinks', 'off');
        disp(result.error);
    end
    closeOwnedModels;
    result.completed = true;
    writeWorkerJson(fullfile(controlDirectory, 'result.json'), result);
end
writeWorkerJson(fullfile(controlDirectory, 'stopped.json'), struct('stopped', true));
exit(0);
end

function closeOwnedModels
%CLOSEOWNEDMODELS Discard only the three models created in this exclusive worker.
modelNames = {'adrc_validation', 'adrc_controller', 'adrc_plant'};
for modelIndex = 1:numel(modelNames)
    if bdIsLoaded(modelNames{modelIndex}), close_system(modelNames{modelIndex}, 0); end
end
end

function writeWorkerJson(path, value)
%WRITEWORKERJSON Persist completion evidence as UTF-8 within the task workspace.
handle = fopen(path, 'w', 'n', 'UTF-8');
assert(handle >= 0, 'Cannot write worker evidence.');
cleanup = onCleanup(@()fclose(handle));
fprintf(handle, '%s', jsonencode(value));
end
