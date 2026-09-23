function run_adrc_session(modelDirectory, outputDirectory)
%RUN_ADRC_SESSION Run offline verification in an existing MATLAB Automation session.
% Uses function-local variables, preserves session paths/settings, and never exits MATLAB.
arguments
    modelDirectory (1,:) char
    outputDirectory (1,:) char
end
original.directory = pwd;
original.path = path;
original.environmentPath = getenv('PATH');
original.random = rng;
configuration = Simulink.fileGenControl('getConfig');
original.cache = configuration.CacheFolder;
original.generated = configuration.CodeGenFolder;
original.structure = configuration.CodeGenFolderStructure;
sessionCleanup = onCleanup(@()restoreSession(original));
status = struct('completed', false, 'success', false, 'session_restored', false, 'error', '');
try
    addpath(modelDirectory);
    setenv('PATH', ['D:\application\WinLibs\mingw64\bin;' original.environmentPath]);
    clear run_adrc_pipeline build_adrc_models adrc_simulation_parameters
    report = run_adrc_pipeline(outputDirectory);
    status.success = report.passed;
catch runError
    status.error = getReport(runError, 'extended', 'hyperlinks', 'off');
    disp(status.error);
end
clear sessionCleanup
restoredConfiguration = Simulink.fileGenControl('getConfig');
status.session_restored = strcmp(pwd, original.directory) && strcmp(path, original.path) && ...
    isequal(rng, original.random) && ...
    strcmp(getenv('PATH'), original.environmentPath) && ...
    strcmp(restoredConfiguration.CacheFolder, original.cache) && ...
    strcmp(restoredConfiguration.CodeGenFolder, original.generated) && ...
    isequal(restoredConfiguration.CodeGenFolderStructure, original.structure);
status.success = status.success && status.session_restored;
status.completed = true;
handle = fopen(fullfile(outputDirectory, 'job_status.json'), 'w', 'n', 'UTF-8');
assert(handle >= 0, 'Cannot save existing-session result.');
fileCleanup = onCleanup(@()fclose(handle));
fprintf(handle, '%s', jsonencode(status));
assert(status.success, 'ADRC_SESSION_FAILED: %s', status.error);
disp('ADRC_EXISTING_SESSION_PIPELINE_PASSED');
end

function restoreSession(original)
%RESTORESESSION Restore only settings temporarily changed by this offline job.
Simulink.fileGenControl('set', 'CacheFolder', original.cache, ...
    'CodeGenFolder', original.generated, 'CodeGenFolderStructure', original.structure, ...
    'keepPreviousPath', true);
cd(original.directory);
path(original.path);
setenv('PATH', original.environmentPath);
rng(original.random);
end
