function preview_adrc_models(modelDirectory)
%PREVIEW_ADRC_MODELS Export native Simulink diagrams without changing the verified SLX files.
% Refuses existing loaded names and closes only models opened by this invocation.
arguments
    modelDirectory (1,:) char
end
modelNames = {'adrc_controller','adrc_plant','adrc_validation'};
for modelIndex = 1:numel(modelNames)
    assert(~bdIsLoaded(modelNames{modelIndex}), 'Refusing to reuse an already loaded model.');
end
modelCleanup = onCleanup(@()closePreviews(modelNames));
for modelIndex = 1:numel(modelNames)
    modelName = modelNames{modelIndex};
    open_system(fullfile(modelDirectory, [modelName '.slx']));
    print(['-s' modelName], '-dpng', '-r120', fullfile(modelDirectory, [modelName '.png']));
end
end

function closePreviews(modelNames)
%CLOSEPREVIEWS Discard display-only changes to models absent before preview began.
for modelIndex = 1:numel(modelNames)
    if bdIsLoaded(modelNames{modelIndex}), close_system(modelNames{modelIndex}, 0); end
end
end
