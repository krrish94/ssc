function dataPrepBatch(sceneIdsFile)

sceneIds = importdata(sceneIdsFile);

for i = 1:length(sceneIds)
    sceneId = sceneIds{i};
    disp(sceneId);
    dataPrep(sceneId);
end


% dataPrep(sceneId);

% End of function dataPrepBatch
end




