function dataPrep(sceneId)

% Setup paths to relevant MATLAB scripts
addpath('utils/');

% Setup dataset paths
suncgDataPath = '/home/jatavalk/data/suncg/';
suncgToolboxPath = '/home/jatavalk/code/SUNCGtoolbox/';
renderedDir = '/home/jatavalk/data/suncg/rendered/';
cameraDir = '/home/jatavalk/data/suncg/rendered/camera_v2/';
pathtogaps = fullfile(suncgToolboxPath, '/gaps/bin/x86_64');
outputDir = '/home/jatavalk/data/suncg/output/';
% Directory to store amodal output (i.e., includes scene voxels outside
% FOV)
outputDir_amodal = '/home/jatavalk/data/suncg/output_amodal';
% Directory to store color images
outputDir_rgb = '/home/jatavalk/data/suncg/output_rgb';
% Directory to store depth images
outputDir_depth = '/home/jatavalk/data/suncg/output_depth';

% Load useful variables
load('suncgObjcategory.mat');
volume_param;

% % SceneID
% sceneId = '0a8b4df06c0b33c5d4c0090fd2384c03';

% Read parameters for data generation
% floorID, roomID, camera pose
cameraFileName = fullfile(cameraDir, sceneId, 'room_camera.txt');
cameraInfoFileName = fullfile(cameraDir, sceneId, 'room_camera_name.txt');

cameraPoseArray = importdata(cameraFileName);
cameraInfoArray = importdata(cameraInfoFileName);
numCameras = min(size(cameraPoseArray,1), size(cameraInfoArray,1));

% cameraId = 1;
% floorId = 0;
% roomId = 4;
% cameraPose = [40.9447 1.57625 38.0651  -0.0947856 -0.196116 -0.975989  -0.0189571 0.980581 -0.195198  0.5534 0.433896  12.3184];
% extCam2World = camPose2Extrinsics(cameraPose);
% extCam2World = [[1 0 0; 0 0 1; 0 1 0]*extCam2World(1:3,1:3)*[1 0 0; 0 0 1; 0 1 0] extCam2World([1,3,2],4)];

timeTaken = 0;
numSamples = 0;


% Process each camera in the scene
for cam = 1:numCameras
    
    disp(['Camera: ', num2str(cam)]);
    
    tic;
    
    cameraId = cam;
    cameraInfo = sscanf(cameraInfoArray{cam}, 'Room#%d_%d_%d');
    floorId = cameraInfo(1);
    roomId = cameraInfo(2);
    cameraPose = cameraPoseArray(cam,:);
    extCam2World = camPose2Extrinsics(cameraPose);
    extCam2World = [[1 0 0; 0 0 1; 0 1 0]*extCam2World(1:3,1:3)*[1 0 0; 0 0 1; 0 1 0] extCam2World([1,3,2],4)];
    
    
    %
    % Get scene voxels
    %
    
    floorId = floorId + 1;
    roomId = roomId + 1;
    pathToData = suncgDataPath;
    
    % Compute voxel range in world coordinates
    voxRangeExtremesCam = [[-voxSizeCam(1:2).*voxUnit/2;0],[-voxSizeCam(1:2).*voxUnit/2;2]+voxSizeCam.*voxUnit];
    voxOriginCam = mean(voxRangeExtremesCam,2);
    
    % Compute voxel grid centers in world coordinates
    voxOriginWorld = extCam2World(1:3,1:3)*voxOriginCam + extCam2World(1:3,4) - [voxSize(1)/2*voxUnit;voxSize(2)/2*voxUnit;voxSize(3)/2*voxUnit];
    voxOriginWorld(3) = height_belowfloor;
    [gridPtsWorldX,gridPtsWorldY,gridPtsWorldZ] = ndgrid(voxOriginWorld(1):voxUnit:(voxOriginWorld(1)+(voxSize(1)-1)*voxUnit), ...
        voxOriginWorld(2):voxUnit:(voxOriginWorld(2)+(voxSize(2)-1)*voxUnit), ...
        voxOriginWorld(3):voxUnit:(voxOriginWorld(3)+(voxSize(3)-1)*voxUnit));
    gridPtsWorld = [gridPtsWorldX(:),gridPtsWorldY(:),gridPtsWorldZ(:)]';
    gridPtsLabel = zeros(1,size(gridPtsWorld,2));
    
    house = loadjson(fullfile(pathToData,'house', sceneId,'house.json'));
    roomStruct = house.levels{floorId}.nodes{roomId};
    floorStruct = house.levels{floorId};
    
    % find all grid in the room
    if exist([fullfile(pathToData,'room',sceneId,roomStruct.modelId) 'f.obj'],'file')
        floorObj = read_wobj([fullfile(pathToData,'room',sceneId,roomStruct.modelId) 'f.obj']);
        inRoom = zeros(size(gridPtsWorldX));
        for i = 1:length(floorObj.objects(3).data.vertices)
            faceId = floorObj.objects(3).data.vertices(i,:);
            floorP = floorObj.vertices(faceId,[1,3])';
            inRoom = inRoom|inpolygon(gridPtsWorldX,gridPtsWorldY,floorP(1,:),floorP(2,:));
        end
        
        % find floor
        floorZ = mean(floorObj.vertices(:,2));
        gridPtsObjWorldInd = inRoom(:)'&(abs(gridPtsWorld(3,:)-floorZ) <= voxUnit/2);
        [~,classRootId] = getobjclassSUNCG('floor',objcategory);
        gridPtsLabel(gridPtsObjWorldInd) = classRootId;
    end
    
    
    
    % find ceiling
    if exist([fullfile(pathToData,'room',sceneId,roomStruct.modelId) 'c.obj'],'file')
        ceilObj = read_wobj([fullfile(pathToData,'room',sceneId,roomStruct.modelId) 'c.obj']);
        ceilZ = mean(ceilObj.vertices(:,2));
        gridPtsObjWorldInd = inRoom(:)'&abs(gridPtsWorld(3,:)-ceilZ) <= voxUnit/2;
        [~,classRootId] = getobjclassSUNCG('ceiling',objcategory);
        gridPtsLabel(gridPtsObjWorldInd) = classRootId;
    end
    
    
    % Load walls
    if exist([fullfile(pathToData,'room',sceneId,roomStruct.modelId) 'w.obj'],'file')
        WallObj = read_wobj([fullfile(pathToData,'room',sceneId,roomStruct.modelId) 'w.obj']);
        inWall = zeros(size(gridPtsWorldX));
        for oi = 1:length(WallObj.objects)
            if WallObj.objects(oi).type == 'f'
                for i = 1:length(WallObj.objects(oi).data.vertices)
                    faceId = WallObj.objects(oi).data.vertices(i,:);
                    floorP = WallObj.vertices(faceId,[1,3])';
                    inWall = inWall|inpolygon(gridPtsWorldX,gridPtsWorldY,floorP(1,:),floorP(2,:));
                end
            end
        end
        gridPtsObjWorldInd = inWall(:)'&(gridPtsWorld(3,:)<ceilZ-voxUnit/2)&(gridPtsWorld(3,:)>floorZ+voxUnit/2);
        [~,classRootId] = getobjclassSUNCG('wall',objcategory);
        gridPtsLabel(gridPtsObjWorldInd) = classRootId;
    end
    
    
    
    % Loop through each object and set voxels to class ID
    if isfield(roomStruct, 'nodeIndices')
        for objId = roomStruct.nodeIndices
            object_struct = floorStruct.nodes{objId+1};
            if isfield(object_struct, 'modelId')
                % Set segmentation class ID
                [classRootName, classRootId] = getobjclassSUNCG(strrep(object_struct.modelId,'/','__'),objcategory);
                
                % Compute object bbox in world coordinates
                objBbox = [object_struct.bbox.min([1,3,2])',object_struct.bbox.max([1,3,2])'];
                
                % Load segmentation of object in object coordinates
                filename= fullfile(pathToData,'object_vox/object_vox_data/',strrep(object_struct.modelId,'/','__'), [strrep(object_struct.modelId,'/','__'), '.binvox']);
                [voxels,scale,translate] = read_binvox(filename);
                [x,y,z] = ind2sub(size(voxels),find(voxels(:)>0));
                objSegPts = bsxfun(@plus,[x,y,z]*scale,translate');
                
                % Convert object to world coordinates
                extObj2World_yup = reshape(object_struct.transform,[4,4]);
                objSegPts = extObj2World_yup*[objSegPts(:,[1,3,2])';ones(1,size(x,1))];
                objSegPts = objSegPts([1,3,2],:);
                
                % Get all grid points within the object bbox in world coordinates
                gridPtsObjWorldInd =      gridPtsWorld(1,:) >= objBbox(1,1) - voxUnit & gridPtsWorld(1,:) <= objBbox(1,2) + voxUnit & ...
                    gridPtsWorld(2,:) >= objBbox(2,1) - voxUnit & gridPtsWorld(2,:) <= objBbox(2,2) + voxUnit & ...
                    gridPtsWorld(3,:) >= objBbox(3,1) - voxUnit & gridPtsWorld(3,:) <= objBbox(3,2) + voxUnit;
                gridPtsObjWorld = gridPtsWorld(:,find(gridPtsObjWorldInd));
                
                
                % If object is a window or door, clear voxels in object bbox
                [~,wallId] = getobjclassSUNCG('wall',objcategory);
                if classRootId == 4 || classRootId == 5
                    gridPtsObjClearInd = gridPtsObjWorldInd&gridPtsLabel==wallId;
                    gridPtsLabel(gridPtsObjClearInd) = 0;
                end
                
                % Apply segmentation to grid points of object
                [indices, dists] = multiQueryKNNSearchImpl(pointCloud(objSegPts'), gridPtsObjWorld',1);
                objOccInd = find(sqrt(dists) <= (sqrt(3)/2)*scale);
                gridPtsObjWorldLinearIdx = find(gridPtsObjWorldInd);
                gridPtsLabel(gridPtsObjWorldLinearIdx(objOccInd)) = classRootId;
            end
        end
    end
    
    % Remove grid points not in field of view
    extWorld2Cam = inv([extCam2World;[0,0,0,1]]);
    gridPtsCam = extWorld2Cam(1:3,1:3)*gridPtsWorld + repmat(extWorld2Cam(1:3,4),1,size(gridPtsWorld,2));
    gridPtsPixX = gridPtsCam(1,:).*(camK(1,1))./gridPtsCam(3,:)+camK(1,3);
    gridPtsPixY = gridPtsCam(2,:).*(camK(2,2))./gridPtsCam(3,:)+camK(2,3);
    invalidPixInd = (gridPtsPixX < 0 | gridPtsPixX >= 640 | gridPtsPixY < 0 | gridPtsPixY >= 480);
    % Before removing invalid indices, also store a full grid
    amodalGridPts = gridPtsLabel;
    % Remove invalid indices
    gridPtsLabel(find(invalidPixInd)) = 0;
    
    % Remove grid points not in the room
    gridPtsLabel(~inRoom(:)&gridPtsLabel(:)==0) = 255;
    amodalGridPts(~inRoom(:)&amodalGridPts(:)==0) = 255;
    
    % Change coordinate axes XYZ -> YZX
    extSwap = [0,1,0; 0,0,1; 1,0,0];
    [gridPtsX,gridPtsY,gridPtsZ] = ind2sub(voxSize,1:size(gridPtsLabel,2));
    gridPts = [gridPtsX(:),gridPtsY(:),gridPtsZ(:)]';
    gridPts = extSwap(1:3,1:3) * gridPts;
    gridPtsLabel(sub2ind(voxSizeTarget,gridPts(1,:)',gridPts(2,:)',gridPts(3,:)')) = gridPtsLabel;
    
    % Change coordinate axes XYZ -> YZX (for amodal grid points)
    [amodalGridPtsX, amodalGridPtsY, amodalGridPtsZ] = ind2sub(voxSize,1:size(amodalGridPts,2));
    amodalGridPtsTmp = [amodalGridPtsX(:), amodalGridPtsY(:), amodalGridPtsZ(:)]';
    amodalGridPtsTmp = extSwap(1:3,1:3) * amodalGridPtsTmp;
    amodalGridPts(sub2ind(voxSizeTarget,amodalGridPtsTmp(1,:)',amodalGridPtsTmp(2,:)',amodalGridPtsTmp(3,:)')) = amodalGridPts;
    
    % Save the volume
    sceneVox = reshape(gridPtsLabel,voxSizeTarget');
    amodalSceneVox = reshape(amodalGridPts, voxSizeTarget');
    % sceneVoxFilename = [depthFilename(1:(end-4)),'.bin'];
    
    % Save usual output (i.e., include all voxels in FOV)
    sceneVoxMatFilename = fullfile(outputDir, sprintf('%08d_%s_fl%03d_rm%04d_0000.png', cameraId-1, sceneId, floorId, roomId));
    sceneVoxMatFilename = [sceneVoxMatFilename(1:(end-4)),'.mat'];
    save(sceneVoxMatFilename,'sceneVox','cameraPose','voxOriginWorld')
    
    % Save amodal output (i.e., include all voxels, even those outside FOV)
    sceneVoxMatFilename = fullfile(outputDir_amodal, sprintf('%08d_%s_fl%03d_rm%04d_0000.png', cameraId-1, sceneId, floorId, roomId));
    sceneVoxMatFilename = [sceneVoxMatFilename(1:(end-4)),'.mat'];
    save(sceneVoxMatFilename,'amodalSceneVox','cameraPose','voxOriginWorld')
    
    % Create a copy of the color and depth image files in their
    % corresponding output dirs, for ease of processing later on.
    rendered_image_directory = fullfile(renderedDir, sceneId, 'images');
    colorimage_src = sprintf('%s/%06d_color.jpg', rendered_image_directory, cameraId-1);
    colorimage_dst = fullfile(outputDir_rgb, sprintf('%08d_%s_fl%03d_rm%04d_0000.jpg', cameraId-1, sceneId, floorId, roomId));
    depthimage_src = sprintf('%s/%06d_depth.png', rendered_image_directory, cameraId-1);
    depthimage_dst = fullfile(outputDir_depth, sprintf('%08d_%s_fl%03d_rm%04d_0000.png', cameraId-1, sceneId, floorId, roomId));
    copyfile(colorimage_src, colorimage_dst);
    copyfile(depthimage_src,  depthimage_dst);
    
    curTime = toc;
    disp(['Time taken: ', num2str(curTime)]);
    timeTaken = timeTaken + curTime;
    numSamples = numSamples + 1;
    
    
    %     %
    %     % Visualize
    %     %
    %
    %     volume_param;
    %     extWorld2Cam = inv([extCam2World;[0,0,0,1]]);
    %     [gridPtsWorldX,gridPtsWorldY,gridPtsWorldZ] = ndgrid(voxOriginWorld(1):voxUnit:(voxOriginWorld(1)+(voxSize(1)-1)*voxUnit), ...
    %                                                          voxOriginWorld(2):voxUnit:(voxOriginWorld(2)+(voxSize(2)-1)*voxUnit), ...
    %                                                          voxOriginWorld(3):voxUnit:(voxOriginWorld(3)+(voxSize(3)-1)*voxUnit));
    %
    %     gridPtsCam = extWorld2Cam(1:3,1:3)*[gridPtsWorldX(:),gridPtsWorldY(:),gridPtsWorldZ(:)]' + repmat(extWorld2Cam(1:3,4),1,prod(voxSize));
    %
    %     voxPath = fullfile(outputDir, sceneId, 'depthVox');
    %     % Output image directory for the current scene
    %     rendered_image_directory = fullfile(renderedDir, sceneId, 'images');
    %
    %     % Load the raw depth file for the current frame
    %     depthFilename = fullfile(voxPath, sprintf('%08d_%s_fl%03d_rm%04d_0000.png', cameraId-1, sceneId, floorId, roomId));
    %     sceneVoxFilename = [depthFilename(1:(end-4)),'.bin'];
    %     depthRaw = double(imread(sprintf('%s/%06d_depth.png', rendered_image_directory, cameraId-1)))/1000;
    %
    %     % Project grid to 2D camera image frame (use 1-indexing for Matlab)
    %     gridPtsPixX = round((camK(1,1)*gridPtsCam(1,:)./gridPtsCam(3,:))+camK(1,3))+1;
    %     gridPtsPixY = round((camK(2,2)*gridPtsCam(2,:)./gridPtsCam(3,:))+camK(2,3))+1;
    %     depthInpaint = depthRaw;
    %     validDepthrane = [1,1,size(depthRaw)];
    %     validPix = find(gridPtsPixX > validDepthrane(2) & gridPtsPixX <= validDepthrane(4) & ...
    %                     gridPtsPixY > validDepthrane(1) & gridPtsPixY <= validDepthrane(3));
    %     outsideFOV = gridPtsPixX <= validDepthrane(2) | gridPtsPixX > validDepthrane(4) | ...
    %                  gridPtsPixY <= validDepthrane(1) | gridPtsPixY > validDepthrane(3);
    %     gridPtsPixX = gridPtsPixX(validPix);
    %     gridPtsPixY = gridPtsPixY(validPix);
    %     gridPtsPixDepth = depthInpaint(sub2ind(size(depthInpaint),gridPtsPixY',gridPtsPixX'))';
    %     validDepth = find(gridPtsPixDepth > 0);
    %     missingDepth = find(gridPtsPixDepth == 0);
    %     gridPtsPixDepth = gridPtsPixDepth(validDepth);
    %     validGridPtsInd = validPix(validDepth);
    %
    %     % Get depth difference
    %     ptDist = (gridPtsPixDepth-gridPtsCam(3,validGridPtsInd))*...
    %               sqrt(1+(gridPtsCam(1,validGridPtsInd)/gridPtsCam(3,validGridPtsInd)).^2+(gridPtsCam(2,validGridPtsInd)/gridPtsCam(3,validGridPtsInd)).^2);
    %
    %     % Compute TSDF
    %     distance = nan(voxSize(1),voxSize(2),voxSize(3));
    %     distance(validPix(validDepth)) = ptDist;
    %     distance = permute(distance,[2,3,1]);
    %     show_volume(abs(distance)<0.1)
    %     hold on;
    %     show_volume(sceneVox);
    %     view(180, 270);
    %     figure;
    %     show_volume(amodalSceneVox);
    %     view(180, 270);
    %
    %     pause(2);
    
    
    % End of for loop over numCameras
end

disp(['Total time:', num2str(timeTaken)]);
disp(['Avg. time per sample:', num2str(timeTaken/numSamples)]);


% End of function dataPrep
end






