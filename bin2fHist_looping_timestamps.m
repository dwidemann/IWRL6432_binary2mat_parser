%Convert 3D PC UAR binary stream file to FHIST

%% Setup Workspace

close all;
clear all;

% TO DO: fix the timestamps so that every frame has one, instead of every
% 100th frame. 

%% Set file paths CHANGE THESE TO MATCH YOUR SETUP
binFilePath = "/Users/davidwidemann/Documents/sf-iwrl6432-data-collects/troublemakers-whiteboard/08_22_2024_09_10_05";
splitStrings = split(binFilePath, filesep);
output_dir = append("testing_all_mat_files_", splitStrings(length(splitStrings)));
mkdir(output_dir);

binFileName = "pHistBytes";

fileList = dir(binFilePath);
fileList = fileList(~ismember({fileList.name}, {'.', '..'}));
num_binary_files = 0;
for i = 1:length(fileList)
    if strcmp(fileList(i).name(end-2:end),'bin')
        num_binary_files = 1 + num_binary_files;
    end
end

max_binary2mat_file = 360;
num_mat_files = ceil(num_binary_files/max_binary2mat_file);
lostSyncFrames = [];
lostSyncFiles = [];

%% Set UART TLV Structures


syncPatternUINT64 = typecast(uint16([hex2dec('0102'),hex2dec('0304'),hex2dec('0506'),hex2dec('0708')]),'uint64');
syncPatternUINT8 = typecast(uint16([hex2dec('0102'),hex2dec('0304'),hex2dec('0506'),hex2dec('0708')]),'uint8');

frameHeaderStructType = struct(...
    'sync',             {'uint64', 8}, ... % See syncPatternUINT64 below
    'version',          {'uint32', 4}, ...
    'packetLength',     {'uint32', 4}, ... % In bytes, including header
    'platform',         {'uint32', 4}, ...
    'frameNumber',      {'uint32', 4}, ... % Starting from 1
    'timeStamp',        {'uint32', 4},...
    'numDetectedObj',   {'uint32', 4}, ... % Number of detected points in this frame
    'numTLVs' ,         {'uint32', 4}, ... % Number of TLVs in this frame
    'subFrameNumber',   {'uint32', 4}); ... % Sub-Frame number

tlvHeaderStruct = struct(...
    'type',             {'uint32', 4}, ... % TLV object Type
    'length',           {'uint32', 4});    % TLV object Length, in bytes, including TLV header

% Point Cloud TLV reporting unit for all reported points
pointUintStruct = struct(...
    'elevUnit',             {'float', 4}, ... % elevation, in rad
    'azimUnit',             {'float', 4}, ... % azimuth, in rad
    'dopplerUnit',          {'float', 4}, ... % Doplper, in m/s
    'rangeUnit',            {'float', 4}, ... % Range, in m
    'snrUnit',              {'float', 4});    % SNR, ratio

% Point Cloud TLV reporting unit for all reported points
pointUnit6432Struct = struct(...
    'xyzUnit',                  {'float', 4}, ... % elevation, in rad
    'dopplerUnit',              {'float', 4}, ... % azimuth, in rad
    'snrUnit',                  {'float', 4}, ... % Doplper, in m/s
    'noiseUnit',                {'float', 4}, ... % Range, in m
    'numDetectedPoints0',        {'uint16', 2}, ...    % SNR, ratio
    'numDetectedPoints1',        {'uint16', 2});    % SNR, ratio

% Point Cloud TLV object consists of an array of points.
% Each point has a structure defined below
pointStruct = struct(...
    'elevation',        {'int8', 1}, ... % elevation, in rad
    'azimuth',          {'int8', 1}, ... % azimuth, in rad
    'doppler',          {'int16', 2}, ... % Doplper, in m/s
    'range',            {'uint16', 2}, ... % Range, in m
    'snr',              {'uint16', 2});    % SNR, ratio

point6432Struct = struct(...
    'x',                {'int16', 2}, ... % x in units given by pointstruct
    'y',                {'int16', 2}, ... % y in units given by pointstruct
    'z',                {'int16', 2}, ... % z in units given by pointstruct
    'doppler',          {'int16', 2}, ... % doppler in units given by pointstruct
    'snr',              {'int8', 1},  ... % snr in units given by pointstruct
    'noise',            {'int8', 1}); % noise in units given by pointstruct
% Target List TLV object consists of an array of targets.
% Each target has a structure define below
targetStruct = struct(...
    'tid',              {'uint32', 4}, ... % Track ID
    'posX',             {'float', 4}, ...   % Target position in X dimension, m
    'posY',             {'float', 4}, ...   % Target position in Y dimension, m
    'posZ',             {'float', 4}, ...   % Target position in Z dimension, m
    'velX',             {'float', 4}, ...   % Target velocity in X dimension, m/s
    'velY',             {'float', 4}, ...   % Target velocity in Y dimension, m/s
    'velZ',             {'float', 4}, ...   % Target velocity in Z dimension, m/s
    'accX',             {'float', 4}, ...   % Target acceleration in X dimension, m/s2
    'accY',             {'float', 4}, ...   % Target acceleration in Y dimension, m/s
    'accZ',             {'float', 4}, ...   % Target acceleration in Z dimension, m/s
    'EC',               {'float', 16*4}, ...% Tracking error covariance matrix, [4x4], in range/angle/doppler coordinates
    'G',                {'float', 4}, ...   % Gating function gain
    'confidence',       {'float', 4});      % Confidence level
% Presence Detection TLV object consists of single uint32 value.
% Each target has a structure define below
presenceStruct = struct(...
    'presence',         {'uint32', 4});     % Presence detection

targetIndex = struct(...
'targetID',         {'uint8', 1});    % Track ID

% Classifier output an array of target ID and TAGs.
% Each target has a structure define below
classifierOutStruct = struct(...
    'tid',              {'uint32', 4}, ... % Track ID
    'tag',              {'uint32', 4});     % Target tag, 1 -- HUMAN, -1 -- moving clutter


% TLV types
MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS 					= 301; % Implemented
MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MAJOR              = 302; % Implemented
MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MINOR              = 303; % Implemented
MMWDEMO_OUTPUT_EXT_MSG_RANGE_AZIMUT_HEAT_MAP_MAJOR      = 304; % Implemented
MMWDEMO_OUTPUT_EXT_MSG_RANGE_AZIMUT_HEAT_MAP_MINOR      = 305; % Implemented
MMWDEMO_OUTPUT_EXT_MSG_STATS                            = 306; % Implemented
MMWDEMO_OUTPUT_EXT_MSG_PRESENCE_INFO                    = 307; % Implemented
MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST                      = 308; % Implemented
MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX                     = 309; % Implemented

MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_RAW_DATA           = 310; % Not implemented
MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_FEATURES           = 311; % Implemented
MMWDEMO_OUTPUT_EXT_MSG_RADAR_CUBE_MAJOR                 = 312; % Not implemented
MMWDEMO_OUTPUT_EXT_MSG_RADAR_CUBE_MINOR                 = 313; % Not implemented
MMWDEMO_OUTPUT_EXT_MSG_POINT_CLOUD_INDICES              = 314; % Not implemented

MMWDEMO_OUTPUT_EXT_MSG_ENHANCED_PRESENCE_INDICATION     = 315; % Not implemented
MMWDEMO_OUTPUT_EXT_MSG_ADC_SAMPLES                      = 316; % Not implemented
MMWDEMO_OUTPUT_EXT_MSG_CLASSIFIER_INFO                  = 317; % Not implemented
MMWDEMO_OUTPUT_EXT_MSG_RX_CHAN_COMPENSATION_INFO        = 318; % Not implemented

    
MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_FLOAT                = 1; % Not tested
MMWDEMO_OUTPUT_MSG_RANGE_PROFILE                        = 2; % Not tested
MMWDEMO_OUTPUT_MSG_NOISE_PROFILE                        = 3; % Not tested
MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP               = 4; % Not tested
MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP               = 5; % Not tested
MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO_FLOAT      = 7; % Not tested
MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP     = 8; % Not tested
MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS                    = 9; % Not tested

MMWDEMO_OUTPUT_EXT_MSG_VITALSPC                         = 1041;% Not implemented

frameHeaderLengthInBytes = lengthFromStruct(frameHeaderStructType);
tlvHeaderLengthInBytes = lengthFromStruct(tlvHeaderStruct);
pointLengthInBytes = lengthFromStruct(pointStruct);
point6432LengthInBytes = lengthFromStruct(point6432Struct);
pointUnitLengthInBytes = lengthFromStruct(pointUintStruct);
pointUnit6432LengthInBytes = lengthFromStruct(pointUnit6432Struct);
targetLengthInBytes = lengthFromStruct(targetStruct);
classifierOutInBytes = lengthFromStruct(classifierOutStruct);
indexLengthInBytes = 1;
presenceLengthInBytes = lengthFromStruct(presenceStruct);

% Allocate for 3D State Space Vectors
dSize = 3; sSize = 9; mSize = 4;
maxNumTracks = 20;
microDopplerSlidWinLen = 10;
classifierEnabled = 1;

fileFrameSize = 100;
fileNumber = 1;
frameStatStruct = ...
    struct('targetFrameNum', [], 'header', [], 'bytes', [], 'bytesAvailable', [], 'packetRead', 0, 'numInputPoints', 0, 'numOutputPoints', 0, 'timestamp', 0, ...
    'start', 0, 'benchmarks', zeros(10,1), 'done', 0, 'numPoints', 0, 'pointCloud', [], 'posLocal', [], 'posWorld', [], 'targetList', [], 'indexArray', [],...
    'rangeAzimuthHeatMapMajor', [], 'rangeAzimuthHeatMapMinor', [], 'rngProfileMajor', [], 'rngProfileMinor', [], ...
    'numPointsMinor', 0, 'pointCloudMinor', [], 'posLocalMinor', [], 'posWorldMinor', [], ...
    'sensorTimingInfo', [], 'sensorTemperatureInfo', [], 'sensorPowerInfo', [], 'microDopplerFeaturesBuf', []);
frameStatCell = struct2cell(frameStatStruct);
frameStatNames = fieldnames(frameStatStruct);


previousPointCloud = [];
previousPoint3D = [];
point3D_W = [];
rngProfileMajor = [];
rngProfileMinor = [];
timestamp = [];
motionStatePerZoneTarget = [];

azimuthTilt = 0; % Can be configured according to .cfg file and setup
elevationTilt = 0; % Can be configured according to .cfg file and setup
scene.RotZ_TW = [cos(azimuthTilt) sin(azimuthTilt) 0; -sin(azimuthTilt) cos(azimuthTilt) 0; 0 0 1];
scene.RotX_TW = [1 0 0; 0 cos(elevationTilt) sin(elevationTilt); 0 -sin(elevationTilt) cos(elevationTilt)];
scene.sensorPos = [0 0 0]; % XYZ of sensor in world coordinates. Can be configured according to .cfg file and setup
Params.dataPath.numDopplerBins = 64;% Can be configured according to .cfg file and setup


activeTracks = zeros(1, maxNumTracks);
numFeaturesPerTarget = 6;
microDopplerHeatmap    = zeros(Params.dataPath.numDopplerBins,microDopplerSlidWinLen,maxNumTracks);
microDopplerFreqLow    = zeros(microDopplerSlidWinLen,maxNumTracks);
microDopplerFreqUp     = zeros(microDopplerSlidWinLen,maxNumTracks);
microDopplerFreqMedian = zeros(microDopplerSlidWinLen,maxNumTracks);
microDopplerFreqMean   = zeros(microDopplerSlidWinLen,maxNumTracks);
microDopplerBwPwr      = zeros(microDopplerSlidWinLen,maxNumTracks);
microDopplerEntropy    = zeros(microDopplerSlidWinLen,maxNumTracks);


% Classifier parameters
if (classifierEnabled == 1)
    % Label index
    classLabels = [-1; 1];

    % Confidence score (score should be greater than this)
    classifierConfidenceScore = 0.6;

    % Block length
    classifierBlkLen = 30;

    % Minimal velocity of the human target to enable classification, m/s (default 0 to disable)
    classificationMinHumanVelocity = 0.3;

    % init the tags buffer
    tagTemporalFiltSize = 5;
    targetClassStruct.tag = zeros(1,tagTemporalFiltSize); % initialize the tag to unknown
    targetClassStruct = repmat(targetClassStruct, 1, maxNumTracks);
    maxAllowedUnknowns = 1;
    
    % Prediction history
    predictionHistStruct = struct('targetPredictionsBuf', 0.5*ones(classifierBlkLen,2), 'matlabPredictionsBuf', 0.5*ones(classifierBlkLen,2));
    predictionHist = repmat(predictionHistStruct, 1, maxNumTracks);
end



lostSync = 0;

%% Open File and Parse

% fid = fopen(binFileName, 'r', 'l'); %open the bin file for reading in little endian
% magicWord = [258;772;1286;1800]; %0x01020304
% 
% 
% dataStream = fread(fid,4,'uint16');
% 
% if (dataStream == magicWord)
%     frameHeader = fread(fid,40,'uint8');
% end
% disp(frameHeader);

%% Open and read using method from visualizer

for output_file_num = 1 : num_mat_files
    frameStatStruct = cell2struct(frameStatCell,frameStatNames);
    fHist = repmat(frameStatStruct, 1, fileFrameSize);
    start_idx = 1 + (output_file_num - 1)*max_binary2mat_file;
    end_idx = min(output_file_num*max_binary2mat_file, num_binary_files);
    row_idx = 1;

    for file_num = start_idx : end_idx
        thisBinFileName = binFilePath + filesep + binFileName + '_' + string(file_num) + ".bin";
        fid = fopen(thisBinFileName, 'r', 'l'); %open the bin file for reading in little endian
        %Read the header first
        frameNum = 0;
        [timestamp] = char(fread(fid, 19, 'uint8'));
        timestamp = convertCharsToStrings(timestamp);
        if file_num == start_idx
            outfile_name = timestamp;
        end
        while ~feof(fid) && (mod(frameNum, 100) ~= 0 || frameNum == 0)
    
            [rxHeader, byteCount] = fread(fid, frameHeaderLengthInBytes, 'uint8');
    
            magicBytes = typecast(uint8(rxHeader(1:8)), 'uint64');
            if(magicBytes ~= syncPatternUINT64)
                reason = 'No SYNC pattern';
                lostSync = 1;
                %[rxDataDebug, byteCountDebug] = fread(fid, bytesAvailable - frameHeaderLengthInBytes, 'uint8');            
                break;
            end       
        %     if(validateChecksum(rxHeader) ~= 0)
        %         reason = 'Header Checksum is wrong';
        %         lostSync = 1;
        %         break; 
        %     end
    
            frameHeader = readToStruct(frameHeaderStructType, rxHeader);
            %disp(frameHeader);
    
            % if((frameHeader.frameNumber >1) && mod(frameHeader.frameNumber,fileFrameSize) == 1)
            %     fileNumber = floor(frameHeader.frameNumber / 100);
            %     tempfhistSaveName = strcat(fhistSaveName , int2str(fileNumber), '.mat');
            %     save(tempfhistSaveName,'fHist'); 
            %     fileNumber = fileNumber + 1;
            % end
    
            % We have a valid header
            targetFrameNum = frameHeader.frameNumber;
            frameNum = frameHeader.frameNumber;
    
            fHist(row_idx).targetFrameNum = targetFrameNum;
            fHist(row_idx).header = frameHeader;
    
            dataLength = frameHeader.packetLength - frameHeaderLengthInBytes;
    
            fHist(row_idx).bytes = dataLength; 
            numInputPoints = 0;
            numInputPointsMajor = 0;
            numInputPointsMinor = 0;            
            rangeAzimuthHeatMapMajor = [];
            rangeAzimuthHeatMapMinor = [];
            rngProfileMajor = [];
            rngProfileMinor = [];
            adcSamples = [];
            sensorTimingInfo = [];
            sensorTemperatureInfo = [];
            sensorPowerInfo = [];
            radarCubeMajor = [];
            radarCubeMinor = [];
            microDopplerFeaturesBuf = [];
            
            numTargets = 0;
            TID = zeros(1,numTargets);
            S = zeros(sSize, numTargets);
            EC = zeros(mSize*mSize, numTargets);
            G = zeros(1,numTargets);  
            Conf = zeros(1,numTargets);
            tPos = zeros(dSize,numTargets);
            presence = 0;
            mIndex = [];
            
            if(dataLength > 0)
            %     while(bytesAvailable < dataLength)
            %         pause(0.01);
            %         count2 = count2 + 1;
            %         bytesAvailable = get(hDataSerialPort,'BytesAvailable');
            %     end
    
                %Read all packet
                [rxData, byteCount] = fread(fid, double(dataLength), 'uint8');
                if(byteCount ~= double(dataLength))
                    reason = 'Data Size is wrong'; 
                    lostSync = 1;
                    %break; 
                end
                offset = 0;
    
            %     fHist(frameNum).benchmarks(1) = 1000*toc(frameStart);
            end
    
            % TLV Parsing
            for nTlv = 1:frameHeader.numTLVs
                tlvType = typecast(uint8(rxData(offset+1:offset+4)), 'uint32');
                tlvLength = typecast(uint8(rxData(offset+5:offset+8)), 'uint32');
                if(tlvLength + offset > dataLength)
                    reason = 'TLV Size is wrong';
                    lostSync = 1;
                    break;                    
                end
                offset = offset + tlvHeaderLengthInBytes;
                valueLength = tlvLength;
                switch(tlvType)
    
                    case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_FLOAT
                            % Get the unit scale for each point cloud dimension
                            numInputPoints = (valueLength)/pointStructXYZLengthinBytes;
                            if(numInputPoints > 0)    
                                % Get Point Cloud from the sensor
                                pointCloudXYZTemp_float = typecast(typecast(uint8(rxData(offset+1: offset+valueLength)),'uint8'), 'single');
                                x = pointCloudXYZTemp_float(1:4:(numInputPoints*4));
                                y = pointCloudXYZTemp_float(2:4:(numInputPoints*4));
                                z = pointCloudXYZTemp_float(3:4:(numInputPoints*4));
                                doppler = pointCloudXYZTemp_float(4:4:(numInputPoints*4));
                                % Convert to polar 
                                range = double(sqrt(x.*x+y.*y+z.*z));
                                elev = double(asin(z./range));
                                azim = double(asin(x./(range.*cos(elev))));
                                offset = offset + valueLength;
                            end
    
                            %Currently we do not have information to which mode the points belong to
                            if enableMajorMotion
                                %Major motion or both
                                numInputPointsMajor = numInputPoints;
                                numInputPointsMinor = 0;
                            else
                                %Only Minor motion
                                numInputPointsMajor = 0;
                                numInputPointsMinor = numInputPoints;
                            end
                            % Point cloud type
                            pointCloudType = [PC_MAJOR*ones(1,numInputPointsMajor) PC_MINOR*ones(1,numInputPointsMinor)];
    
    
    
                    case MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS
                        % Point Cloud TLV
                        % Get the unit scale for each point cloud dimension
                        pointUnit = typecast(uint8(rxData(offset+1: offset+pointUnit6432LengthInBytes-4)),'single');
                        xyzUnit = pointUnit(1);
                        dopplerUnit = pointUnit(2);
                        snrUnit = pointUnit(3);
                        noiseUnit = pointUnit(4);
                        numInputPts = typecast(uint8(rxData(offset+(pointUnit6432LengthInBytes-4)+1 : offset+pointUnit6432LengthInBytes)),'uint16');
                        numInputPointsMajor = numInputPts(1);
                        numInputPointsMinor = numInputPts(2);
    
                        offset = offset + pointUnit6432LengthInBytes;
    
                        numInputPoints = (valueLength - pointUnit6432LengthInBytes)/point6432LengthInBytes;
                        if(numInputPoints > 0)    
                            % Get Point Cloud from the sensor
                            pointCloudTemp = typecast(uint8(rxData(offset+1: offset+valueLength - pointUnit6432LengthInBytes)),'uint8');
                            
                            %Received points in cartesian coordinates
                            pointCloudTemp = reshape(pointCloudTemp, point6432LengthInBytes, numInputPoints);
                            
                            x = xyzUnit * double(typecast(reshape(pointCloudTemp(1:2,:), 2*numInputPoints, 1),'int16'));
                            y = xyzUnit * double(typecast(reshape(pointCloudTemp(3:4,:), 2*numInputPoints, 1),'int16'));
                            z = xyzUnit * double(typecast(reshape(pointCloudTemp(5:6,:), 2*numInputPoints, 1),'int16'));
                            doppler = dopplerUnit * double(typecast(reshape(pointCloudTemp(7:8,:), 2*numInputPoints, 1),'int16'));
                            snr = snrUnit * double(typecast(reshape(pointCloudTemp(9,:), numInputPoints, 1),'uint8'));
                            noise = noiseUnit * double(typecast(reshape(pointCloudTemp(10,:), numInputPoints, 1),'uint8'));
                            
                            range = double(sqrt(x.*x+y.*y+z.*z));
                            elev = real(double(asin(z./range))); % ignore the precision errors
                            azim = real(double(asin(x./(range.*cos(elev))))); % ignore the precision errors
                            
                            % Fill the point cloud
                            pointCloud = [double(x) double(y) double(z) double(doppler) double(snr) double(noise)]';
    %                         point3D_T = [double(x) double(y) double(z) ]';
    % 
    %                         % Point cloud type
    %                         pointCloudType = [PC_MAJOR*ones(1,numInputPointsMajor) PC_MINOR*ones(1,numInputPointsMinor)];
    % 
    %                         % Rotate to sensor orientation and translate to sensor position
    %                         point3D_W = scene.RotZ_TW*scene.RotX_TW*point3D_T;
    %                         point3D_W = point3D_W + scene.sensorPos.';
                        end                        
                        offset = offset + valueLength - pointUnit6432LengthInBytes;
    
                        
                    case MMWDEMO_OUTPUT_EXT_MSG_RANGE_AZIMUT_HEAT_MAP_MAJOR
                        rangeAzimuthHeatMapMajor = double(typecast(uint8(rxData(offset+1: offset+valueLength)),'uint32'));
                        rangeAzimuthHeatMapMajor = reshape(rangeAzimuthHeatMapMajor,...
                                                            [16,... % Size of azimuth FFT (Needs to be adjusted for each cfg file)
                                                            64]);  % Number of range bins (Needs to be adjusted for each cfg file)
                        offset = offset + valueLength;
                        
                    case MMWDEMO_OUTPUT_EXT_MSG_RANGE_AZIMUT_HEAT_MAP_MINOR
                        rangeAzimuthHeatMapMinor = double(typecast(uint8(rxData(offset+1: offset+valueLength)),'uint32'));
                        rangeAzimuthHeatMapMinor = reshape(rangeAzimuthHeatMapMinor,...
                                                            [16,... % Size of azimuth FFT (Needs to be adjusted for each cfg file)
                                                            64]);  % Number of range bins (Needs to be adjusted for each cfg file)
                        offset = offset + valueLength;
                        
                    case MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MAJOR
                        rngProfileMajor = double(typecast(uint8(rxData(offset+1: offset+valueLength)),'uint32'));
                        offset = offset + valueLength;
    
                    case MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MINOR
                        rngProfileMinor = double(typecast(uint8(rxData(offset+1: offset+valueLength)),'uint32'));
                        offset = offset + valueLength;
    
                    case MMWDEMO_OUTPUT_EXT_MSG_STATS
                        %Read timing: two uint32 values (8-Bytes)  (1LSB = 1msec)
                        msgTemp = double(typecast(uint8(rxData(offset+1: offset+8)),'uint32'));
                        sensorTimingInfo.interFrameProcessingTime_usec = msgTemp(1);
                        sensorTimingInfo.transmitOutputTime_usec = msgTemp(2);
                        offset = offset + 8;
    
                        %Read power: Four uint16 values (8-Bytes), 4 Power Rails: 1.8V, 3.3V, 1.2V and 1.2V RF
                        powerMeasured = double(typecast(uint8(rxData(offset+1: offset+8)),'uint16'));
                        sensorPowerInfo.meanPower = sum(powerMeasured) * 0.1; %received values are in 100uW (1LSB = 100uW)
                        offset = offset + 8;
    
                        %Read Temperature: Four int16 values, (1LSB = 1 deg C) temperature sensors sensors are for Rx, Tx, PM, DIG
                        temperatureMeasured = double(typecast(uint8(rxData(offset+1: offset+8)),'int16'));
                        sensorTemperatureInfo.rfeTemperature = round(mean(temperatureMeasured(1:3)));  %ToDo: We average RX, TX and PM ?
                        sensorTemperatureInfo.digitalTemperature = temperatureMeasured(4);
                        offset = offset + 8;
    
                    case MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS
                        msgTemp = (typecast(uint8(rxData(offset+1: offset+valueLength)),'int32'));
                        sensorTemperatureInfo.rfeTemperature = msgTemp(1);
                        sensorTemperatureInfo.digitalTemperature = msgTemp(2);
                        offset = offset + valueLength;
                    
                    case MMWDEMO_OUTPUT_EXT_MSG_ENHANCED_PRESENCE_INDICATION
                        % Read the number of zones
                        msgTemp = (typecast(uint8(rxData(offset+1: offset+valueLength)),'uint8'));
                        numPresenceZones = double(msgTemp(1));
                        presenceResultRaw = msgTemp(2:end);
                        offset = offset + valueLength;
    
                        motionStatePerZoneTarget = zeros(1,length(presenceResultRaw)*4);
                        for n = 1:length(presenceResultRaw)
                            % Get the bits and partition into 2-bit values
                            presenceBitVector = double(reshape(bitget(presenceResultRaw(n),1:8),2,[]));
                            % Convert bits to flag values
                            motionStatePerZoneTarget(4*n-3:4*n) = 2.^(0:1) * presenceBitVector;
                        end
    
                        % Ignore the unused zones
                        motionStatePerZoneTarget = motionStatePerZoneTarget(1:numPresenceZones);
    
                    case MMWDEMO_OUTPUT_EXT_MSG_ADC_SAMPLES
                        adcSamples = double(typecast(uint8(rxData(offset+1: offset+valueLength)),'int16'));
                        adcSamples = reshape(adcSamples,[], Params.dataPath.numTxAnt*Params.dataPath.numRxAnt).';
                        offset = offset + valueLength;
                    
                    case MMWDEMO_OUTPUT_EXT_MSG_CLASSIFIER_INFO
                        targetPredictions = double(typecast(uint8(rxData(offset+1: offset+valueLength)),'uint8'));
                        targetPredictions = reshape(targetPredictions,2,[]).'; %Row: [prediction[0] prediction[1]]
                        targetPredictions = targetPredictions(:,1:2)/128; %Predictions received in Q7 format
                        offset = offset + valueLength;
                        
                        for n=1:size(targetPredictions,1)
                            % Track IDs are aligned with the tracker
                            tid = TID(n)+1;
                            predictionHist(tid).targetPredictionsBuf = circshift(predictionHist(tid).targetPredictionsBuf, -1, 1);
                            predictionHist(tid).targetPredictionsBuf(end,:) = targetPredictions(n,:);
                        end
    
                    case MMWDEMO_OUTPUT_EXT_MSG_RX_CHAN_COMPENSATION_INFO
                        rcChcomp = double(typecast(uint8(rxData(offset+1: offset+valueLength)),'single'));
                        rxChanCompText = sprintf('compRangeBiasAndRxChanPhase %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n', rcChcomp);
                        offset = offset + valueLength;
                    
                    case MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST
                        % Target List TLV
                        numTargets = valueLength/targetLengthInBytes;                        
                        TID = zeros(1,numTargets);
                        S = zeros(sSize, numTargets);
                        EC = zeros(mSize*mSize, numTargets);
                        G = zeros(1,numTargets);  
                        Conf = zeros(1,numTargets);
                        tPos = zeros(dSize,numTargets);
                        for n=1:numTargets
                            TID(n)  = typecast(uint8(rxData(offset+1:offset+4)),'uint32');      %1x4=4bytes
                            offset = offset + 4;
                            S(:,n)  = typecast(uint8(rxData(offset+1:offset+36)),'single');     %9x4=36bytes
                            offset = offset + 36;
                            EC(:,n) = typecast(uint8(rxData(offset+1:offset+64)),'single');     %4x4x4=64bytes
                            offset = offset + 64;
                            G(n)    = typecast(uint8(rxData(offset+1:offset+4)),'single');      %1x4=4bytes
                            offset = offset + 4;
                            Conf(n) = typecast(uint8(rxData(offset+1:offset+4)),'single');      %1x4=4bytes
                            offset = offset + 4;
                            tPos(:,n) = scene.RotZ_TW*scene.RotX_TW*S(1:3,n);
                            tPos(3,n) = tPos(3,n) + scene.sensorPos(3);
                        end
                    
                    case MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX
                        % Target Index TLV
                        numIndices = valueLength/indexLengthInBytes;
                        mIndex = typecast(uint8(rxData(offset+1:offset+numIndices)),'uint8');
                        offset = offset + valueLength;
                    
                    case MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_RAW_DATA
                        microDopplerBuf = typecast(uint8(rxData(offset+1:offset+valueLength)),'single');
                        offset = offset + valueLength;
                        microDopplerBuf = reshape(microDopplerBuf, Params.dataPath.numDopplerBins, numTargets);
                        
                        for n = 1:numTargets
                            % Track IDs are aligned with the tracker
                            tid = TID(n)+1;
    
                            % Circularly shift the buffer
                            microDopplerHeatmap(:,:,tid) = circshift(microDopplerHeatmap(:,:,tid),-1,2);
    
                            % Fill the buffer with the new data
                            microDopplerHeatmap(:,end,tid) = microDopplerBuf(:,n);
                        end
                        
                    case MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_FEATURES
                        microDopplerFeaturesBuf = typecast(uint8(rxData(offset+1:offset+valueLength)),'single');
                        offset = offset + valueLength;
                        microDopplerFeaturesBuf = reshape(microDopplerFeaturesBuf, numFeaturesPerTarget, numTargets);
                        
                        for n = 1:numTargets
                            % Track IDs are aligned with the tracker
                            tid = TID(n)+1;
    
                            % Circularly shift the buffer
                            microDopplerFreqLow(:,tid)    = circshift(microDopplerFreqLow(:,tid),-1,1);
                            microDopplerFreqUp(:,tid)     = circshift(microDopplerFreqUp(:,tid),-1,1);
                            microDopplerFreqMean(:,tid)   = circshift(microDopplerFreqMean(:,tid),-1,1);
                            microDopplerFreqMedian(:,tid) = circshift(microDopplerFreqMedian(:,tid),-1,1);
                            microDopplerBwPwr(:,tid)      = circshift(microDopplerBwPwr(:,tid),-1,1);
                            microDopplerEntropy(:,tid)    = circshift(microDopplerEntropy(:,tid),-1,1);
                            
                            % Fill the buffer with the new data
                            microDopplerFreqLow(end,tid)    = microDopplerFeaturesBuf(1,n);
                            microDopplerFreqUp(end,tid)     = microDopplerFeaturesBuf(2,n);
                            microDopplerFreqMean(end,tid)   = microDopplerFeaturesBuf(4,n);
                            microDopplerFreqMedian(end,tid) = microDopplerFeaturesBuf(5,n);
                            microDopplerBwPwr(end,tid)      = microDopplerFeaturesBuf(3,n);
                            microDopplerEntropy(end,tid)    = microDopplerFeaturesBuf(6,n);
                        end
                        
                    case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO_FLOAT
                        num_points_in_TLV = valueLength/pointStructXYZ_sideinfo_LengthinBytes;
                        if (num_points_in_TLV >0)
                            % copy the points to a structure 
                            pointCloudXYZTemp_sideinfo = typecast (typecast(uint8(rxData(offset+1: offset+valueLength)),'uint8'), 'uint16');
                            snr = single(pointCloudXYZTemp_sideinfo(1:2:(num_points_in_TLV*2)));
                            noise = single(pointCloudXYZTemp_sideinfo(2:2:(num_points_in_TLV*2)));
                        end      
                        pointCloud = [range azim elev double(doppler) double(snr)]';
                        point3D_T = [double(x) double(y) double(z) ]';
    
                        % Rotate to sensor orientation and translate to sensor position
                        point3D_W = scene.RotZ_TW*scene.RotX_TW*point3D_T;
                        point3D_W = point3D_W + scene.sensorPos.';
                        offset = offset + valueLength;
                    
                    case MMWDEMO_OUTPUT_EXT_MSG_VITALSPC
                        pDataInVitals = typecast(uint8(rxData(offset+1:offset+valueLength)),'int16');
                        offset = offset + valueLength;
                        
                        % Update the real-imaginary sequence
                        pDataInVitals = single(pDataInVitals);
                        pDataInVitals = reshape(pDataInVitals,2,[]);
                        pDataInVitals = flip(pDataInVitals,1);
                        pDataInVitals = reshape(pDataInVitals,1,[]);
    
                        % Circularly shift the buffer and fill the buffer with the new data
                        pDataInVitalsBuf = circshift(pDataInVitalsBuf,-1,2);
                        pDataInVitalsBuf(:,end) = pDataInVitals;
                        vitalsBufCounter = vitalsBufCounter + 1;
                        if vitalsBufCounter>VS_TOTAL_FRAME
                            vitalsBufAvailable = 1;
                        end
    
                    case 1010
                        % Target List TLV
                        numTargets = valueLength/targetLengthInBytes;                        
                        TID = zeros(1,numTargets);
                        S = zeros(9, numTargets);
                        EC = zeros(16, numTargets);
                        G = zeros(1,numTargets);  
                        Conf = zeros(1,numTargets);
                        for n=1:numTargets
                            TID(n)  = typecast(uint8(rxData(offset+1:offset+4)),'uint32');      %1x4=4bytes
                            offset = offset + 4;
                            S(:,n)  = typecast(uint8(rxData(offset+1:offset+36)),'single');     %9x4=36bytes
                            offset = offset + 36;
                            EC(:,n) = typecast(uint8(rxData(offset+1:offset+64)),'single');     %4x4x4=64bytes
                            offset = offset + 64;
                            G(n)    = typecast(uint8(rxData(offset+1:offset+4)),'single');      %1x4=4bytes
                            offset = offset + 4;
                            Conf(n) = typecast(uint8(rxData(offset+1:offset+4)),'single');      %1x4=4bytes
                            offset = offset + 4;
                        end
    
    
                    case 1011
                        % Target Index TLV
                        numIndices = valueLength/indexLengthInBytes;
                        mIndex = typecast(uint8(rxData(offset+1:offset+numIndices)),'uint8');
                        offset = offset + valueLength;
    
                    case 1012
                        % Should this require a loop for the number of targets
                        % in the scene?
                        numIndices = valueLength/indexLengthInBytes;
                        tid = typecast(uint8(rxData(offset+1:offset+4)),'uint8');
                        maxZ = typecast(uint8(rxData(offset+5:offset+8)),'single');
                        minZ = typecast(uint8(rxData(offset+9:offset+12)),'single');
                        offset = offset+valueLength;
    
                    case 1020
                        % Point Cloud TLV
                        % Get the unit scale for each point cloud dimension
                        pointUnit = typecast(uint8(rxData(offset+1: offset+pointUnitLengthInBytes)),'single');
                        elevUnit = pointUnit(1);
                        azimUnit = pointUnit(2);
                        dopplerUnit = pointUnit(3);
                        rangeUnit = pointUnit(4);
                        snrUnit = pointUnit(5);
    
                        offset = offset + pointUnitLengthInBytes;
                        numInputPoints = (valueLength - pointUnitLengthInBytes)/pointLengthInBytes;
                        if(numInputPoints > 0)    
    
                            % Get Point Cloud from the sensor
                            pointCloudTemp = typecast(uint8(rxData(offset+1: offset+valueLength- pointUnitLengthInBytes)),'uint8');
    
                            rangeInfo = (double(pointCloudTemp(6:pointLengthInBytes:end)) * 256 + double(pointCloudTemp(5:pointLengthInBytes:end)));
                            rangeInfo = rangeInfo * rangeUnit;
    
                            azimuthInfo =  double(pointCloudTemp(2:pointLengthInBytes:end));
                            indx = find(azimuthInfo >= 128);
                            azimuthInfo(indx) = azimuthInfo(indx) - 256;
                            azimuthInfo = azimuthInfo * azimUnit; % * pi/180;
    
                            elevationInfo =  double(pointCloudTemp(1:pointLengthInBytes:end));
                            indx = find(elevationInfo >= 128);
                            elevationInfo(indx) = elevationInfo(indx) - 256;
                            elevationInfo = elevationInfo * elevUnit; % * pi/180;
    
                            dopplerInfo = double(pointCloudTemp(4:pointLengthInBytes:end)) * 256 + double(pointCloudTemp(3:pointLengthInBytes:end));
                            %dopplerInfo =  double(pointCloudTemp(2:pointLengthInBytes:end));
                            indx = find(dopplerInfo >= 32768);
                            dopplerInfo(indx) = dopplerInfo(indx) - 65536;
                            dopplerInfo = dopplerInfo * dopplerUnit;
    
                            snrInfo = double(pointCloudTemp(8:pointLengthInBytes:end)) * 256 + double(pointCloudTemp(7:pointLengthInBytes:end));
                            snrInfo = snrInfo * snrUnit;
                            snrInfo(snrInfo<=1) = 1.1;
    
                            %idx = 1:length(dopplerInfo);  %include zero doppler
                            idx = find(dopplerInfo~=0);    %exclude zero doppler
    
                            range = rangeInfo(idx)';
                            azim = azimuthInfo(idx)';
                            elev = elevationInfo(idx)';
                            doppler = dopplerInfo(idx)';
                            snr = snrInfo(idx)';
                            pointCloudIn = [range; azim; elev; doppler; snr];
    
            %                 % Transformation from spherical to cartesian
            %                 point3D_T = [range.*cos(elev).*sin(azim); range.*cos(elev).*cos(azim);  range.*sin(elev)];
            %                 % Rotation along X axis
            %                 point3D_W = scene.RotX_TW*point3D_T;
            %                 % Move along Z axis
            %                 point3D_W(3,:) = point3D_W(3,:) + scene.sensorPos(3);
    
                            % One frame delay function for point cloud
                            % Because (n)th tracker output corresponds to (n-1)th point cloud
                            pointCloud = previousPointCloud;
                            point3D = previousPoint3D;
    
                            previousPointCloud = pointCloudIn;
                            previousPoint3D = point3D_W;
                        end
                        offset = offset + valueLength  - pointUnitLengthInBytes;
    
                    case 1021
                        % Presence Detection TLV
                        presence = typecast(uint8(rxData(offset+1:offset + presenceLengthInBytes)),'uint32');
                        offset = offset + valueLength;
                    case 320
                        % Presence Detection TLV
                        offset = offset + valueLength;
    
                    otherwise
                        "Uninterpretable TLV"
                        reason = 'TLV Type is wrong';
                        lostSync = 1;
                        break;                           
                end
            end
            if(lostSync)
                lostSyncStr = sprintf('lost sync %d', frameNum);
                disp(lostSyncStr);
                lostSyncFrames = [lostSyncFrames; frameNum];
                lostSyncFiles = [lostSyncFiles; file_num];
            end
    
            if(numInputPoints == 0)
                % No point cloud: read the previous
                pointCloud = previousPointCloud;                            
                point3D = previousPoint3D;
                % Set the prevous to zeros
                previousPointCloud = single(zeros(5,0));
                previousPoint3D = single(zeros(3,0));
            end
    
            numOutputPoints = size(pointCloud,2);
            if(numTargets == 0)
                TID = [];
                S = [];
                EC = [];
                G = [];
                Conf = [];
            end
    
            % Store Point cloud        
            fHist(row_idx).numInputPoints = numInputPoints;
            fHist(row_idx).numOutputPoints = numOutputPoints;    
            fHist(row_idx).numTargets = numTargets;
            fHist(row_idx).pointCloud = pointCloud;
            fHist(row_idx).targetList.numTargets = numTargets;
            fHist(row_idx).targetList.TID = TID;
            fHist(row_idx).targetList.S = S;
            fHist(row_idx).targetList.EC = EC;
            fHist(row_idx).targetList.G = G;
            fHist(row_idx).targetList.Conf = Conf;        
            fHist(row_idx).indexArray = mIndex;
            fHist(row_idx).presence = presence;
            fHist(row_idx).motionState = motionStatePerZoneTarget;
            fHist(row_idx).rangeAzimuthHeatMapMajor = rangeAzimuthHeatMapMajor;
            fHist(row_idx).rangeAzimuthHeatMapMinor = rangeAzimuthHeatMapMinor;
            fHist(row_idx).rngProfileMajor = rngProfileMajor;
            fHist(row_idx).rngProfileMinor = rngProfileMinor;
            fHist(row_idx).sensorTimingInfo = sensorTimingInfo;
            fHist(row_idx).sensorTemperatureInfo = sensorTemperatureInfo;
            fHist(row_idx).sensorPowerInfo = sensorPowerInfo;
            fHist(row_idx).microDopplerFeaturesBuf = microDopplerFeaturesBuf;
            fHist(row_idx).timestamp = timestamp;
            timestamp = [];
            if(isempty(predictionHist) ~= 1)
                fHist(row_idx).predictionHist = predictionHist;
            end
            row_idx = row_idx + 1;
            
    
        end
        fclose(fid);
    end
    
    
    
    tempfhistSaveName = strcat(output_dir , filesep, outfile_name, '.mat')
    save(tempfhistSaveName,'fHist'); 
    clear fHist;
end

lostSyncFiles = unique(lostSyncFiles);

%% Function Declarations

function length = lengthFromStruct(S)
    fieldName = fieldnames(S);
    length = 0;
    for n = 1:numel(fieldName)
        [~, fieldLength] = S.(fieldName{n});
        length = length + fieldLength;
    end
end

function CS = validateChecksum(header)
    h = typecast(uint8(header),'uint16');
    a = uint32(sum(h));
    b = uint16(sum(typecast(a,'uint16')));
    CS = uint16(bitcmp(b));
end

function [R] = readToStruct(S, ByteArray)
    fieldName = fieldnames(S);
    offset = 0;
    for n = 1:numel(fieldName)
        [fieldType, fieldLength] = S.(fieldName{n});
        R.(fieldName{n}) = typecast(uint8(ByteArray(offset+1:offset+fieldLength)), fieldType);
        offset = offset + fieldLength;
    end
end