%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION

K = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1];

RansacFlag = 1; % Turn this to 0 to turn off Ransac

tic

for n = 2:length(sampledData)
    %% Initalize Loop load images
    n

    CurrImg = sampledData(n).img;
    Lastimg = sampledData(n-1).img;

    
    %% Detect good points

    c = vision.PointTracker('MaxBidirectionalError', 1); % Point Tracke

    corner = detectFASTFeatures(Lastimg); 
    P = double(corner.Location); % Increasing accuracy by using double

    
    %% Initalize the tracker to the last frame.

    initialize(c, P, Lastimg);
 
    %% Find the location of the next points;
    
    [k,val] = c(CurrImg);

    
    %% Calculate velocity
    % Use a for loop
    p_old = [];
    p_new = [];
    VEL = [];
    for i = 1 : length(k)

        dt = (sampledData(n).t - sampledData(n-1).t);
        
        old_pts = inv(K) * [P(i,1) ; P(i,2); 1];
        p_old(i,1) = old_pts(1,1);
        p_old(i,2) = old_pts(2,1);

        new_pts = inv(K) * [k(i,1); k(i,2); 1];
        p_new(i,1) = new_pts(1,1);
        p_new(i,2) = new_pts(2,1);

        u(i) = (p_new(i,1) - p_old(i,1))/dt;
        v(i) = (p_new(i,2) - p_old(i,2))/dt;

        VEL = [VEL; u(i); v(i)];

    end

    
 
    
    %% Calculate Height
    
    [position, orientation, Rcw] = estimatePose(sampledData, n);

    R = eul2rotm(orientation);

    Twb = [R, position; 0, 0, 0, 1];
    
    tcb = [-0.04; 0; -0.03];
    Rcb = [cosd(45), -sind(45), 0; -cosd(45), -sind(45), 0; 0, 0, -1]; % About X 180 and about Z 45

    Tcb = [Rcb, tcb; 0, 0, 0, 1];

    Tbc = inv(Tcb);

    Tcw = inv(Twb * Tcb);
    
    X = position(1);
    Y = position(2);
    Z = Tcw(3,4);

    f = [];

    for i = 1: length(k)

        
        fu = [-1/Z, 0, p_new(i,1)/Z, p_new(i,1) * p_new(i,2), -(1 + p_new(i,1)^2), p_new(i,2); 0, -1/Z, p_new(i,2)/Z, (1 + p_new(i,2)^2), -p_new(i,1) * p_new(i,2), -p_new(i,1)];
        f = [f;fu];

    end

    optP = [p_new(:,1), p_new(:,2)];

    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC
    e = 0.8;
    velocityRansac = velocityRANSAC(VEL, optP, Z, Rcw, e);
    size(velocityRansac)

    %% Thereshold outputs into a range.
    % Not necessary
    
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame

    V = pinv(f) * VEL;
    Twc = inv(Tcw);

    if RansacFlag == 1

        Vel = [R, zeros(3,3); zeros(3,3), R] * [Tbc(1:3, 1:3), -Tbc(1:3, 1:3)*skew(Tbc(1:3, 4)); zeros(3,3), Tbc(1:3, 1:3)] * velocityRansac;
    end
    if RansacFlag == 0

        Vel = [R, zeros(3,3); zeros(3,3), R] * [Tbc(1:3, 1:3), -Tbc(1:3, 1:3)*skew(Tbc(1:3, 4)); zeros(3,3), Tbc(1:3, 1:3)] * V;
    end


    estimatedV(:,n) = Vel;


    
    %% ADD SOME LOW PASS FILTER CODE
    % Written outside
    % Not neceessary but recommended
    
    estimatedV(:,n) = Vel;
    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
%     estimatedV(:,n) = Vel; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end
%% LOW PASS FILTER

    time = zeros(length(sampledData), 1);
        for i=1:length(time)
            time(i,1) = sampledData(1,i).t;
        end
    time = sgolayfilt(time, 1, 101);

    vx = transpose(estimatedV(1,:));
    vy = transpose(estimatedV(2,:));
    vz = transpose(estimatedV(3,:));

    wx = transpose(estimatedV(4,:));
    wy = transpose(estimatedV(5,:));
    wz = transpose(estimatedV(6,:));


    estimatedV(1,:) = sgolayfilt(vx, 1, 17);
    estimatedV(2,:) = sgolayfilt(vy, 1, 17);
    estimatedV(3,:) = sgolayfilt(vz, 1, 7);
    estimatedV(4,:) = sgolayfilt(wx, 1, 5);
    estimatedV(5,:) = sgolayfilt(wy, 1, 5);
    estimatedV(6,:) = sgolayfilt(wz, 1, 5);
    
toc
plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
