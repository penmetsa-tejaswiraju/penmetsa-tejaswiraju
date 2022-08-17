function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow
    % optPos = Position of the features in the camera frame 
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter
    
    p = 0.99;
    M = 4;

    k = log(1 - p)/log(1-(e^M));

    Max = 0;

    for i = 1:k
        inliners = 0;

        randomP = randperm(length(optPos),3);

        P1 = optPos(randomP(1,1),:);
        P2 = optPos(randomP(1,2),:);
        P3 = optPos(randomP(1,3),:);

        H1 = [-1/Z, 0, P1(1,1)/Z, P1(1,1) * P1(1,2), -(1 + P1(1,1)^2), P1(1,2); 0, -1/Z, P1(1,2)/Z, (1 + P1(1,2)^2), -P1(1,1)*P1(1,2), -P1(1,1)];
        H2 = [-1/Z, 0, P2(1,1)/Z, P2(1,1) * P2(1,2), -(1 + P2(1,1)^2), P2(1,2); 0, -1/Z, P2(1,2)/Z, (1 + P2(1,2)^2), -P2(1,1)*P2(1,2), -P2(1,1)];
        H3 = [-1/Z, 0, P3(1,1)/Z, P3(1,1) * P3(1,2), -(1 + P3(1,1)^2), P3(1,2); 0, -1/Z, P3(1,2)/Z, (1 + P3(1,2)^2), -P3(1,1)*P3(1,2), -P3(1,1)];

        H = [H1; H2; H3];

        Vopt = [optV(2*randomP(1,1) - 1); optV(2*randomP(1,1)); optV(2*randomP(1,2) - 1); optV(2*randomP(1,2)); optV(2*randomP(1,3) - 1); optV(2*randomP(1,3))];

        V = pinv(H)*Vopt;

        for a = 1: length(optPos)

            Hi = [-1/Z 0 optPos(a,1)/Z  optPos(a,1)*optPos(a,2) -(1+optPos(a,1)^2) optPos(a,2);
              0 -1/Z optPos(a,2)/Z (1+optPos(a,2)^2) -optPos(a,1)*optPos(a,2) -optPos(a,1)];
            Pi = [optV(2*a - 1); optV(2*a)];
            delta = norm(Hi*V -Pi)^2;
            if(delta<= 1e-3)
                inliners = inliners +1;
            end

        end

        if(inliners >= Max)
             Max = inliners;
             Vel = V;
        end

    end



    
    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    
end