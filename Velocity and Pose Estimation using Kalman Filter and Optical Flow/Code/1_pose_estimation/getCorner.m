function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)

    for t = 1:length(id)
        x = 0; % Resetting the value
        y = 0; % Resetting the value

        if id(t) <= 11  % Column 1
            y = 0;
            x = id(t) * (0.152 + 0.152);
        elseif (id(t) >= 12) && (id(t) <= 23) % Column 2
            y = 0.152 + 0.152;
            x = rem(id(t),12) * (0.152 + 0.152);
        elseif (id(t) >= 24) && (id(t) <= 35) % Column 3
            y = 2 * (0.152 + 0.152);
            x = rem(id(t),12) * (0.152 + 0.152);
        elseif (id(t) >= 36) && (id(t) <= 47) % Column 4
            y = 2 * (0.152 + 0.152) + (0.152 + 0.178);
            x = rem(id(t),12) * (0.152 + 0.152);
        elseif (id(t) >= 48) && (id(t) <= 59) % Column 5
            y = 3 * (0.152 + 0.152) + (0.152 + 0.178);
            x = rem(id(t),12) * (0.152 + 0.152);
        elseif (id(t) >= 60) && (id(t) <= 71) % Column 6
            y = 4 * (0.152 + 0.152) + (0.152 + 0.178);
            x = rem(id(t),12) * (0.152 + 0.152);
        elseif (id(t) >= 72) && (id(t) <= 83) % Column 7
            y = 4 * (0.152 + 0.152) + 2 * (0.152 + 0.178);
            x = rem(id(t),12) * (0.152 + 0.152);
        elseif (id(t) >= 84) && (id(t) <= 95) % Column 8
            y = 5 * (0.152 + 0.152) + 2 * (0.152 + 0.178);
            x = rem(id(t),12) * (0.152 + 0.152);
        elseif id(t) >= 96 % Column 9
            y = 6 * (0.152 + 0.152) + 2 * (0.152 + 0.178);
            x = rem(id(t),12) * (0.152 + 0.152);
        end
       
        P0x = x + (0.152/2);
        P0y = y + (0.152/2);
        P1x = x + 0.152;
        P1y = y;
        P2x = x + 0.152;
        P2y = y + 0.152;
        P3x = x;
        P3y = y + 0.152;
        P4x = x;
        P4y = y;

        new = [P0x; P0y; P1x; P1y; P2x; P2y; P3x; P3y; P4x; P4y];
        if t == 1
            res = new;
        elseif t > 1
            res = horzcat(res, new);
        end

    end
 
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method
    res;
    return
end