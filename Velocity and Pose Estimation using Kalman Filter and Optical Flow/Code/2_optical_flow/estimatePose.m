function [position, orientation, R_c2w] = estimatePose(data, t)

    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    ID = data(t).id; % ID of april tags

    res = getCorner(ID);

    K = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1]; % Camera Intrinsic Matrix
   

    % Calculation for P and T

    for i = 1:length(data(t).id)

        p0hat = [res(1,i), res(2,i), 1, 0, 0, 0, -data(t).p0(1,i) * res(1,i), -data(t).p0(1,i) * res(2,i), -data(t).p0(1,i);
                 0 , 0 , 0, res(1,i), res(2,i), 1, -data(t).p0(2,i) * res(1,i), -data(t).p0(2,i) * res(2,i), -data(t).p0(2,i)];
        
        p1hat = [res(3,i), res(4,i), 1, 0, 0, 0, -data(t).p1(1,i) * res(3,i), -data(t).p1(1,i) * res(4,i), -data(t).p1(1,i);
                 0 , 0 , 0, res(3,i), res(4,i), 1, -data(t).p1(2,i) * res(3,i), -data(t).p1(2,i) * res(4,i), -data(t).p1(2,i)];

        p2hat = [res(5,i), res(6,i), 1, 0, 0, 0, -data(t).p2(1,i) * res(5,i), -data(t).p2(1,i) * res(6,i), -data(t).p2(1,i);
                 0 , 0 , 0, res(5,i), res(6,i), 1, -data(t).p2(2,i) * res(5,i), -data(t).p2(2,i) * res(6,i), -data(t).p2(2,i)];

        p3hat = [res(7,i), res(8,i), 1, 0, 0, 0, -data(t).p3(1,i) * res(7,i), -data(t).p3(1,i) * res(8,i), -data(t).p3(1,i);
                 0 , 0 , 0, res(7,i), res(8,i), 1, -data(t).p3(2,i) * res(7,i), -data(t).p3(2,i) * res(8,i), -data(t).p3(2,i)];

        p4hat = [res(9,i), res(10,i), 1, 0, 0, 0, -data(t).p4(1,i) * res(9,i), -data(t).p4(1,i) * res(10,i), -data(t).p4(1,i);
                 0 , 0 , 0, res(9,i), res(10,i), 1, -data(t).p4(2,i) * res(9,i), -data(t).p4(2,i) * res(10,i), -data(t).p4(2,i)];

        if i == 1
            phat = [p0hat; p1hat; p2hat; p3hat; p4hat];
        elseif i > 1
            phat = vertcat(phat, [p0hat; p1hat; p2hat; p3hat; p4hat]);
        end
        
    end

                
        [u, s, v] = svd(phat);
        

        % Calculating H matrix
        
          b = v(:,9);
          r = [b(1), b(2), b(3); b(4), b(5), b(6); b(7), b(8), b(9)];

            
          % Sign of H

          sk = svd(r);
          h = r/sk(2);
          g = transpose([data(t).p0(1:2,1);1]) * r * [res(1:2,1);1];

          if g < 0

               h = -h;

          end


          cal = pinv(K) * h;

          % r = r1hat r2hat That

          r1hat = cal(:,1);
          r2hat = cal(:,2);
          that = cal(:,3);

        % Calculating R and T

          [ud, sd, vd] = svd([r1hat, r2hat, cross(r1hat, r2hat)]);

          Rcw = ud * [1, 0, 0; 0, 1, 0; 0, 0, det(ud * vd')] * vd';

          tcw = that/norm(r1hat);
          

        tbc = [0.04*cosd(45); -0.04*sind(45); -0.03];
        Rbc = [cosd(45), -sind(45), 0; -cosd(45), -sind(45), 0; 0, 0, -1]; % About X 180 and about Z 45

        Tbc = [Rbc, tbc; 0, 0, 0, 1];
        Tcw = [Rcw, tcw; 0, 0, 0, 1];

        Tbw = Tbc * Tcw;
        Twb = pinv(Tbw);

        Rwb = Twb(1:3, 1:3);
        twb = Twb(1:3, 4);

        position = twb;
        orientation = rotm2eul(Rwb);
        R_c2w = Rcw;


    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    %R_c2w = Rotation which defines camera to world frame
end
