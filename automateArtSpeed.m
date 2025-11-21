
clc
clear
close all

%% Define parameters
basePath = "...\SimData\";
lengths = [11, 13, 15];
collisionTypes = ["Lateral", "Rear"];
versions = ["c0","c1","c2","c3","c4"];

% Crash iter values 
crash_iters = struct( ...
    'Lateral_11', [289,462,611,722,814], ...
    'Rear_11',    [321,479,642,735,834], ...
    'Lateral_13', [209,364,471,544,625], ...
    'Rear_13',    [266,455,493,568,640], ...
    'Lateral_15', [219,403,531,622,704], ...
    'Rear_15',    [293,430,555,636,716] ...
);

windowSize = 10;

%% Store results
results = table('Size',[0 8], ...
    'VariableTypes',{'double','string','string','string','double','double','double','double'}, ...
    'VariableNames',{'Length','CollisionType','Version','FileName','one','oneThree','two','Time'});

%% Loop through all combinations
for L = lengths
    for cType = collisionTypes
        for v = 1:length(versions)
            version = versions(v);
            fname = sprintf("vehicle_data%s_%d_%s.csv", cType, L, version);
            fullpath = fullfile(basePath, fname);

            if ~isfile(fullpath)
                fprintf('Missing file: %s\n', fullpath);
                continue;
            end

            data = readtable(fullpath);

            key = sprintf('%s_%d', cType, L);
            crash_iter = crash_iters.(key)(v);

            % Set trailer length index
            if L == 11
                tl_length = 1;
            elseif L == 13
                tl_length = 2;
            else
                tl_length = 3;
            end

            %% 

            b = (1/windowSize) * ones(1, windowSize);
            a = 1;

            x1 = table2array(data(:,1));
            y1 = table2array(data(:,2));
            vL1 = table2array(data(:,7));
            vLx1 = table2array(data(:,8));
            vLy1 = table2array(data(:,9));
            psi1 = table2array(data(:,22));
            psi1dot= [diff(psi1)/0.05; 0];

            x2 = table2array(data(:,3));
            y2 = table2array(data(:,4));
            vL2 = table2array(data(:,12));
            vLx2 = table2array(data(:,13));
            vLy2 = table2array(data(:,14));
            psi2 = table2array(data(:,25));
            psi2dot= [diff(psi2)/0.05; 0];

            x3 = table2array(data(:,5));
            y3 = table2array(data(:,6));
            vL3 = table2array(data(:,17));
            vLx3 = table2array(data(:,18));
            vLy3 = table2array(data(:,19));
            psi3 = table2array(data(:,28));
            psi3dot= [diff(psi3)/0.05; 0];

            %% Coordinates setup
            if (tl_length==2)
                corners_unit1 = [-3.408212,-1.475510;2.455874,-1.475510;-3.408212,1.443235;2.455874,1.443235];
                corners_unit2 = [-12.053223,-1.254007;1.636379,-1.254007;-12.053223,1.247322;1.636379,1.247322];
                w2=1.254007;w3=1.281716;l2=13.68;w=w2+w3;l1=5.85;l_fa=4.4;l_ra=1.65;
            elseif(tl_length==3)
                corners_unit1 = [-3.833065,-1.655833;2.880729,-1.655833;-3.833065,1.623618;2.880729,1.623618];
                corners_unit2 = [-12.611754,-1.254012;2.194917,-1.254012;-12.611754,1.247316;2.194917,1.247316];
                w2=1.254012;w3=1.081716;l2=14.80;w=w2+w3;l1=6.71;l_fa=4.4;l_ra=1.7;
            else
                corners_unit1 = [-3.833065,-1.655833;2.880729,-1.655833;-3.833065,1.623618;2.880729,1.623618];
                corners_unit2 = [-9.589092,-1.254018;0.827748,-1.254012;-9.589092,1.247311;0.827748,1.247311];
                w2=1.254018;w3=1.081716;l2=10.41;w=w2+w3;l1=6.71;l_fa=4.4;l_ra=1.3;
            end

            corners_unit3 = [-2.366670,-1.081716;2.425109,-1.081716;-2.366670,1.081734;2.425109,1.081734];
            num_points = length(x1);

            % Initialize arrays
            s_long32=[]; s_lat32=[]; v_rel_x31=[]; v_rel_y31=[]; a_rel_x31=[]; a_rel_y31=[];
            storeX=[]; storeY=[];
            c1x=[]; c1y=[]; c2x=[]; c2y=[]; c3x=[]; c3y=[];

            % Loop through each data point
            for i = 1:num_points
                heading_angle1 = psi1(i);
                heading_angle2 = psi2(i);
                heading_angle3 = psi3(i);
                R1 = [cos(heading_angle1), -sin(heading_angle1); sin(heading_angle1), cos(heading_angle1)];
                R2 = [cos(heading_angle2), -sin(heading_angle2); sin(heading_angle2), cos(heading_angle2)];
                R3 = [cos(heading_angle3), -sin(heading_angle3); sin(heading_angle3), cos(heading_angle3)];
                rotated_corners1 = (R1 * corners_unit1')';
                translated_corners1 = rotated_corners1 + [x1(i), y1(i)];
                rotated_corners2 = (R2 * corners_unit2')';
                translated_corners2 = rotated_corners2 + [x2(i), y2(i)];
                rotated_corners3 = (R3 * corners_unit3')';
                translated_corners3 = rotated_corners3 + [x3(i), y3(i)];
                c1 = (translated_corners1(2,:)+translated_corners1(4,:))/2;
                c2 = (translated_corners2(2,:)+translated_corners2(4,:))/2;
                c3 = (translated_corners3(2,:)+translated_corners3(4,:))/2;
                c1x(i)=c1(1); c1y(i)=c1(2);
                c2x(i)=c2(1); c2y(i)=c2(2);
                c3x(i)=c3(1); c3y(i)=c3(2);
            end

            vcGx1 = diff(c1x)/0.05;
            vcGy1 = diff(c1y)/0.05;
            vcGx2 = diff(c2x)/0.05;
            vcGy2 = diff(c2y)/0.05;
            vcGx3 = diff(c3x)/0.05;
            vcGy3 = diff(c3y)/0.05;
                      
            vcLx1 = vcGx1.*cos(psi1(1:end-1).')+vcGy1.*sin(psi1(1:end-1).');
            vcLy1 = -vcGx1.*sin(psi1(1:end-1).')+vcGy1.*cos(psi1(1:end-1).');
            
            vcLx2 = vcGx2.*cos(psi2(1:end-1).')+vcGy2.*sin(psi2(1:end-1).');
            vcLy2 = -vcGx2.*sin(psi2(1:end-1).')+vcGy2.*cos(psi2(1:end-1).');
            
            vcLx3 = vcGx3.*cos(psi3(1:end-1).')+vcGy3.*sin(psi3(1:end-1).');
            vcLy3 = -vcGx3.*sin(psi3(1:end-1).')+vcGy3.*cos(psi3(1:end-1).');           

            tic 
            
            for i=1:num_points-1
            
         
            % Calculate transformed speeds
            [vc1Lx3, vc1Ly3] = transformSpeeds(psi1(i), vcLx3(i), vcLy3(i), psi3(i));
            
            % Calculate relative velocity
            v_rel_x31(i) = vc1Lx3 - vcLx1(i);
            v_rel_y31(i) = vc1Ly3 - vcLy1(i);
            
            % Position difference vector between vehicle 3 and vehicle 2 and
            % vehicle 1
            pos_diff31 = [c3x(i) - c1x(i); c3y(i) - c1y(i)];
            
             % Transform position difference to vehicle 2 and vehicle 1 frame
            pos_diff_v1_frame31 = R1' * pos_diff31;
            
            % Longitudinal and lateral distances in vehicle 1 frame
            s_long31(i) = pos_diff_v1_frame31(1);
            s_lat31(i) = pos_diff_v1_frame31(2);
        
        
            % 2D TTC computation 3 1
            TTCx=  (abs(s_long31(i))-l1)/v_rel_x31(i);
            s_lat_f= abs(s_lat31(i))-v_rel_y31(i)*TTCx;
        
            TTCy=  (abs(s_lat31(i))-w)/v_rel_y31(i);
            s_long_f= s_long31(i)-v_rel_x31(i)*TTCy;
        
            if(abs(s_lat_f) < w && v_rel_x31(i)>0 && abs(s_long31(i))>l1)
                TTC_long1(i)= TTCx;
            else
                TTC_long1(i)= 5;
            end    
            
            if(abs(s_long_f) <= l1 && v_rel_y31(i)>0 && abs(s_lat31(i))>w)
                TTC_lat1(i)= TTCy;
            else
                TTC_lat1(i)= 5;
            end  
            
            tau1(i)=min(TTC_long1(i),TTC_lat1(i));
            
            dt=0.005;
            time=0;
            
            X1=[];
            Y1=[];
            X2=[];
            Y2=[];
            X3=[];
            Y3=[];
        
            X1(1) = c1x(i);
            Y1(1) = c1y(i);
            X2(1) = c2x(i);
            Y2(1) = c2y(i);
            X3(1) = c3x(i);
            Y3(1) = c3y(i);
        
            psi_tc = psi1(i);
            psi_tr0 = psi2(i);
            psi_tr = psi_tr0;
            k=2;
            TTC_long2(i) = 5;
            TTC_lat2(i) = 5;
        
            while (time<=tau1(i))
        
                X1(k) = X1(k-1) + vcGx1(i) * dt;
                Y1(k) = Y1(k-1) + vcGy1(i) * dt;
                X3(k) = X3(k-1) + vcGx3(i) * dt;
                Y3(k) = Y3(k-1) + vcGy3(i) * dt;
        
                A = abs(tan((psi_tr0 - psi_tc)/2));
                B = vcLx1(i) * (-time)/l2;
        
                if (psi_tr - psi_tc < 0)
                  psi_tr = psi_tc - 2*atan(A*exp(B));
        
                else
                  psi_tr = psi_tc + 2*atan(A*exp(B));  
        
                end
        
                X2(k) = X1(k) - l_fa * cos(psi_tc) +  l_ra * cos(psi_tr);
                Y2(k) = Y1(k) - l_fa * sin(psi_tc) +  l_ra * sin(psi_tr);
        
                % Position difference vector between vehicle 3 and vehicle 2 and
                % vehicle 1
                pos_diff32 = [X3(k) - X2(k); Y3(k) - Y2(k)];
        
                R2 = [cos(psi_tr), -sin(psi_tr);
                      sin(psi_tr),  cos(psi_tr)];
            
                % Transform position difference to vehicle 3 and vehicle 2 frame
                pos_diff_v1_frame32 = R2' * pos_diff32;
            
                % Longitudinal and lateral distances in vehicle 2 frame
                s_long32 = pos_diff_v1_frame32(1);
                s_lat32= pos_diff_v1_frame32(2);
        
                %
                if(abs(s_lat32) < w && round(abs(s_long32))==round(l2))
                  TTC_long2(i)= time;
                  break
                end    
            
                if(abs(s_long32) <= l2 && abs(s_lat32)<w)
                  TTC_lat2(i)= time;
                  break
                end  
        
                time = time + dt;
                k=k+1;
        
            end
        
             tau2(i)=min([TTC_long2(i),TTC_lat2(i)]);
        
        
             TTC(i)=min(tau1(i),tau2(i));
            
            end

  time_needed = toc;
%%

       x = -0.05:0.05:(crash_iter-1)*0.05 - 0.05;
       y = TTC(1:crash_iter);

      % Compute the target x values
       x_target_1s = x(end) - 1;
       x_target_1_3s = x(end) - 1.38;
       x_target_2s = x(end) - 2;
       

      % Interpolate to find corresponding y values
       y_target_1s = interp1(x, y, x_target_1s, 'linear');
       y_target_1_3s = interp1(x, y, x_target_1_3s, 'linear');
       y_target_2s = interp1(x, y, x_target_2s, 'linear');


      % Store the results
       results = [results; {L, cType, version, fname, y_target_1s, y_target_1_3s, y_target_2s, time_needed}];
       fprintf("Processed: %s | y_target_1s = %.3f | y_target_13s = %.3f| y_target_2s = %.3f\n", fname, y_target_1s, y_target_1_3s, y_target_2s);

        end
    end
end

%% Save results

writetable(results, 'ytarget_results_speed.xlsx');
fprintf('\nAll results saved to ytarget_results.xlsx\n');

%% Boxplot of y_target for each trailer length and collision type

% Read results from Excel
data = readtable('ytarget_results_speed.xlsx');

% Ensure collision type is categorical
data.CollisionType = categorical(data.CollisionType);
data.Length = categorical(data.Length);

figure('Position',[100 100 1200 600]);

% Get unique lengths
lengths = categories(data.Length);

for i = 1:length(lengths)
    L = lengths{i};

    % Filter for current length
    subData = data(data.Length == L, :);

    subplot(1, numel(lengths), i);
    boxchart(subData.CollisionType, subData.one, ...
        'BoxFaceColor',[0.2 0.6 0.8], ...
        'MarkerStyle','o','MarkerColor','k');

    title(sprintf('Trailer Length = %s m', L), 'FontWeight', 'bold');
    xlabel('Collision Type');
    ylabel('y_{target}');
    grid on;
end

sgtitle('Distribution of y_{target} by Trailer Length and Collision Type', 'FontWeight', 'bold');


%%
function [long_speed_v2_in_v1, lat_speed_v2_in_v1] = transformSpeeds(heading_v1, long_speed_v2, lat_speed_v2, heading_v2)

    % Convert headings from degrees to radians for trigonometric functions
    heading_v1_rad = deg2rad(heading_v1);
    heading_v2_rad = deg2rad(heading_v2);
    
    % Rotation matrix for vehicle 1
    R_v1 = [cos(heading_v1_rad), -sin(heading_v1_rad);
            sin(heading_v1_rad), cos(heading_v1_rad)];
        
    % Rotation matrix for vehicle 2
    R_v2 = [cos(heading_v2_rad), -sin(heading_v2_rad);
            sin(heading_v2_rad), cos(heading_v2_rad)];
    
    % Speeds of vehicle 2 in its own frame
    speed_v2 = [long_speed_v2; lat_speed_v2];
    
    % Transform speed of vehicle 2 to global frame
    speed_v2_global = R_v2 * speed_v2;
    
    % Transform speed of vehicle 2 from global frame to vehicle 1 frame
    speed_v2_in_v1_frame = R_v1' * speed_v2_global; % Using the transpose of R_v1 to go from global to v1 frame
    
    % Extract longitudinal and lateral speeds of vehicle 2 in vehicle 1 frame
    long_speed_v2_in_v1 = speed_v2_in_v1_frame(1);
    lat_speed_v2_in_v1 = speed_v2_in_v1_frame(2);
end


function [long_accel_v2_in_v1, lat_accel_v2_in_v1] = transformAccelerations(heading_v1, long_accel_v2, lat_accel_v2, heading_v2)

    % Convert headings from degrees to radians for trigonometric functions
    heading_v1_rad = deg2rad(heading_v1);
    heading_v2_rad = deg2rad(heading_v2);
    
    % Rotation matrix for vehicle 1
    R_v1 = [cos(heading_v1_rad), -sin(heading_v1_rad);
            sin(heading_v1_rad), cos(heading_v1_rad)];
        
    % Rotation matrix for vehicle 2
    R_v2 = [cos(heading_v2_rad), -sin(heading_v2_rad);
            sin(heading_v2_rad), cos(heading_v2_rad)];
    
    % Accelerations of vehicle 2 in its own frame
    accel_v2 = [long_accel_v2; lat_accel_v2];
    
    % Transform acceleration of vehicle 2 to global frame
    accel_v2_global = R_v2 * accel_v2;
    
    % Transform acceleration of vehicle 2 from global frame to vehicle 1 frame
    accel_v2_in_v1_frame = R_v1' * accel_v2_global; % Using the transpose of R_v1 to go from global to v1 frame
    
    % Extract longitudinal and lateral accelerations of vehicle 2 in vehicle 1 frame
    long_accel_v2_in_v1 = accel_v2_in_v1_frame(1);
    lat_accel_v2_in_v1 = accel_v2_in_v1_frame(2);
end



