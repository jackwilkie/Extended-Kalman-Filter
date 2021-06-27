%% Robotics Assignment Sem 2
%Jack Wilkie
%13/03/21
%Robotics Kalman Filter Assignment

close all;
clear all;
clc
cd 'C:\Users\jackw\Desktop\Robotics_sem2'

%% Specifiy Robot Parameters

fs= 10;   %frequecny of enocder measurements
ts = 1/fs;  %enocder period

r = 10;  %robot wheel radius
d = 2*r;  %robot wheel diameter
c = pi * d;
base = 120;  %robot wheel base 
l = base/2;  %length from centre of gravity to wheel
vmax = 20;  %maximum robot velocity
varianceR = 0.1;  %variance in encoder measurement

%robot starting position
x0 = -250;  %robot starting position
y0 = -250;
z0 = 0;

theta0 = 0;  %robot starting rotation


%beacon locations
B1 = [2500;2500;1000];
B2 = [-2500;2500;1000];
B3 = [2500;-2500;1000];

varianceB = 20; %variance of beacon distance

%initalise robot parameteres
x = x0;
y = y0;
z = z0;

%left and right wheel speed
vl = vmax;
vr = vmax;

theta = theta0;
t=0;


%% Define Kinematic Eqns as Functions
v_roll =@(vl,vr) (vr+vl)/2 ;  %roll velocity

v_theta =@(vl,vr) (vr-vl)/(2*l);  %angular velocity

%% Q1 Plot Rails
%track robot position
count = 1;
x_loc(count,:) = x;
y_loc(count,:) = y;
theta_value(count,:) = 0; 
wheel_array(count,:) = [vl,vr]; 

%rastor scan size
rastor_l = 2000;  %rastor length
rastor_w = 150;  %rastor spacing
rastor_p = 7;  %rastor passes

for sweep = 1:rastor_p
    
    %reset starting positon after sweep
    x0 = x;
    y0=y;
    theta0 = theta;
    
    if sweep < rastor_p && mod(sweep,2) == 1 %odd pass travel x turn clock wise travel y then turn again
        %travel in x direction
        while abs(x-x0) < rastor_l
             count = count + 1;  %increment counter
             x = x + (v_roll(vl,vr)*ts); %find new x position
             x_loc(count,:) = x;  %store new x location
             y_loc(count,:) =y; %store new y location
             theta_value(count,:) = theta;  %store new angle
             wheel_array(count,:) = [vl, vr];  %store wheel speeds
        end
        
        %turn to face y axis
        while theta - theta0 < pi/2
            count = count + 1;  %increment counter
            theta = theta + (v_theta(-vl,vr)*ts);  %turn anticlockwise (with respect to robot)
            x_loc(count,:) = x;  %store new x location
            y_loc(count,:) =y; %store new y location
            theta_value(count,:) = theta;  %store new angle
            wheel_array(count,:) = [-vl, vr];  %store wheel speeds
       
            
        end
        
        %travel in y axis
        while abs(y-y0) < rastor_w
             count = count + 1;  %increment counter
             y = y + (v_roll(vl,vr)*ts); %find new x position
             x_loc(count,:) = x;  %store new x location
             y_loc(count,:) =y; %store new y location
             theta_value(count,:) = theta;  %store new angle
             wheel_array(count,:) = [vl, vr];  %store wheel speeds
        end
        
        %turn back to x axis
        while theta - theta0 < pi
            count = count + 1;  %increment counter
            theta = theta + (v_theta(-vl,vr)*ts);  %turn anticlockwise (with respect to robot)
            x_loc(count,:) = x;  %store new x location
            y_loc(count,:) =y; %store new y location
            theta_value(count,:) = theta;  %store new angle
            wheel_array(count,:) = [-vl, vr];  %store wheel speeds
            
        end 
    
    elseif sweep < rastor_p && mod(sweep,2) == 0 %odd pass travel x turn clock wise travel y then turn again 
               
        %travel in x direction
        while abs(x-x0) < rastor_l
             count = count + 1;  %increment counter
             x = x - (v_roll(vl,vr)*ts); %find new x position
             x_loc(count,:) = x;  %store new x location
             y_loc(count,:) =y; %store new y location
             theta_value(count,:) = theta;  %store new angle
             wheel_array(count,:) = [vl, vr];  %store wheel speeds
        end
        
        %turn to face y axis
        while theta - theta0 > -pi/2
            count = count + 1;  %increment counter
            theta = theta + (v_theta(vl,-vr)*ts);  %turn anticlockwise (with respect to robot)
            x_loc(count,:) = x;  %store new x location
            y_loc(count,:) =y; %store new y location
            theta_value(count,:) = theta;  %store new angle
            wheel_array(count,:) = [vl, -vr];  %store wheel speeds
       
            
        end
        
        %travel in y axis
        while abs(y-y0) < rastor_w
             count = count + 1;  %increment counter
             y = y + (v_roll(vl,vr)*ts); %find new x position
             x_loc(count,:) = x;  %store new x location
             y_loc(count,:) =y; %store new y location
             theta_value(count,:) = theta;  %store new angle
             wheel_array(count,:) = [vl, vr];  %store wheel speeds
        end
        
        %turn back to x axis
        while theta - theta0 > -pi
            count = count + 1;  %increment counter
            theta = theta + (v_theta(vl,-vr)*ts);  %turn anticlockwise (with respect to robot)
            x_loc(count,:) = x;  %store new x location
            y_loc(count,:) =y; %store new y location
            theta_value(count,:) = theta;  %store new angle
            wheel_array(count,:) = [vl, -vr];  %store wheel speeds
            
        end 
        
        
    else %final pass- travel length but dont turn
        
        while abs(x-x0) < rastor_l  %travel length of rastor
            count = count +1;
            x = x + (v_roll(vl,vr)*ts); %find new x position
            x_loc(count,:) = x;  %store new x location
            y_loc(count,:) =y; %store new y location
            theta_value(count,:) = theta;  %store new angle
            wheel_array(count,:) = [vl, vr];  %store wheel speeds
        end 
        
    end  
    
    
end


% plot robots rails
rail = [x_loc y_loc zeros(length(x_loc),1) theta_value];

figure;
plot(x_loc, y_loc);
title('Robots on Rails Path');
xlabel('X Position (mm)');
ylabel('Y Position (mm)');


%% Q2 Add Noise
%kinematic eqns estimation with noise
x_noise_speed =@(vl,vr,theta) ((vr+vl)/2)*cos(theta) ;  %roll velocity with x noise
y_noise_speed =@(vl,vr,theta) ((vr+vl)/2)*sin(theta) ; %roll velocity with y noise


%generate noise
noise_left_wheel = sqrt(varianceR).*randn(length(wheel_array),1);
noise_right_wheel = sqrt(varianceR).*randn(length(wheel_array),1);

%add noise to wheel speed measurements
wheels_noisy(:,1) =  wheel_array(:,1) + noise_left_wheel; %left wheel noise
wheels_noisy(:,2) =  wheel_array(:,2) + noise_right_wheel; %right wheel noise

%generate noisey dead reckoning path measurement
%initalise path
x = -250;
y = -250;
theta = 0;

%add noise to measurements
for i = 1:length(wheel_array)
    dead_reckoning(i, 1:4) = [x, y, 0 ,theta]; %store robot parameters
    
    x =  dead_reckoning(i,1)+(x_noise_speed(wheels_noisy(i,1), wheels_noisy(i,2), dead_reckoning(i,4))*ts);
    y =  dead_reckoning(i,2)+(y_noise_speed(wheels_noisy(i,1), wheels_noisy(i,2), dead_reckoning(i,4))*ts);
    theta = dead_reckoning(i,4)+ (v_theta(wheels_noisy(i,1),wheels_noisy(i,2))*ts);  %calculate robot angle
    
end 

%plot dead reckoning vs ideal path
figure;
hold on
plot(x_loc, y_loc);
plot(dead_reckoning(:,1), dead_reckoning(:,2))
hold off
title('Robots Actual Path vs Dead Reckoning');
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
legend('Actual Robot Path','Dead Reckoning');

%% Q3 Implement Beacon Estimates
rad_dist =@(x,y,z,beacon) sqrt((x-beacon(1)).^2 + (y-beacon(2)).^2 + (z-beacon(3)).^2); %find radial distance of robot from beacon with variance


r_b1 = rad_dist(rail(:,1),rail(:,2),rail(:,3),B1);
r_b2 = rad_dist(rail(:,1),rail(:,2),rail(:,3),B2);
r_b3 = rad_dist(rail(:,1),rail(:,2),rail(:,3),B3);

%generate beacon noise
r_b1_noise = sqrt(varianceB).*randn(length(r_b1),1);
r_b2_noise = sqrt(varianceB).*randn(length(r_b2),1);
r_b3_noise = sqrt(varianceB).*randn(length(r_b3),1);

%add noise to beacon position estimate
r_b1_noisy = r_b1 + r_b1_noise;
r_b2_noisy = r_b2 + r_b2_noise;
r_b3_noisy = r_b3 + r_b3_noise;

%convert to 2d radius measurement
r_b1_2d = sqrt((r_b1_noisy.^2)-(1000^2));
r_b2_2d = sqrt((r_b2_noisy.^2)-(1000^2));
r_b3_2d = sqrt((r_b3_noisy.^2)-(1000^2));

%coefficencts of linear eqns
A = (-2*B1(1))+(2*B2(1));
B = (-2*B1(2))+(2*B2(2));
C = (r_b1_2d.^2) - (r_b2_2d.^2) -(B1(1)^2) + (B2(2)^2) - (B1(2)^2) + (B2(2)^2);
D = (-2*B2(1))+(2*B3(1));
E =(-2*B2(2))+(2*B3(2));
F = (r_b2_2d.^2) - (r_b3_2d.^2) - (B2(1)^2) + (B3(1)^2) - (B2(2)^2) + (B3(2)^2);
    

%find linear intersections (location estimate)
x_est = ((C.*E)-(F.*B))./((A.*E)-(B.*D));
y_est = ((C.*D)-(A.*F))./((B.*D)-(A.*E));


%Plot beacon estimates
figure;
hold on
plot(x_loc, y_loc,'LineWidth',1.2);
plot(dead_reckoning(:,1), dead_reckoning(:,2),'LineWidth',1.2);
%scatter(x_est,y_est,1,[154/255 205/255 50/255],'filled');
scatter(x_est,y_est,3.5,[169/255 169/255 169/255],'filled');
hold off
title('Robot Positioning Comparison');
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
legend('Actual Robot Path','Dead Reckoning', 'Beacon Position Estimate');

%% Q4 Implement Kalman Filter

%measurement noise
R = [varianceB,0,0;0,varianceB,0;0,0,varianceB];

V = [1,0,0;0,1,0;0,0,1];  %3x3 identity matrix

%process noise
Q = [varianceR,0;0,varianceR];

b_est = [];  %beacon position estimates

p0 = zeros(4,4);  %starting postition
P = p0;  %postition

xkal = zeros(length(x_est),4); %kalman filtered 

prev_state = rail(1,:);

for i = 1:length(x_est)
    
    %update state using wheel speeds
    current_state = [prev_state(1) + (x_noise_speed(wheels_noisy(i,1),wheels_noisy(i,2),prev_state(4))*ts)
                    prev_state(2) + (y_noise_speed(wheels_noisy(i,1),wheels_noisy(i,2),prev_state(4))*ts)
                    prev_state(3)
                    prev_state(4) +  (v_theta(wheels_noisy(i,1),wheels_noisy(i,2))*ts)  ];
    
    
    %calculate jacobians
    
    %parial derivatives of state parameters
    A = [1,0,0, (-1/2)*(wheels_noisy(i,1) + wheels_noisy(i,2))*sin(prev_state(4)); 0,1,0, (1/2)*(wheels_noisy(i,1) + wheels_noisy(i,2))*cos(prev_state(4)); 0,0,1,0; 0,0,0,1];
    
    %partial derivatives of left and right wheel noise
    W = [cos(prev_state(4))*(ts/2)   sin(prev_state(4))*(ts/2)
         (cos(prev_state(4))*ts)/2   (sin(prev_state(4))*ts)/2
         0   0
         ts/l   ts/l];
    
    %measurement jacobian
    L1 = sqrt(((current_state(1)-B1(1))^2) + ((current_state(2)-B1(2))^2)  + ((current_state(3)-B1(3))^2));
    L2 = sqrt(((current_state(1)-B2(1))^2) + ((current_state(2)-B2(2))^2)  + ((current_state(3)-B2(3))^2));
    L3 = sqrt(((current_state(1)-B3(1))^2) + ((current_state(2)-B3(2))^2)  + ((current_state(3)-B3(3))^2));
    
    H = [(current_state(1)-B1(1))/L1   (current_state(2)-B1(2))/L1   (current_state(3)-B1(3))/L1   0
         (current_state(1)-B2(1))/L2   (current_state(2)-B2(2))/L2   (current_state(3)-B2(3))/L2   0
         (current_state(1)-B3(1))/L3   (current_state(2)-B3(2))/L1   (current_state(3)-B3(3))/L3   0];
    
     
    %beacon measurements 
    z = [r_b1_noisy(i);r_b2_noisy(i);r_b3_noisy(i)];
    
    
    %beacon distance estimates using filtered state estimate
    h = [rad_dist(current_state(1),current_state(2),current_state(3),B1)
         rad_dist(current_state(1),current_state(2),current_state(3),B2)
         rad_dist(current_state(1),current_state(2),current_state(3),B3)];
        
     
    %kalman filter 
    P = A*P*A' + W*Q*W';
    
    K = P*H'*inv(H*P*H' + V*R*V');
    
    %state correction
    state = current_state + K*(z-h);
    
    xkal(i,1:4) = state(1:4)';
    
    P = (eye(4)-K*H)*P;
    
    prev_state = state;
end

figure;
hold on
plot(x_loc, y_loc,'LineWidth',1.4);
plot(dead_reckoning(:,1), dead_reckoning(:,2),'LineWidth',1.2);
%scatter(x_est,y_est,1,[154/255 205/255 50/255],'filled');
scatter(x_est,y_est,3.5,[169/255 169/255 169/255],'filled');
plot(xkal(:,1),xkal(:,2),'LineWidth',1.2);
hold off
title('Robot Positioning Comparison');
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
legend('Actual Robot Path','Dead Reckoning', 'Beacon Position Estimate', 'EKF Path Estimate');
    
