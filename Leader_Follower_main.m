% Leader-follower robot 
% Main

clear all; close all; clc

% Robot properties
num_bots = 3;               % Change this for number of bots

%% Warehouse simulation
steps = 500;

% Loading map and attributes
% load exampleMaps.mat
% map = binaryOccupancyMap(simpleMap,1);
map = imread('Warehouse.png');
map = imresize(map,0.2);
map = rgb2gray(map);
map = map < 0.5;
map = binaryOccupancyMap(map);

robotradius = 0.3;
startLocation = [6 52];
endLocation = [93 6];

path = path_plan(map,robotradius,startLocation,endLocation);
path = path';

%smoothing points from path planning function
X_temp = path(1,1:end);
Y_temp = path(2,1:end);
T = 1:numel(X_temp);
xy = [X_temp;Y_temp];
pp = spline(T,xy);
tInterp = linspace(1,numel(X_temp),steps);
xyInterp = ppval(pp, tInterp);

x = zeros(num_bots,steps);
y = zeros(num_bots,steps);

x(1,:) = xyInterp(1,:);
y(1,:) = xyInterp(2,:);

%% General Pattern Simulation
% final_pos = 30;
% t = 1:pi/50:2*pi;
% x = zeros(num_bots,length(t));
% y = zeros(num_bots,length(t));
% 
% % ------ Arbitrary shapes ------
% % Line
% % x(1,:) = (final_pos/2).*ones(1,length(t));
% % x(1,:) = linspace(1,final_pos,length(t));
% % y(1,:) = linspace(1,final_pos,length(t));
% % y(1,:) = (final_pos/2).*ones(1,length(t));
% 
% % Circle
% r = 5;
% x(1,:) = r*sin(t);
% y(1,:) = r*cos(t);
% 
% % Square
% % x(1,:) = final_pos*square(t);
% % y(1,:) = final_pos*square(t);
% % ------ ---------------- ------

%% Follower Properties
% x(2,1) = 5*rand(1)+startLocation(1);         % Starting position of bot 2
% y(2,1) = 5*rand(1)+startLocation(2);
% x(3,1) = 5*rand(1)+startLocation(1);         % Starting position of bot 3
% y(3,1) = 5*rand(1)+startLocation(2);
% x(4,1) = 4*rand(1)+(final_pos/2);
% y(4,1) = 4*rand(1);
% x(5,1) = 4*rand(1)+(final_pos/2);
% y(5,1) = 4*rand(1);

x(2,1) = 7.5;         
y(2,1) = 56.2;
x(3,1) = 10.6;         
y(3,1) = 53.8;

% Follower Matrix - which robot follows which, R0 is always the leader
%[  R0 R1 R2 - leader
% R0 0  0  0
% R1 1  0  0
% R2 1  0  0 ]  This is an example of what this matrix should look like
%               This matrix gives the information of which robot is 
%               treating which robot as the leader

R = [0 0 0;                 
     1 0 0;
     1 0 0];                     
[R_pos,~] = find(R.');  

% [R1 R2......Rn] - follower robots
alpha_d = [deg2rad(0), deg2rad(0)]; % Angle between follower and leader
rho_d = [2, 4];                     % Distance between follower and leader
k = 0.7;                            % Gain
%% Leader-Follower Algorithm
for j = 2:num_bots
   
%    x_temp = zeros(1,length(x));
%    y_temp = zeros(1,length(y));
   R_loc = R_pos(j-1); 
   phi = 0;
   p_LA = 0;
   FA = 0;
   dT = x(1,2)-x(1,1);
   %
   rho = sqrt((x(R_loc,1)-x(j,1))^2 + (y(R_loc,1)-y(j,1))^2);
   alpha = atan2((y(R_loc,1)-y(j,1)),(x(R_loc,1)-x(j,1))) - FA;
   fullIntegral = [rho; alpha; phi];
   
   [err_rho_temp,err_alpha_temp,x_temp,y_temp] = calc_LeaderFollower(x,y,R_loc,j,phi,p_LA,FA,dT,k,rho,alpha,rho_d,alpha_d,fullIntegral);
   err_rho(j-1,:) = err_rho_temp;
   err_alpha(j-1,:) = err_alpha_temp;
   x(j,:) = x_temp;
   y(j,:) = y_temp;
        
end

%% Plotting Results
figure(1)
hold on
% xlim([0 final_pos+2])
% ylim([0 final_pos+2])
title('Leader-Follower Pattern Formation')
xlabel('x (m)')
ylabel('y (m)')
grid on
show(map)
plot(x(1,5:end),y(1,5:end))
plot(x(2,5:end),y(2,5:end))
plot(x(3,5:end),y(3,5:end))
% plot(x(4,10:end),y(4,10:end))
% plot(x(5,10:end),y(5,10:end))
a=plot(x(1,5),y(1,5),'ko','MarkerSize',10);
b=plot(x(2,5),y(2,5),'ro','MarkerSize',10);
c=plot(x(3,5),y(3,5),'ro','MarkerSize',10);
% d=plot(x(4,10),y(4,10),'ro');
% e=plot(x(5,10),y(5,10),'ro');
pause(5)
for k=6:length(x)
    a.XData = x(1,k);
    b.XData = x(2,k);
    c.XData = x(3,k);
%     d.XData = x(4,k);
%     e.XData = x(5,k);
    a.YData = y(1,k);
    b.YData = y(2,k);
    c.YData = y(3,k);
%     d.YData = y(4,k);
%     e.YData = y(5,k);
    drawnow;
    pause(0.05)
end
hold off

% Printing the error rates
%
% figure(2)
% hold on
% plot([1:length(err_rho)],err_rho)
% grid on
% xlim([-1 length(err_rho)+5])
% xlabel('time step')
% ylabel('error')
% title('Error rate - Distance')
% legend('R1','R2')
% hold off
% 
% figure(3)
% hold on
% plot([1:length(err_rho)],err_alpha)
% grid on
% xlim([-1 length(err_alpha)+5])
% xlabel('time step')
% ylabel('error')
% title('Error rate - Angle')
% legend('R1','R2')
% hold off




