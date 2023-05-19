%% Full car model
% Simulation and animation of a half car model.
%
% Inspired by: https://www.mathworks.com/matlabcentral/fileexchange/75019-vertical-vehicle-behavior-models
%
%%
clear ; close all ; clc

%% Parameters
% Values from Jazar Example 508.

% Inertial
m   = 840;      % Sprung mass [kg]
mf  = 53;       % Unsprung mass front [kg]
mr  = 76;       % Unsprung mass rear [kg]
Ix  = 820;      % Moment of inertia x [kg.m2]
Iy  = 1100;     % Moment of inertia y [kg.m2]

% Distance
a1  = 1.4;      % Dist. CG front axle [m]
a2  = 1.47;     % Dist. CG rear axle [m]
b1  = 0.7;      % Dist. CG left wheel [m]
b2  = 0.75;     % Dist. CG right wheel [m]
w = b1+b2;      % Track [m]
% Damping
cf  = 5000;      % Damping front [N.s/m]
cr  = 5000;      % Damping rear [N.s/m]

% Stiffness
kf  = 10000;    % Stiffness front [N/m]
kr  = 13000;    % Stiffness rear [N/m]
ktf = 200000;   % Stiffness tire front [N/m]
ktr = 200000;   % Stiffness tire rear [N/m]
kR  = 10000;    % Stiffness stabilizing bar [Nm/rad]

% Animation model
z_s_static = 1.6;   % Sprung mass static vertical position      [m]
z_u_static = 0.7;   % Unsprung mass static vertical position    [m]
l_win        = 5;   % Length window analysis                    [m]
l_win_maring = 1;   % Length window margin at the begining      [m]

% Video
playback_speed = 0.1;               % Speed of playback
tF      = 1;                        % Final time                [s]
fR      = 30/playback_speed;        % Frame rate                [fps]
dt      = 1/fR;                     % Time resolution           [s]
time    = linspace(0,tF,tF*fR);     % Time                      [s]

%% Track
% LEFT SIDE
% Stretch 1
x1_total_left = 5;       % Distance of the first stretch             [m]
dx_left = 0.1;           % resolution                                [m]
x1_left = 0:dx_left:x1_total_left;
z1_t_left = zeros(1,length(x1_left));
% Stretch 2
R_left = 0.2;            % Radius                        [m]
th_t_left = 0:0.01:pi;
x2_left = -R_left*cos(th_t_left) + x1_total_left+R_left;
z2_t_left = R_left*sin(th_t_left);
% Stretch 3
x3_total_left = 15;      % Distance of the last stretch              [m]
dx_left = 0.1;           % resolution                                [m]
x3_left = x1_total_left+2*R_left:dx_left:x1_total_left+2*R_left+x3_total_left;
z3_t_left = zeros(1,length(x3_left));
% Concatenating - left
x_track_left = [x1_left x2_left(2:end) x3_left(2:end)];
z_track_left = [z1_t_left z2_t_left(2:end) z3_t_left(2:end)];

% RIGHT SIDE
% Stretch 1
% Longitudinal offset between left and right
offset_left_right = 1; % [m]
x1_total_right = 5 + offset_left_right ;       % Distance of the first stretch             [m]
dx_right = 0.1;           % resolution                                [m]
x1_right = 0:dx_right:x1_total_right;
z1_t_right = zeros(1,length(x1_right));
% Stretch 2
R_right = 0.2;            % Radius                        [m]
th_t_right = 0:0.01:pi;
x2_right = -R_right*cos(th_t_right) + x1_total_right+R_right;
z2_t_right = R_right*sin(th_t_right);
% Stretch 3
x3_total_right = 15-offset_left_right;      % Distance of the last stretch              [m]
dx_right = 0.1;           % resolution                                [m]
x3_right = x1_total_right+2*R_right:dx_right:x1_total_right+2*R_right+x3_total_right;
z3_t_right = zeros(1,length(x3_right));
% Concatenating - right - OBS: To maintain the same distance, offset added at
% section 1 is subtracted at the section 3.
x_track_right = [x1_right x2_right(2:end) x3_right(2:end)];
z_track_right = [z1_t_right z2_t_right(2:end) z3_t_right(2:end)];

figure
hold on ; box on ; grid on ; axis equal
plot(x1_left,z1_t_left)
plot(x2_left,z2_t_left)
plot(x3_left,z3_t_left)
xlabel('Distance x [m]')
ylabel('Distance z [m]')
legend('Stretch 1','Stretch 2','Stretch 3')
title('Input')

figure
hold on ; box on ; grid on ; axis equal
plot(x_track_left,z_track_left,'r','LineWidth',2)
plot(x_track_right,z_track_right,'g','LineWidth',2)
legend('left','right')
xlabel('Distance x [m]')
ylabel('Distance z [m]')
title('Input')

%% Model

c11 = 2*cf + 2*cr;
c12 = b1*cf - b2*cf + b1*cr - b2*cr;
c21 = c12;
c13 = 2*a2*cr - 2*a1*cf;
c31 = c13;
c22 = b1^2*cf + b2^2*cf + b1^2*cr + b2^2*cr;
c23 = a1*b2*cf - a1*b1*cf + a2*b1*cr - a2*b2*cr;
c32 = c23;
c33 = 2*cf*a1^2 + 2*cr*a2^2;

k11 = 2*kf + 2*kr;
k12 = b1*kf - b2*kf + b1*kr - b2*kr;
k21 = k12;
k13 = 2*a2*kr - 2*a1*kf;
k31 = k13;
k22 = kR + b1^2*kf + b2^2*kf + b1^2*kr + b2^2*kr;
k23 = a1*b2*kf - a1*b1*kf + a2*b1*kr - a2*b2*kr;
k32 = k23;
k33 = 2*kf*a1^2 + 2*kr*a2^2;
k24 = -b1*kf - 1/w*kR;
k42 = k24;
k44 = kf + ktf + 1/w^2*kR;
k25 = b2*kf + 1/w*kR;
k52 = k25;
k55 = kf + ktf + 1/w^2*kR;

% Mass
M = [   m   0   0   0   0   0   0   ;
        0   Ix  0   0   0   0   0   ;
        0   0   Iy  0   0   0   0   ;
        0   0   0   mf  0   0   0   ;
        0   0   0   0   mf  0   0   ;
        0   0   0   0   0   mr  0   ;
        0   0   0   0   0   0   mr  ];

% Damping
C = [   c11     c12     c13     -cf     -cf     -cr     -cr     ;
        c21     c22     c23     -b1*cf  b2*cf   b2*cr   -b1*cr  ;
        c31     c32     c33     a1*cf   a1*cf   -a2*cr  -a2*cr  ;
        -cf     -b1*cf  a1*cf   cf      0       0       0       ;
        -cf     b2*cf   a1*cf   0       cf      0       0       ;
        -cr     b2*cr   -a2*cr  0       0       cr      0       ;
        -cr     -b1*cr  -a2*cr  0       0       0       cr      ];
    
% Stiffness
K = [   k11     k12     k13     -kf     -kf     -kr     -kr     ;
        k21     k22     k23     k24     k25     b2*kr   -b1*kr  ;
        k31     k32     k33     a1*kf   a1*kf   -a2*kr  -a2*kr  ;
        -kf     k42     a1*kf   k44     -kR/w^2 0       0       ;
        -kf     k52     a1*kf   -kR/w^2 k55     0       0       ;
        -kr     b2*kr   -a2*kr  0       0       kr+ktr  0       ;
        -kr     -b1*kr  -a2*kr  0       0       0       kr+ktr  ];

% Input
F = [   0   0   0   0   ;
        0   0   0   0   ;
        0   0   0   0   ;
        ktf 0   0   0   ;
        0   ktf 0   0   ;
        0   0   ktr 0   ;
        0   0   0   ktr ];

% State space model
% Coordinates [x phi theta x1 x2 x3 x4]
% States [x phi theta x1 x2 x3 x4 d_x d_phi d_theta d_x1 d_x2 d_x3 d_x4]

A = [   zeros(7)    eye(7)  ;
        -M\K        -M\C       ];

B = [   zeros(7,4)  ;
        M\F         ];

C = eye(14);

D = zeros(14,4);

sys = ss(A,B,C,D);

%% Simulation

% Input
vel = 10;                       % Longitudinal speed of the car             [m/s]
lon_pos_front = vel*time + a1+a2;   % Longitudinal position of the front axle   [m]
% OBS: Added wheelbase!
lon_pos_rear = vel*time;           % Longitudinal position of the rear axle    [m]
% 
u1 = interp1(x_track_left ,z_track_left ,lon_pos_front);    % Front left
u2 = interp1(x_track_right,z_track_right,lon_pos_front);    % Front right
u3 = interp1(x_track_right,z_track_right ,lon_pos_rear);    % Rear right
u4 = interp1(x_track_left ,z_track_left,lon_pos_rear);      % Rear left

figure
hold on ; grid on ; box on
plot(time,u1)
plot(time,u2)
plot(time,u3)
plot(time,u4)
legend('u1','u2','u3','u4')
xlabel('Time [s]')
ylabel('Distance z [m]')
legend('u1','u2','u3','u4')
title('Input')

u_vet = [u1' u2' u3' u4'];

[y,time,~] = lsim(sys,u_vet,time);

% Retrieving states
z       = y(:,1);
ph      = y(:,2);
th      = y(:,3);
z1      = y(:,4);
z2      = y(:,5);
z3      = y(:,6);
z4      = y(:,7);
d_z     = y(:,8);
d_ph    = y(:,9);
d_th    = y(:,10);
d_z1    = y(:,11);
d_z2    = y(:,12);
d_z3    = y(:,13);
d_z4    = y(:,14);

%% Results

figure
hold on ; grid on ; box on
plot(time,z)
plot(time,z1)
plot(time,z2,'--')
plot(time,z3)
plot(time,z4,'--')
xlabel('Time [s]')
ylabel('Vertical coordinate [m]')
legend('z','z1','z2','z3','z4')

figure
hold on ; grid on ; box on
plot(time,ph)
plot(time,th)
xlabel('Time [s]')
ylabel('Angle [rad]')
legend('ph','th')

color = cool(6); % Colormap

figure
% set(gcf,'Position',[50 50 1280 720])  % YouTube: 720p
% set(gcf,'Position',[50 50 854 480])   % YouTube: 480p
set(gcf,'Position',[50 50 640 640])     % Social

hold on ; grid on ; box on ; axis equal
set(gca,'CameraPosition',10*[28.4530   32.3439    6.0810])

% Create and open video writer object
v = VideoWriter('full_car_model.mp4','MPEG-4');
v.Quality   = 100;
% v.FrameRate = fR;
open(v);

for i=1:length(time)

    cla
    
    % Instant position (rear axle)
    x_inst = vel*time(i);
    
    % Tire longitudinal position
    x_1 = a1+a2+x_inst;
    x_2 = a1+a2+x_inst;
    x_3 = x_inst;
    x_4 = x_inst;
    
    % Absolute position edges of the sprung mass
    z_s_1 = z_s_static + z(i) + b1*ph(i) - a1*th(i) ;
    z_s_2 = z_s_static + z(i) - b2*ph(i) - a1*th(i) ;
    z_s_3 = z_s_static + z(i) - b2*ph(i) + a2*th(i) ;
    z_s_4 = z_s_static + z(i) + b1*ph(i) + a2*th(i) ;

    z_sprung = [z_s_1 z_s_2 z_s_3 z_s_4];
    
    % Absolute position of the unsprung masses
    z_u_1 = z1(i) + z_u_static;
    z_u_2 = z2(i) + z_u_static;
    z_u_3 = z3(i) + z_u_static;
    z_u_4 = z4(i) + z_u_static;
    
    z_unsprung = [z_u_1 z_u_2 z_u_3 z_u_4];
    
    % Track passing by:
    set(gca,'xlim',[x_inst-l_win_maring x_inst+l_win],'ylim',[-1.7 1.7],'zlim',[-0.5 2])

    % Track right
    plot3([-10 x_track_right],-b2*ones(1,length(x_track_right)+1),[0 z_track_right],'k','LineWidth',3)
    % Track left
    plot3([-10 x_track_left] , b1*ones(1,length(x_track_left)+1) ,[0 z_track_left],'k','LineWidth',3)

    % Sprung Mass
    fill3([x_1 x_2 x_3 x_4],[b1 -b2 -b2 b1],[z_s_1 z_s_2 z_s_3 z_s_4],color(6,:))
    
    % Spring
    plotSpring(z_u_static , z_s_static , u_vet , y , z_sprung , z_unsprung , i , x_1 , x_2 , x_3 , x_4 , b1 , b2)

    % Unsprung mass
    plot3( a1+a2 + x_inst , b1 , z_u_1 ,'Marker','o','Color','k','MarkerFacecolor',color(1,:),'MarkerSize',10)
    plot3( a1+a2 + x_inst ,-b2 , z_u_2 ,'Marker','o','Color','k','MarkerFacecolor',color(2,:),'MarkerSize',10)
    plot3(         x_inst ,-b2 , z_u_3 ,'Marker','o','Color','k','MarkerFacecolor',color(3,:),'MarkerSize',10)
    plot3(         x_inst , b1 , z_u_4 ,'Marker','o','Color','k','MarkerFacecolor',color(4,:),'MarkerSize',10)

    % Tire 
    plot3(x_inst+a1+a2 ,  b1 , u_vet(i,1),'ko','MarkerFacecolor','k','MarkerSize',10) % Front left
    plot3(x_inst+a1+a2 , -b2 , u_vet(i,2),'ko','MarkerFacecolor','k','MarkerSize',10) % Front right
    plot3(x_inst       , -b2 , u_vet(i,3),'ko','MarkerFacecolor','k','MarkerSize',10) % Rear left
    plot3(x_inst       ,  b1 , u_vet(i,4),'ko','MarkerFacecolor','k','MarkerSize',10) % Rear right
    
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    
    frame = getframe(gcf);
    writeVideo(v,frame);

end

close(v);

%%

function plotSpring(z_u_static,z_s_static,u_vet ,y, z_sprung , z_unsprung , i , x_1 , x_2 , x_3 , x_4, b1 , b2)

    % Spring parameters
    rod_Pct    = 0.11;      % Length rod percentage of total gap
    spring_Pct = 1/3;       % Spring pitch percentage of spring gap 
    spring_wid = 2;         % Spring line width

    % Tire 1 spring  geometry 
    c_t_1 = x_1;            % Longitudinal position
    w_t_1 = 0.07;           % Width
    % Tire 2 spring geometry 
    c_t_2 = x_2;         	% Longitudinal position
    w_t_2 = 0.07;           % Width
    % Tire 3 spring geometry 
    c_t_3 = x_3;         	% Longitudinal position
    w_t_3 = 0.07;           % Width
    % Tire 4 spring geometry 
    c_t_4 = x_4;         	% Longitudinal position
    w_t_4 = 0.07;           % Width
    
    % Suspension 1 spring geometry 
    c_s_1 = x_1;            % Longitudinal position
    w_s_1 = 0.1;            % Width
    % Suspension 2 spring geometry 
    c_s_2 = x_2;         	% Longitudinal position
    w_s_2 = 0.1;            % Width
    % Suspension 3 spring geometry 
    c_s_3 = x_3;         	% Longitudinal position
    w_s_3 = 0.1;            % Width
    % Suspension 4 spring geometry 
    c_s_4 = x_4;         	% Longitudinal position
    w_s_4 = 0.1;            % Width
    
    % Base front and rear
    z_b_1 = u_vet(:,1); % Front left
    z_b_2 = u_vet(:,2); % Front right
    z_b_3 = u_vet(:,3); % Rear right
    z_b_4 = u_vet(:,4); % Rear left

    % Unsprung mass absolute vertical position (lower center point)
    z_u_1 = z_unsprung(1); % Front left
    z_u_2 = z_unsprung(2); % Front right
    z_u_3 = z_unsprung(3); % Rear right
    z_u_4 = z_unsprung(4); % Rear left

    % Spring front and rear (Tire) length without rods
    L_u_1 = (z_u_1 - z_b_1) - 2*rod_Pct * z_u_static; % Front left
    L_u_2 = (z_u_2 - z_b_2) - 2*rod_Pct * z_u_static; % Front right
    L_u_3 = (z_u_3 - z_b_3) - 2*rod_Pct * z_u_static; % Rear right
    L_u_4 = (z_u_4 - z_b_4) - 2*rod_Pct * z_u_static; % Rear left

    % Sprung mass front and rear absolute vertical position (lower center point)
    z_s_1 = z_sprung(1); % Front left
    z_s_2 = z_sprung(2); % Front right
    z_s_3 = z_sprung(3); % Rear right
    z_s_4 = z_sprung(4); % Rear left
    
    % Spring front and rear (Suspension) length without rods
    L_s_1 = (z_s_1 - z_u_1) - 2*rod_Pct*(z_s_static-z_u_static) ;
    L_s_2 = (z_s_2 - z_u_2) - 2*rod_Pct*(z_s_static-z_u_static) ;
    L_s_3 = (z_s_3 - z_u_3) - 2*rod_Pct*(z_s_static-z_u_static) ;
    L_s_4 = (z_s_4 - z_u_4) - 2*rod_Pct*(z_s_static-z_u_static) ;

    % Spring tire 1 front left
    spring_u_1_X = [ 
                c_t_1                               % Start
                c_t_1                               % rod
                c_t_1 + w_t_1                       % Part 1   
                c_t_1 - w_t_1                       % Part 2
                c_t_1 + w_t_1                       % Part 3
                c_t_1 - w_t_1                       % Part 4
                c_t_1 + w_t_1                       % Part 5
                c_t_1 - w_t_1                       % Part 6
                c_t_1                               % Part 7
                c_t_1                               % rod/End
                ];
    
	spring_u_1_Z = [ 
                z_b_1(i)                                            % Start
                z_b_1(i)+  rod_Pct*z_u_static                         % rod
                z_b_1(i)+  rod_Pct*z_u_static                         % Part 1 
                z_b_1(i)+  rod_Pct*z_u_static+  spring_Pct*L_u_1(i)     % Part 2
                z_b_1(i)+  rod_Pct*z_u_static+  spring_Pct*L_u_1(i)     % Part 3
                z_b_1(i)+  rod_Pct*z_u_static+2*spring_Pct*L_u_1(i)   % Part 4
                z_b_1(i)+  rod_Pct*z_u_static+2*spring_Pct*L_u_1(i)   % Part 5
                z_b_1(i)+  rod_Pct*z_u_static+3*spring_Pct*L_u_1(i)   % Part 6
                z_b_1(i)+  rod_Pct*z_u_static+3*spring_Pct*L_u_1(i)   % Part 7
                z_b_1(i)+2*rod_Pct*z_u_static+3*spring_Pct*L_u_1(i) % rod/End
               ]; 
           
    % Spring tire 2 front right
    spring_u_2_X = [ 
                c_t_2                               % Start
                c_t_2                               % rod
                c_t_2+w_t_2                         % Part 1   
                c_t_2-w_t_2                         % Part 2
                c_t_2+w_t_2                         % Part 3
                c_t_2-w_t_2                         % Part 4
                c_t_2+w_t_2                         % Part 5
                c_t_2-w_t_2                         % Part 6
                c_t_2                               % Part 7
                c_t_2                               % rod/End
                ];
    
	spring_u_2_Z = [ 
                z_b_2(i)                                            % Start
                z_b_2(i)+rod_Pct*z_u_static                         % rod
                z_b_2(i)+rod_Pct*z_u_static                         % Part 1 
                z_b_2(i)+rod_Pct*z_u_static+spring_Pct*L_u_2(i)     % Part 2
                z_b_2(i)+rod_Pct*z_u_static+spring_Pct*L_u_2(i)     % Part 3
                z_b_2(i)+rod_Pct*z_u_static+2*spring_Pct*L_u_2(i)   % Part 4
                z_b_2(i)+rod_Pct*z_u_static+2*spring_Pct*L_u_2(i)   % Part 5
                z_b_2(i)+rod_Pct*z_u_static+3*spring_Pct*L_u_2(i)   % Part 6
                z_b_2(i)+rod_Pct*z_u_static+3*spring_Pct*L_u_2(i)   % Part 7
                z_b_2(i)+2*rod_Pct*z_u_static+3*spring_Pct*L_u_2(i) % rod/End
               ]; 
           
    % Spring tire 3 rear right
    spring_u_3_X = [ 
                c_t_3                               % Start
                c_t_3                               % rod
                c_t_3+w_t_3                         % Part 1   
                c_t_3-w_t_3                         % Part 2
                c_t_3+w_t_3                         % Part 3
                c_t_3-w_t_3                         % Part 4
                c_t_3+w_t_3                         % Part 5
                c_t_3-w_t_3                         % Part 6
                c_t_3                               % Part 7
                c_t_3                               % rod/End
                ];
    
	spring_u_3_Z = [ 
                z_b_3(i)                                            % Start
                z_b_3(i)+  rod_Pct*z_u_static                         % rod
                z_b_3(i)+  rod_Pct*z_u_static                         % Part 1 
                z_b_3(i)+  rod_Pct*z_u_static+  spring_Pct*L_u_3(i)     % Part 2
                z_b_3(i)+  rod_Pct*z_u_static+  spring_Pct*L_u_3(i)     % Part 3
                z_b_3(i)+  rod_Pct*z_u_static+2*spring_Pct*L_u_3(i)   % Part 4
                z_b_3(i)+  rod_Pct*z_u_static+2*spring_Pct*L_u_3(i)   % Part 5
                z_b_3(i)+  rod_Pct*z_u_static+3*spring_Pct*L_u_3(i)   % Part 6
                z_b_3(i)+  rod_Pct*z_u_static+3*spring_Pct*L_u_3(i)   % Part 7
                z_b_3(i)+2*rod_Pct*z_u_static+3*spring_Pct*L_u_3(i) % rod/End
                 ];
             
    % Spring tire 4 rear right
    spring_u_4_X = [ 
                c_t_4                               % Start
                c_t_4                               % rod
                c_t_4+w_t_4                         % Part 1   
                c_t_4-w_t_4                         % Part 2
                c_t_4+w_t_4                         % Part 3
                c_t_4-w_t_4                         % Part 4
                c_t_4+w_t_4                         % Part 5
                c_t_4-w_t_4                         % Part 6
                c_t_4                               % Part 7
                c_t_4                               % rod/End
                ];
    
	spring_u_4_Z = [ 
                z_b_4(i)                                            % Start
                z_b_4(i)+  rod_Pct*z_u_static                         % rod
                z_b_4(i)+  rod_Pct*z_u_static                         % Part 1 
                z_b_4(i)+  rod_Pct*z_u_static+  spring_Pct*L_u_4(i)     % Part 2
                z_b_4(i)+  rod_Pct*z_u_static+  spring_Pct*L_u_4(i)     % Part 3
                z_b_4(i)+  rod_Pct*z_u_static+2*spring_Pct*L_u_4(i)   % Part 4
                z_b_4(i)+  rod_Pct*z_u_static+2*spring_Pct*L_u_4(i)   % Part 5
                z_b_4(i)+  rod_Pct*z_u_static+3*spring_Pct*L_u_4(i)   % Part 6
                z_b_4(i)+  rod_Pct*z_u_static+3*spring_Pct*L_u_4(i)   % Part 7
                z_b_4(i)+2*rod_Pct*z_u_static+3*spring_Pct*L_u_4(i) % rod/End
                 ];

    % Spring suspension 1 front left
    spring_s_1_X = [ 
                c_s_1                               % Start
                c_s_1                               % rod
                c_s_1+w_s_1                         % Part 1   
                c_s_1-w_s_1                         % Part 2
                c_s_1+w_s_1                         % Part 3
                c_s_1-w_s_1                         % Part 4
                c_s_1+w_s_1                         % Part 5
                c_s_1-w_s_1                         % Part 6
                c_s_1                               % Part 7
                c_s_1                               % rod/End
                ];
    
	spring_s_1_Z = [ 
                z_u_1                                                                % Start
                z_u_1+  rod_Pct*(z_s_static-z_u_static)                          % rod
                z_u_1+  rod_Pct*(z_s_static-z_u_static)                          % Part 1 
                z_u_1+  rod_Pct*(z_s_static-z_u_static)+  spring_Pct*L_s_1    % Part 2
                z_u_1+  rod_Pct*(z_s_static-z_u_static)+  spring_Pct*L_s_1    % Part 3
                z_u_1+  rod_Pct*(z_s_static-z_u_static)+2*spring_Pct*L_s_1    % Part 4
                z_u_1+  rod_Pct*(z_s_static-z_u_static)+2*spring_Pct*L_s_1    % Part 5
                z_u_1+  rod_Pct*(z_s_static-z_u_static)+3*spring_Pct*L_s_1    % Part 6
                z_u_1+  rod_Pct*(z_s_static-z_u_static)+3*spring_Pct*L_s_1    % Part 7
                z_u_1+2*rod_Pct*(z_s_static-z_u_static)+3*spring_Pct*L_s_1    % rod/End
               ];
           
    % Spring suspension 2 front right
    spring_s_2_X = [ 
                c_s_2                               % Start
                c_s_2                               % rod
                c_s_2+w_s_2                         % Part 1   
                c_s_2-w_s_2                         % Part 2
                c_s_2+w_s_2                         % Part 3
                c_s_2-w_s_2                         % Part 4
                c_s_2+w_s_2                         % Part 5
                c_s_2-w_s_2                         % Part 6
                c_s_2                               % Part 7
                c_s_2                               % rod/End
                ];
    
	spring_s_2_Z = [ 
                z_u_2                                                                % Start
                z_u_2+  rod_Pct*(z_s_static-z_u_static)                          % rod
                z_u_2+  rod_Pct*(z_s_static-z_u_static)                          % Part 1 
                z_u_2+  rod_Pct*(z_s_static-z_u_static)+  spring_Pct*L_s_2    % Part 2
                z_u_2+  rod_Pct*(z_s_static-z_u_static)+  spring_Pct*L_s_2    % Part 3
                z_u_2+  rod_Pct*(z_s_static-z_u_static)+2*spring_Pct*L_s_2    % Part 4
                z_u_2+  rod_Pct*(z_s_static-z_u_static)+2*spring_Pct*L_s_2    % Part 5
                z_u_2+  rod_Pct*(z_s_static-z_u_static)+3*spring_Pct*L_s_2    % Part 6
                z_u_2+  rod_Pct*(z_s_static-z_u_static)+3*spring_Pct*L_s_2    % Part 7
                z_u_2+2*rod_Pct*(z_s_static-z_u_static)+3*spring_Pct*L_s_2    % rod/End
               ];
    
   % Spring suspension 3 rear right
    spring_s_3_X = [ 
                c_s_3                               % Start
                c_s_3                               % rod
                c_s_3+w_s_3                         % Part 1   
                c_s_3-w_s_3                         % Part 2
                c_s_3+w_s_3                         % Part 3
                c_s_3-w_s_3                         % Part 4
                c_s_3+w_s_3                         % Part 5
                c_s_3-w_s_3                         % Part 6
                c_s_3                               % Part 7
                c_s_3                               % rod/End
                ];           
    spring_s_3_Z = [ 
                z_u_3                                                                % Start
                z_u_3+  rod_Pct*(z_s_static-z_u_static)                          % rod
                z_u_3+  rod_Pct*(z_s_static-z_u_static)                          % Part 1 
                z_u_3+  rod_Pct*(z_s_static-z_u_static)+  spring_Pct*L_s_3    % Part 2
                z_u_3+  rod_Pct*(z_s_static-z_u_static)+  spring_Pct*L_s_3    % Part 3
                z_u_3+  rod_Pct*(z_s_static-z_u_static)+2*spring_Pct*L_s_3    % Part 4
                z_u_3+  rod_Pct*(z_s_static-z_u_static)+2*spring_Pct*L_s_3    % Part 5
                z_u_3+  rod_Pct*(z_s_static-z_u_static)+3*spring_Pct*L_s_3    % Part 6
                z_u_3+  rod_Pct*(z_s_static-z_u_static)+3*spring_Pct*L_s_3    % Part 7
                z_u_3+2*rod_Pct*(z_s_static-z_u_static)+3*spring_Pct*L_s_3    % rod/End
               ];

    
   % Spring suspension 4 rear right
    spring_s_4_X = [ 
                c_s_4                               % Start
                c_s_4                               % rod
                c_s_4+w_s_4                         % Part 1   
                c_s_4-w_s_4                         % Part 2
                c_s_4+w_s_4                         % Part 3
                c_s_4-w_s_4                         % Part 4
                c_s_4+w_s_4                         % Part 5
                c_s_4-w_s_4                         % Part 6
                c_s_4                               % Part 7
                c_s_4                               % rod/End
                ];           
    spring_s_4_Z = [ 
                z_u_4                                                                % Start
                z_u_4+  rod_Pct*(z_s_static-z_u_static)                          % rod
                z_u_4+  rod_Pct*(z_s_static-z_u_static)                          % Part 1 
                z_u_4+  rod_Pct*(z_s_static-z_u_static)+  spring_Pct*L_s_4    % Part 2
                z_u_4+  rod_Pct*(z_s_static-z_u_static)+  spring_Pct*L_s_4    % Part 3
                z_u_4+  rod_Pct*(z_s_static-z_u_static)+2*spring_Pct*L_s_4    % Part 4
                z_u_4+  rod_Pct*(z_s_static-z_u_static)+2*spring_Pct*L_s_4    % Part 5
                z_u_4+  rod_Pct*(z_s_static-z_u_static)+3*spring_Pct*L_s_4    % Part 6
                z_u_4+  rod_Pct*(z_s_static-z_u_static)+3*spring_Pct*L_s_4    % Part 7
                z_u_4+2*rod_Pct*(z_s_static-z_u_static)+3*spring_Pct*L_s_4    % rod/End
               ];
           
    % PLOT - TIRE
    plot3(spring_u_1_X, b1*ones(1,length(spring_u_1_X))  ,spring_u_1_Z,'k','LineWidth',spring_wid)
    plot3(spring_u_2_X,-b2*ones(1,length(spring_u_2_X))  ,spring_u_2_Z,'k','LineWidth',spring_wid)
    plot3(spring_u_3_X,-b2*ones(1,length(spring_u_3_X))  ,spring_u_3_Z,'k','LineWidth',spring_wid)
    plot3(spring_u_4_X, b1*ones(1,length(spring_u_4_X))  ,spring_u_4_Z,'k','LineWidth',spring_wid)
    
    % PLOT - SUSPENSION
    plot3(spring_s_1_X, b1*ones(1,length(spring_u_1_X)) , spring_s_1_Z,'k','LineWidth',spring_wid)
    plot3(spring_s_2_X,-b2*ones(1,length(spring_u_2_X)) , spring_s_2_Z,'k','LineWidth',spring_wid)
    plot3(spring_s_3_X,-b2*ones(1,length(spring_u_3_X)) , spring_s_3_Z,'k','LineWidth',spring_wid)
    plot3(spring_s_4_X, b1*ones(1,length(spring_u_4_X)) , spring_s_4_Z,'k','LineWidth',spring_wid)
        
end

function plotDamper(z_s_static , z_u_static , y , i , x_1 , x_2 , a1 , a2 , h_u)
    
    % Damper parameters
    rod_Lower_Pct = 0.1;      % Length lower rod percentage of total gap  
    rod_Upper_Pct = 0.55;      % Length upper rod percentage of total gap
    cylinder_Height_Pct = 0.55;      % Length cylinder percentage of total gap
    damper_line_wid  = 2;   % Damper line width
    
    offset_hor_1 = -0.3; % Horizontal offset 1 front [m]
    offset_hor_2 =  0.3; % Horizontal offset 2 rear [m]
    
    % Suspension 1 spring geometry
    c_1 = x_1;  % Longitudinal position
    w_1 = 0.07;  % Width
    
    % Suspension 2 spring geometry
    c_2 = x_2;  % Longitudinal position
    w_2 = 0.07;  % Width
    
    % Unsprung mass absolute vertical position (lower center point)
    z_u_1 = y(:,3) + z_u_static; % Front
    z_u_2 = y(:,4) + z_u_static; % Rear

    % Sprung mass front and rear absolute vertical position (lower center point)
    z_s_1 = y(:,1) - a1*y(:,2) + z_s_static; % Front
    z_s_2 = y(:,1) + a2*y(:,2) + z_s_static; % Rear
    
    % rod attached to unsprung mass
    rod_u_1_X = [c_1+offset_hor_1 c_1+offset_hor_1]';
    rod_u_1_Y = [z_u_1+h_u  z_u_1+h_u+rod_Lower_Pct*(z_s_static-z_u_static)];
    
    rod_u_2_X = [c_2+offset_hor_2 c_2+offset_hor_2];
    rod_u_2_Y = [z_u_2+h_u  z_u_2+h_u+rod_Lower_Pct*(z_s_static-z_u_static)];
    
    % Damper 1 base cylinder - rod - base 
    cylinder_1_X = [   
                    c_1-w_1+offset_hor_1
                    c_1-w_1+offset_hor_1
                    c_1+w_1+offset_hor_1
                    c_1+w_1+offset_hor_1
                ];
                
    cylinder_1_Y = [
                    z_u_1(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static)+cylinder_Height_Pct*(z_s_static-z_u_static)
                    z_u_1(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static) 
                    z_u_1(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static) 
                    z_u_1(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static)+cylinder_Height_Pct*(z_s_static-z_u_static)
                ];
            
    % Damper 2 base cylinder - rod - base 
    cylinder_2_X = [   
                    c_2-w_2+offset_hor_2
                    c_2-w_2+offset_hor_2
                    c_2+w_2+offset_hor_2
                    c_2+w_2+offset_hor_2
                ];
                
    cylinder_2_Y = [
                    z_u_2(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static)+cylinder_Height_Pct*(z_s_static-z_u_static)
                    z_u_2(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static) 
                    z_u_2(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static) 
                    z_u_2(i)+h_u+rod_Lower_Pct*(z_s_static-z_u_static)+cylinder_Height_Pct*(z_s_static-z_u_static)
                ];
    
    % rod attached to sprung mass 1 front
    rod_s_1_X = [c_1+offset_hor_1  c_1+offset_hor_1];
    rod_s_1_Y = [z_s_1 z_s_1-rod_Upper_Pct*(z_s_static-z_u_static)];

    % rod attached to sprung mass 2 rear
    rod_s_2_X = [c_2+offset_hor_2  c_2+offset_hor_2];
    rod_s_2_Y = [z_s_2 z_s_2-rod_Upper_Pct*(z_s_static-z_u_static)];

    % Piston inside cylinder 1 front
    piston_1_X = [c_1-0.6*w_1+offset_hor_1  c_1+0.6*w_1+offset_hor_1];
    piston_1_Y = [z_s_1-rod_Upper_Pct*(z_s_static-z_u_static) z_s_1-rod_Upper_Pct*(z_s_static-z_u_static)];
    
    % Piston inside cylinder 2 rear
    piston_2_X = [c_2-0.6*w_2+offset_hor_2  c_2+0.6*w_2+offset_hor_2];
    piston_2_Y = [z_s_2-rod_Upper_Pct*(z_s_static-z_u_static) z_s_2-rod_Upper_Pct*(z_s_static-z_u_static)];
    
    % Total damper 1 iteration
    rod_u_1_Y_val  = rod_u_1_Y(i,:);
    rod_s_1_Y_val  = rod_s_1_Y(i,:);
    piston_1_Y_Val = piston_1_Y(i,:);
    
    % Total damper 2 iteration
    rod_u_2_Y_val  = rod_u_2_Y(i,:);
    rod_s_2_Y_val  = rod_s_2_Y(i,:);
    piston_2_Y_Val = piston_2_Y(i,:);

    % PLOT DAMPER 1
    % rods
    plot(rod_u_1_X,rod_u_1_Y_val,'k','LineWidth',damper_line_wid)
    plot(rod_s_1_X,rod_s_1_Y_val,'k','LineWidth',damper_line_wid)
    % Damper parts
    plot(piston_1_X,piston_1_Y_Val,'k','LineWidth',damper_line_wid)
    plot(cylinder_1_X,cylinder_1_Y,'k','LineWidth',damper_line_wid)
    
    % PLOT DAMPER 2
    % rods
    plot(rod_u_2_X,rod_u_2_Y_val,'k','LineWidth',damper_line_wid)
    plot(rod_s_2_X,rod_s_2_Y_val,'k','LineWidth',damper_line_wid)
    % Damper parts
    plot(piston_2_X,piston_2_Y_Val,'k','LineWidth',damper_line_wid)
    plot(cylinder_2_X,cylinder_2_Y,'k','LineWidth',damper_line_wid)

end
















