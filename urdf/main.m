clear;clc;
close all;

%%
%建立机器人
load I.mat
scale = 1.5;

% l1 = 0.45/scale;
% l2 = 1.0/scale;
% l3 = 1.31228/scale;
% l4 = 0.28445/scale+0.3;
% l5 = 0.4/scale;% 连杆长度mm

l1 = 0.29333;
l2 = 2/3;
l3 = 0.87133;
l4 = 0.10667+0.3;
l5 = 266.67/1000;% 连杆长度mm

m = [205.811,102.832,267.675,25.069,10.895,3.354]; % 质量
r = [-209.222,130.066,-63.899;
    -546.723,46.512,-305.326;
    -1.664,73.292,305.529;
    0,-171.963,-84.563;
    0,83.811,14.196;
    0.011,82,-128.852;
    ]; % 质心坐标 mm

% 机器人初始化
a(1) = l5;  alpha(1) = -pi/2; d(1) = l1; theta(1) = 0;
a(2) = l2;  alpha(2) = 0;     d(2) = 0;  theta(2) = 0;
a(3) = 0;   alpha(3) = -pi/2; d(3) = 0;  theta(3) = 0;
a(4) = 0;   alpha(4) = pi/2;  d(4) = l3;  theta(4) = 0;
a(5) = 0;   alpha(5) = -pi/2; d(5) = 0;  theta(5) = 0;
a(6) = 0;   alpha(6) = 0;     d(6) = l4; theta(6) = 0;

for i = 1:6
    L(i) = Link([theta(i),d(i),a(i),alpha(i),0]);
    L(i).m = m(i)/(scale^3);
    L(i).r = r(i,:)/1000/scale;
    L(i).I = I(:,:,i)/(10^6)/(scale^5);
    L(i).Jm = 0;
end

L(1).qlim=[-pi,pi];
L(2).qlim=[-71.48/180*pi,73.52/180*pi];
L(3).qlim=[-204.22/180*pi,65.78/180*pi];
L(4).qlim=[-pi,pi];
L(5).qlim=[-110/180*pi,110/180*pi];
L(6).qlim=[-pi,pi];

mybot= SerialLink(L,'name','mybot');
mybot.offset = [0 -pi/2 0 0 0 0];

mybot.plot(zeros(1,6))
%%
%正向运动学
Theta=30*ones(1,6);
Theta=[0 0 0 0 90 0]
%%验证正逆解的结果

Theta=Theta/180*pi;%换算成弧度
T=mybot.fkine(Theta)%求正解的齐次变换矩阵
W=[-800,+800,-800,+800,-800,+800];
mybot.plot(Theta,'tilesize',150,'workspace',W);%显示三维动画
hold on;plot3(0,0,0,'o')

q1=mybot.ikine(T)*180/pi%求逆解验证关节角
rpy=tr2rpy(T,'xyz')*180/pi %求未端姿态,工具法兰为绕XYZ轴旋转
myT=rpy2tr(rpy,'xyz','deg')
mybot.teach(T,rpy')%显示roll/pitch/yaw angles,GUI可调界面

%该z值为J1关节为原点的z，与simulink仿真应有233.33mm的偏差
%J6关节的坐标系原点到末端有406.67mm的偏差