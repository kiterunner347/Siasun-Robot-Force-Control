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

% mybot.plot(zeros(1,6))
%%
%正向运动学
% Theta=30*ones(1,6);
% Theta=[0 0 0 0 90 0]
% %%验证正逆解的结果
% 
% Theta=Theta/180*pi;%换算成弧度
% T=mybot.fkine(Theta)%求正解的齐次变换矩阵
% W=[-800,+800,-800,+800,-800,+800];
% mybot.plot(Theta,'tilesize',150,'workspace',W);%显示三维动画
% hold on;plot3(0,0,0,'o')
% 
% q1=mybot.ikine(T)*180/pi%求逆解验证关节角
% rpy=tr2rpy(T,'xyz')*180/pi %求未端姿态,工具法兰为绕XYZ轴旋转
% myT=rpy2tr(rpy,'xyz','deg')
% mybot.teach(T,rpy')%显示roll/pitch/yaw angles,GUI可调界面

%该z值为J1关节为原点的z，与simulink仿真应有233.33mm的偏差
%J6关节的坐标系原点到末端有406.67mm的偏差

%%
% 给定各个过程的期望Pd[xyz，rpy]
% 第一阶段，下降阶段
v=0.1;step=[20,200,50];tw=1;
t1=linspace(0,tw,step(1));
timeSample(1)=tw/step(1);

Pd1=[1.2;0;0.6;0;0;pi;1]*ones(1,step(1));
Pd1(3,:)=0.6-v*t1;

% 第二阶段，寻孔阶段
% tw寻孔时间，w螺旋寻孔频率，v径向速度，n总插值点数
x0=1.2;y0=0;z0=0.5;gain=6;
v=0.006*gain;w=10*gain;tw=10;
step(2)=step(2)*gain;
d1=0.03;d2=0.01;
if 2*pi*v/w/(d1-d2)<1
    fprintf('Right!\n');
else
    fact=2*pi*v/w/(d1-d2)
    fprintf('Error!\n');
    return
end

t2=linspace(0,tw,step(2));
Pd2=[x0+v.*t2.*sin(w*t2);y0+v.*t2.*cos(w*t2);z0*ones(1,step(2));
   [0;0;pi;2]*ones(1,step(2))];
timeSample(2)=tw/step(2);
% plot(Pd2(1,:),Pd2(2,:),'-',1.2,0.04,'o');

% 第三阶段，插孔阶段
% xs,ys下降位置,v_down下降速度
for i=1:step(2)
    if norm(Pd2(1:2,i)-[1.2;-0.04])<(d1-d2)/2
        xs=Pd2(1,i);
        ys=Pd2(2,i);
        
        Pd2(:,i+1:end)=[];
        t2(i+1:end)=[];
        step(2)=length(t2);
        
        break;
    end
end

v=0.02;tw=1.5;
t3=linspace(0,tw,step(3));
timeSample(3)=tw/step(3);

Pd3=[xs;ys;0.5;0;0;pi;3]*ones(1,step(3));
Pd3(3,:)=0.5-v*t3;

% 信号合并
threshold=[t1(end),t1(end)+t2(end)];
sgPd.time=[t1,t1(end)+t2,t1(end)+t2(end)+t3];
sgPd.signals.values=[Pd1';Pd2';Pd3'];
sgPd.signals.dimensions=7;

Fd2=[0 0 5 0 0 0]';
Fd3=[0 0 0 0 0 0]';