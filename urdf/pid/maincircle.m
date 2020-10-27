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

%%
%该z值为J1关节为原点的z，与simulink仿真应有233.33mm的偏差
%J6关节的坐标系原点到末端有406.67mm的偏差

%tw寻孔时间，w螺旋寻孔频率，v径向速度，n总插值点数
x0=1.2;y0=0;z0=0.5;
v=0.006;w=1;tw=25;
n=200;
t=linspace(0,tw,n);t=t';
P=[x0+v.*t.*sin(w*t),y0+v.*t.*cos(w*t),z0*ones(n,1)];

%将笛卡尔空间进行逆解计算得到每个关节的角度
ikInt=zeros(1,6);
for i=1:1
    T(:,:,i)=transl(P(i,:))*rpy2tr([0,0,180],'xyz');
    config=mybot.ikine(T(:,:,i),ikInt);
    
    if ~any(config(:))
        fprintf('Someting goes wrong!\n');
        return;
    end
    
    ikInt=config;
    qrt(i,:)=config;
end
%绘制末端轨迹
% W=[-1,1,-1,1,-1,1]*2;
% mybot.plot(qrt,'workspace',W,'view',[120,20],...
%     'trail',{'r','LineWidth',2});

myP=[1.2 0 0.4];
myT=transl(myP)*rpy2tr([0,0,180],'xyz')
config=mybot.ikine(myT,ikInt)
% 0.6->[-0.0000    0.0953   -0.1523   -0.0000    1.6279   -0.0000]
% 0.3->[-0.0000    0.1468    0.1396   -0.0000    1.2844    0.0000]
% 0.4->[-0.0000    0.1125    0.0595   -0.0000    1.3988    0.0000]