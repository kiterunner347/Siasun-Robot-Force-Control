function mytheta = inverse_kinematics(T,start)
%计算逆运动学求解,输入末端齐次变换矩阵及起始角度，输出唯一解。
%推导过程见word。
% T=[ 1.0000 0 0 1.2;
%     0 -1.0000 0 0; 
%     0 0 -1.0000 0.6 ;
%     0 0 0 1.0000];
% start=zeros(6,1);
% [-0.0000    0.0953   -0.1523   -0.0000    1.6279   -0.0000]

%D-H参数
d1 = 0.29333;
d4 = 0.87133;
d6 = 0.10667+0.3;
a1 = 266.67/1000;
a2 = 2/3;

qlim = zeros(6,2);
qlim(1,:)=[-pi,pi];
qlim(2,:)=[-71.48/180*pi,73.52/180*pi];
qlim(3,:)=[-204.22/180*pi,65.78/180*pi];
qlim(4,:)=[-pi,pi];
qlim(5,:)=[-110/180*pi,110/180*pi];
qlim(6,:)=[-pi,pi];

theta=zeros(8,6);
theta1=zeros(4,1);
theta2=zeros(4,1);
theta3=zeros(4,1);
R36=zeros(3,3,4);

%第一步
p06=T(1:3,4);
R06=T(1:3,1:3);
pw=p06-R06*[0 0 d6]';
pwx=pw(1);
pwy=pw(2);
pwz=pw(3);

%第二步
theta1(1)=atan2(pwy,pwx);
theta1(2)=theta1(1);
theta1(3)=theta1(1);
theta1(4)=theta1(3);
s3=( a2^2+d4^2-(pwx-a1*cos(theta1(1)))^2+(pwy-a1*sin(theta1(1)))^2-(pwz-d1)^2 )/2/d4/a2;
c3_1=sqrt(1-s3^2);
c3_2=-c3_1;
theta3(1)=atan2(s3,c3_1);
theta3(2)=theta3(1);
theta3(3)=atan2(s3,c3_2);
theta3(4)=theta3(3);

%theta2的解，其中1，2，3，4与theta3一一对应
c2_1=( sqrt(pwx^2+pwy^2)*(a2-d4*s3)-(pwz-d1)*d4*c3_1 )/( a2^2+d4^2-2*a2*d4*s3 );
s2_1=( -(pwz-d1)*(a2-d4*s3)-sqrt(pwx^2+pwy^2)*d4*c3_1 )/( a2^2+d4^2-2*a2*d4*s3 );
theta2(1)=atan2(s2_1,c2_1);

c2_2=( sqrt(pwx^2+pwy^2)*(a2-d4*s3)-(pwz-d1)*d4*c3_2)/( a2^2+d4^2-2*a2*d4*s3 );
s2_2=( -(pwz-d1)*(a2-d4*s3)-sqrt(pwx^2+pwy^2)*d4*c3_2 )/( a2^2+d4^2-2*a2*d4*s3 );
theta2(2)=atan2(s2_2,c2_2);

c2_3=( -sqrt(pwx^2+pwy^2)*(a2-d4*s3)-(pwz-d1)*d4*c3_1 )/( a2^2+d4^2-2*a2*d4*s3 );
s2_3=( -(pwz-d1)*(a2-d4*s3)+sqrt(pwx^2+pwy^2)*d4*c3_1 )/( a2^2+d4^2-2*a2*d4*s3 );
theta2(3)=atan2(s2_3,c2_3);

c2_4=( -sqrt(pwx^2+pwy^2)*(a2-d4*s3)-(pwz-d1)*d4*c3_2)/( a2^2+d4^2-2*a2*d4*s3 );
s2_4=( -(pwz-d1)*(a2-d4*s3)+sqrt(pwx^2+pwy^2)*d4*c3_2 )/( a2^2+d4^2-2*a2*d4*s3 );
theta2(4)=atan2(s2_4,c2_4);

for i=[1,3]
aa=pwz-d1-cos(theta3(i))*d4;
bb=2*a2-2*sin(theta3(i))*d4;
cc=pwz-d1+cos(theta3(i))*d4;
% theta2(1)=2*atan2(-bb-sqrt(bb^2-4*aa*cc),2*aa)

theta2(i)=2*atan((-bb-sqrt(bb^2-4*aa*cc))/(2*aa))+pi/2;
theta2(i+1)=2*atan((-bb+sqrt(bb^2-4*aa*cc))/(2*aa))+pi/2;

end

theta1(1)=atan2(pwy,pwx);
%theta1(2)=atan2(-pwy,-pwx);
theta1(2)=-atan2(pwy,pwx);
theta1(3)=atan2(pwy,pwx);
%theta1(4)=atan2(-pwy,-pwx);
theta1(4)=-atan2(pwy,pwx);

%第三步、四
R03_1=[sin(theta2(1)+theta3(1))*cos(theta1(1)) sin(theta1(1)) cos(theta2(1)+theta3(1))*cos(theta1(1));...
    sin(theta2(1)+theta3(1))*sin(theta1(1)) -cos(theta1(1)) cos(theta2(1)+theta3(1))*sin(theta1(1));...
    cos(theta2(1)+theta3(1)) 0 -sin(theta2(1)+theta3(1)) ];
R36(:,:,1)=inv(R03_1)*R06;
R03_2=[sin(theta2(2)+theta3(2))*cos(theta1(2)) sin(theta1(2)) cos(theta2(2)+theta3(2))*cos(theta1(2));...
    sin(theta2(2)+theta3(2))*sin(theta1(2)) -cos(theta1(2)) cos(theta2(2)+theta3(2))*sin(theta1(2));...
    cos(theta2(2)+theta3(2)) 0 -sin(theta2(2)+theta3(2))];
R36(:,:,2)=inv(R03_2)*R06;
R03_3=[sin(theta2(3)+theta3(3))*cos(theta1(3)) sin(theta1(3)) cos(theta2(3)+theta3(3))*cos(theta1(3));...
    sin(theta2(3)+theta3(3))*sin(theta1(3)) -cos(theta1(3)) cos(theta2(3)+theta3(3))*sin(theta1(3));...
    cos(theta2(3)+theta3(3)) 0 -sin(theta2(3)+theta3(3))];
R36(:,:,3)=inv(R03_3)*R06;
R03_4=[sin(theta2(4)+theta3(4))*cos(theta1(4)) sin(theta1(4)) cos(theta2(4)+theta3(4))*cos(theta1(4));...
    sin(theta2(4)+theta3(4))*sin(theta1(4)) -cos(theta1(4)) cos(theta2(4)+theta3(4))*sin(theta1(4));...
    cos(theta2(4)+theta3(4)) 0 -sin(theta2(4)+theta3(4))];
R36(:,:,4)=inv(R03_4)*R06;

%第五步
n=1;
for i=1:4
    R=R36(:,:,i);
    a3x=R(1,3);
    a3y=R(2,3);
    a3z=R(3,3);
    s3z=R(3,2);
    n3z=R(3,1);
    
    theta4_1=atan2(-a3y,-a3x);
    theta5_1=atan2(sqrt(a3x^2+a3y^2),a3z);
    theta6_1=atan2(-s3z,n3z);
    theta4_2=atan2(a3y,a3x);
    theta5_2=atan2(-sqrt(a3x^2+a3y^2),a3z);
    theta6_2=atan2(s3z,-n3z);
    
    %输出结果
    % theta(n,:)=[theta1(i) theta2(i)-pi/2 theta3(i) theta4_1 theta5_1 theta6_1];
    theta(n,:)=[theta1(i) theta2(i) theta3(i) theta4_1 theta5_1 theta6_1];
    n=n+1;
    % theta(n,:)=[theta1(i) theta2(i)-pi/2 theta3(i) theta4_2 theta5_2 theta6_2];
    theta(n,:)=[theta1(i) theta2(i) theta3(i) theta4_2 theta5_2 theta6_2];
    n=n+1;
end

%根据关节角度取舍
k = [];
for i = 1:8
    flag = 0;
    for j = 1:6
        if theta(i,j)<qlim(j,1)||theta(i,j)>qlim(j,2)
            flag = 1;
        end
    end
    % flag=0表示这个解对
    
    if ~flag
        k = [k,i];
    end
end
%无解情况
q=zeros(1,6);
if isempty(k)
    q = zeros(1,6);
else
    
theta_kexing = theta(k,:);

%逆解唯一性约束条件
w = [1 1 1 3 3 3];
min_los = 100;
for i = 1:size(theta_kexing,1)
    p = pos(theta_kexing);
    % p_start = pos(start);
    los = 0;
    for j = 1:6
        los = los + w(j)*abs(theta_kexing(i,j)-start(j))*norm(p(:,j)-p(:,j));
    end
        if los<min_los
            q = theta_kexing(i,:);
            min_los = los;
        end
end
end

mytheta=q';

end