function T = Tr(theta,d,a,alpha)
%∆Î¥Œ±‰ªªæÿ’Û
    T1 = zeros(4,4);
    T1(1:3,1:3) = rotz(theta);
    T1(3,4) = d;
    T1(4,4) = 1;
    T2 = zeros(4,4);
    T2(1:3,1:3) = rotx(alpha);
    T2(1,4) = a;
    T2(4,4) = 1;
    T = T1*T2;    
end