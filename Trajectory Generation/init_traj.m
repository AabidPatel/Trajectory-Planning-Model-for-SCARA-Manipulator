%main code
clear all;
close all;
clc
Ts=0.001;

q0 = [-0.927293431584587;
     -1.28700579042062;
     0.700000000000000;
     2.21429922200521];
p0 = [0 -0.80 0];
p1 = [0 -0.80 0.5];
p2 = [0.5 -0.6 0.5];
p3 = [0.8 0.0 0.5];
p4 = [0.8 0.0 0.0];

t0 = 0.0;
t1 = 0.6;
t2 = 2.0;
t3 = 3.4;
t4 = 4.0;
t = t0:Ts:t3;
t = t';

% for first segment, qf is p1 and qi is p0

qt1 = [];
tf1 = t1-t0;
qc1_dot_dot = 1.1*(4*(p1-p0)/tf1^2);
tc1 = (tf1/2) - 0.5 * ((qc1_dot_dot*(tf1^2)-4*(p1-p0))/(qc1_dot_dot))^0.5;

for T = t0:Ts:tf1
    if T>=0 && tc1>=T
        qt1(end+1,:) = p0+0.5*qc1_dot_dot*T^2;
    elseif T>tc1 && tf1-tc1>=T
        qt1(end+1,:) = p0+qc1_dot_dot*tc1*(T-tc1/2);
    elseif T>tf1-tc1 && tf1>=T
        qt1(end+1,:) = p1-0.5*qc1_dot_dot*(tf1-T)^2;
    end
end

s1 = [0];
for i = 1:length(qt1)
    if i < length(qt1)
        s1(i+1) = norm(qt1(i+1,:)-qt1(i,:)) + s1(i);
    end
end
s1 = s1';

% for second segment, qf is p2 and qi is p1

qt2 = [];
tf2 = t2-t1;
qc2_dot_dot = 1.1*4*(p2-p1)/tf2^2;
tc2 = (tf2/2) - 0.5 * (((qc2_dot_dot*tf2^2)-(4*(p2-p1)))/(qc2_dot_dot))^0.5;
for T = t0:Ts:tf2
    if T>=0 && tc2>=T
        qt2(end+1,:) = p1+0.5*qc2_dot_dot*T^2;
    elseif T>tc2 && tf2-tc2>=T
        qt2(end+1,:) = p1+qc2_dot_dot*tc2*(T-tc2/2);
    else%if t>(tf2-tc2) && tf2>=t
        qt2(end+1,:) = p2-0.5*qc2_dot_dot*(tf2-T)^2;
    end
end

s2 = [0];
for i = 1:length(qt2)
%     s2(i) = norm(qt2(i,:)-p1);
    if i < length(qt2)
        s2(i+1) = norm(qt2(i+1,:)-qt2(i,:)) + s2(i);
    end
end
s2 = s2';

% for third segment, qf is p3 and qi is p2

qt3 = [];
tf3 = t3-t2;
qc3_dot_dot = 1.1*4*(p3-p2)/tf3^2;
tc3 = (tf3/2) - 0.5 * (((qc3_dot_dot*tf3^2)-(4*(p3-p2)))/(qc3_dot_dot))^0.5;

for T = t0:Ts:tf3
    if T>=0 && tc3>=T
        qt3(end+1,:) = p2+0.5*qc3_dot_dot*T^2;
    elseif T>tc3 && tf3-tc3>=T
        qt3(end+1,:) = p2+qc3_dot_dot*tc3*(T-tc3/2);
    else%if t>tf3-tc3 && tf3>=t
        qt3(end+1,:) = p3-0.5*qc3_dot_dot*(tf3-T)^2;
    end
end

s3 = [0];
for i = 1:length(qt3)
%     s3(i) = norm(qt3(i,:)-p2);
    if i < length(qt3)
        s3(i+1) = norm(qt3(i+1,:)-qt3(i,:)) + s3(i);
    end
end
s3 = s3';

% for fourth segment, qf is p4 and qi is p3

qt4 = [];
tf4 = t4-t3;
qc4_dot_dot = 1.1*4*(p4-p3)/tf4^2;
tc4 = (tf4/2) - 0.5 * (((qc4_dot_dot*tf4^2)-(4*(p4-p3)))/(qc4_dot_dot))^0.5;

for T = t0:Ts:tf4
    if T>=0 && tc4>=T
        qt4(end+1,:) = p3+0.5*qc4_dot_dot*T^2;
    elseif T>tc4 && tf4-tc4>=T
        qt4(end+1,:) = p3+qc4_dot_dot*tc4*(T-tc4/2);
    else%if t>tf4-tc4 && tf4>=t
        qt4(end+1,:) = p4-0.5*qc4_dot_dot*(tf4-T)^2;
    end
end

s4 = [0];
for i = 1:length(qt4)
    if i < length(qt4)
        s4(i+1) = norm(qt4(i+1,:)-qt4(i,:)) + s4(i);
    end
end
s4 = s4';

% Considering Anticipation Time, the trajectory will end in 3.4 seconds:
% For first segment: 0 to 0.6
% For second segment: 0.4 to 1.8
% For thrid segment: 1.6 to 3
% For fourth segment: 2.8 to 3.4

% For first segment:
s1_at = zeros(3401,1);
s1_at(1:601) = s1;
s1_at(602:length(s1_at)) = s1(length(qt1));

% For second segment:
s2_at = zeros(3401,1);
s2_at(401:1801) = s2;
s2_at(1802:length(s2_at)) = s2(length(qt2));

% For third segment:
s3_at = zeros(3401,1);
s3_at(1601:3001) = s3;
s3_at(3002:length(s3_at)) = s3(length(qt3));

% For fourth segment:
s4_at = zeros(3401,1);
s4_at(2801:3401) = s4;

pd = p0 + (s1_at/norm(p1-p0)*(p1-p0))+(s2_at/norm(p2-p1)*(p2-p1))+(s3_at/norm(p3-p2)*(p3-p2))+(s4_at/norm(p4-p3)*(p4-p3));

pd_dot = diff(pd/Ts);
pd_dot(3401,:) = pd_dot(end,:);

pd_dot_dot = diff(pd_dot/Ts);
pd_dot_dot(3401,:) = pd_dot_dot(end,:);

figure(1)
plot3(pd(:,1), pd(:,2), pd(:,3))
title('Trajectory')

figure(2)
plot(pd)
title('Position')
legend('x', 'y', 'z')

figure(3)
plot(pd_dot)
title('Velocity')
legend('x', 'y', 'z')

figure(4)
plot(pd_dot_dot)
title('Acceleration')
legend('x', 'y', 'z')

theta_d = zeros(3401,1);
theta_d_dot = zeros(3401,1);
theta_d_dot_dot = zeros(3401,1);

save('generated_traj', 'Ts', 'q0', 't', 'pd', 'pd_dot', 'pd_dot_dot', 'theta_d_dot_dot', 'theta_d_dot', 'theta_d')
