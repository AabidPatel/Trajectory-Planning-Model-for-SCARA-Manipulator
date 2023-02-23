function [n_q_q_dot] = n(q, q_dot)

    Fm1 = 0.0001; 
    Fm2 = 0.0001;
    Fm3 = 0.01;
    Fm4 = 0.005;
    g = 9.8;
    kr1 = 1;
    kr2 = 1; 
    kr3 = 50;
    kr4 = 20;
    ml1 = 25;
    ml2 = 25; 
    ml3 = 10;
    ml4 = 5;
    a1 = 0.5;
    a2 = 0.5;
    l1 = 0.25;
    l2 = 0.25;
    theta1 = q(1);
    theta2 = q(2);
    theta4 = q(4);
    d3 = q(3);
    d = [0 0 -d3 0];
    alpha = [0 0 0 0];
    theta = [theta1 theta2 0 theta4];
    a = [a1 a2 0 0];
    theta1_dot = q_dot(1);
    theta2_dot = q_dot(2);

    trans0_1 = DH_table(d(1,1), alpha(1,1), theta(1,1), a(1,1));
    trans1_2 = DH_table(d(1,2), alpha(1,2), theta(1,2), a(1,2));
    trans2_3 = DH_table(d(1,3), alpha(1,3), theta(1,3), a(1,3));
    %trans3_4 = DH_table(d(1,4), alpha(1,4), theta(1,4), a(1,4));

    transb_0 = [1 0 0 0; 0 1 0 0; 0 0 1 1; 0 0 0 1];
    transb_1 = (transb_0 * trans0_1);
    transb_2 = (transb_1 * trans1_2);
    transb_3 = (transb_2 * trans2_3);
    % transb_4 = (transb_3 * trans3_4);

    z0 = [transb_0(1,3); transb_0(2,3); transb_0(3,3)];
    z1 = [transb_1(1,3); transb_1(2,3); transb_1(3,3)];
    z2 = [transb_2(1,3); transb_1(2,3); transb_1(3,3)];
    % z2 is facing upwards but the movement is down, it will be negative
    z3 = [transb_3(1,3); transb_3(2,3); transb_3(3,3)];

    p0 = [transb_0(1,4); transb_0(2,4); transb_0(3,4)];
    p1 = [transb_1(1,4); transb_1(2,4); transb_1(3,4)];
    p2 = [transb_2(1,4); transb_2(2,4); transb_2(3,4)];
    p3 = [transb_3(1,4); transb_3(2,4); transb_3(3,4)];
    % p4 = [transb_4(1,4); transb_4(2,4); transb_4(3,4)];

    % Position of Motors
    pm1 = p0;
    pm2 = p1;
    pm3 = p2;
    pm4 = p3;

    % Position of links by considering Center of Mass
    pl1 = pm1 + [l1*cos(theta1); l1*sin(theta1); 0];
    pl2 = pm2 + [l2*cos(theta1+theta2); l2*sin(theta1+theta2); 0];
    pl3 = pm3;
    pl4 = pm4;

    Jl1p1 = cp(z0, (pl1-p0));
    Jl2p1 = cp(z0, (pl2-p0));
    Jl3p1 = cp(z0, (pl3-p0));
    Jl4p1 = cp(z0, (pl4-p0));
    Jl1p2 = cp(z1, (pl1-p1));
    Jl2p2 = cp(z1, (pl2-p1));
    Jl3p2 = cp(z1, (pl3-p1));
    Jl4p2 = cp(z1, (pl4-p1));
    Jl1p3 = zeros(3,1);
    Jl2p3 = zeros(3,1);
    Jl3p3 = -z2;
    Jl4p3 = -z2;
    Jl1p4 = cp(z3, pl1-p3);
    Jl2p4 = cp(z3, pl2-p3);
    Jl3p4 = cp(z3, pl3-p3);
    Jl4p4 = cp(z3, pl4-p3);

    F = [kr1^2*Fm1 0 0 0;
        0 kr2^2*Fm2 0 0;
        0 0 kr3^2*Fm3 0;
        0 0 0 kr4^2*Fm4];

    C = [(-55*sin(theta2)/8)*theta2_dot, (-55*sin(theta2)/8)*(theta1_dot+theta2_dot), 0, 0;
        (55*sin(theta2)/8)*theta1_dot, 0, 0, 0;
        0, 0, 0, 0;
        0, 0, 0, 0];

    g_0 = [0 0 -g]';

    g1_q = -((ml1*g_0'*Jl1p1)+(ml2*g_0'*Jl2p1)+(ml3*g_0'*Jl3p1)+(ml4*g_0'*Jl4p1));
    g2_q = -((ml1*g_0'*Jl1p2)+(ml2*g_0'*Jl2p2)+(ml3*g_0'*Jl3p2)+(ml4*g_0'*Jl4p2));
    g3_q = -((ml1*g_0'*Jl1p3)+(ml2*g_0'*Jl2p3)+(ml3*g_0'*Jl3p3)+(ml4*g_0'*Jl4p3));
    g4_q = -((ml1*g_0'*Jl1p4)+(ml2*g_0'*Jl2p4)+(ml3*g_0'*Jl3p4)+(ml4*g_0'*Jl4p4));

    g_q = [g1_q; g2_q; g3_q; g4_q];

    n_q_q_dot = C*q_dot + F*q_dot + g_q;

end

function [mat] = cp(z, p)
    mat = [z(2,1)*p(3,1) - z(3,1)*p(2,1); 
         -(z(1,1)*p(3,1) - z(3,1)*p(1,1)); 
           z(1,1)*p(2,1) - z(2,1)*p(1,1)];
end

function [mat] = DH_table(d, alpha, theta, a)
    mat = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
           sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
           0 sin(alpha) cos(alpha) d;
           0 0 0 1];
end
