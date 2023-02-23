function [B_qy] = B(y, q)

    kr1 = 1;
    kr2 = 1; 
    kr3 = 50;
    kr4 = 20;
    ml1 = 25;
    ml2 = 25; 
    ml3 = 10;
    ml4 = 5;
    Il1 = 5;
    Il2 = 5;
    Il3 = 0;
    Il4 = 1;
    Im1 = 0.0001;
    Im2 = 0.0001;
    Im3 = 0.01;
    Im4 = 0.005;
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

    trans0_1 = DH_table(d(1,1), alpha(1,1), theta(1,1), a(1,1));
    trans1_2 = DH_table(d(1,2), alpha(1,2), theta(1,2), a(1,2));
    trans2_3 = DH_table(d(1,3), alpha(1,3), theta(1,3), a(1,3));
    %trans3_4 = DH_table(d(1,4), alpha(1,4), theta(1,4), a(1,4));

    transb_0 = [1 0 0 0; 0 1 0 0; 0 0 1 1; 0 0 0 1];
    transb_1 = (transb_0 * trans0_1);
    transb_2 = (transb_1 * trans1_2);
    transb_3 = (transb_2 * trans2_3);
    %transb_4 = (transb_3 * trans3_4);

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

    Jl2p2 = cp(z1, (pl2-p1));
    Jl3p2 = cp(z1, (pl3-p1));
    Jl4p2 = cp(z1, (pl4-p1));

    Jl3p3 = -z2;
    Jl4p3 = -z2;

    Jl4p4 = cp(z3, pl4-p3);
    
    %Jmp1 = cp(z0, (pm1-p0));
    %Jmp2 = cp(z1, (pm2-p1));
    %Jmp3 = -z2;
    %Jmp4 = cp(z3, pm4-p3);

    Jlo1 = z0; 
    Jlo2 = z1; 
    Jlo3 = zeros(3,1); 
    Jlo4 = z3;

    % For Links
    Jpl1 = [Jl1p1 zeros(3,1) zeros(3,1) zeros(3,1); 0 0 0 0];
    Jpl2 = [Jl2p1 Jl2p2 zeros(3,1) zeros(3,1); 0 0 0 0];
    Jpl3 = [Jl3p1 Jl3p2 Jl3p3 zeros(3,1); 0 0 0 0];
    Jpl4 = [Jl4p1 Jl4p2 Jl4p3 Jl4p4; 0 0 0 0];

    Jol1 = [Jlo1 zeros(3,1) zeros(3,1) zeros(3,1); 0 0 0 0];
    Jol2 = [Jlo1 Jlo2 zeros(3,1) zeros(3,1); 0 0 0 0];
    Jol3 = [Jlo1 Jlo2 Jlo3 zeros(3,1); 0 0 0 0];
    Jol4 = [Jlo1 Jlo2 Jlo3 Jlo4; 0 0 0 0];

    % For Motors
    %Jpm1 = [Jmp1 zeros(3,1) zeros(3,1) zeros(3,1)];
    %Jpm2 = [Jmp1 Jp2 zeros(3,1) zeros(3,1)];
    %Jpm3 = [Jmp1 Jmp2 Jmp3 zeros(3,1)];
    %Jpm4 = [Jmp1 Jmp2 Jmp3 Jmp4];

    Jom1 = [kr1*z0 zeros(3,1) zeros(3,1) zeros(3,1); 0 0 0 0];
    Jom2 = [Jlo1 kr2*z1 zeros(3,1) zeros(3,1); 0 0 0 0];
    Jom3 = [Jlo1 Jlo2 kr3*z2 zeros(3,1); 0 0 0 0];
    Jom4 = [Jlo1 Jlo2 Jlo3 kr4*z3; 0 0 0 0];  
    
    b1 = ((ml1*(Jpl1'*Jpl1))+(Jol1'*Il1*Jol1)+(0)+(Jom1'*Im1*Jom1));
    b2 = ((ml2*(Jpl2'*Jpl2))+(Jol2'*Il2*Jol2)+(0)+(Jom2'*Im2*Jom2));
    b3 = ((ml3*(Jpl3'*Jpl3))+(Jol3'*Il3*Jol3)+(0)+(Jom3'*Im3*Jom3));
    b4 = ((ml4*(Jpl4'*Jpl4))+(Jol4'*Il4*Jol4)+(0)+(Jom4'*Im4*Jom4));

    % B_q = ((ml1*(Jpl1'*Jpl1))+(Jol1'*Il1*Jol1)+(0)+(Jom1'*Im1*Jom1))+((ml2*(Jpl2'*Jpl2))+(Jol2'*Il2*Jol2)+(0)+(Jom2'*Im2*Jom2))+((ml3*(Jpl3'*Jpl3))+(Jol3'*Il3*Jol3)+(0)+(Jom3'*Im3*Jom3))+((ml4*(Jpl4'*Jpl4))+(Jol4'*Il4*Jol4)+(0)+(Jom4'*Im4*Jom4));
    B_q = b1 + b2 + b3 + b4;
    B_qy = B_q*y;
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