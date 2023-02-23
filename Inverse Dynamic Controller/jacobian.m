function [jacobian, xe_dot]=jacobian(q, q_dot)
    % Jacobian Code
    %syms theta1 theta2 theta4 d3
    a1 = 0.5;
    a2 = 0.5;
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
    trans3_4 = DH_table(d(1,4), alpha(1,4), theta(1,4), a(1,4));

    transb_0 = [1 0 0 0; 0 1 0 0; 0 0 1 1; 0 0 0 1];
    transb_1 = (transb_0 * trans0_1);
    transb_2 = (transb_1 * trans1_2);
    transb_3 = (transb_2 * trans2_3);
    transb_4 = (transb_3 * trans3_4);

    z0 = [transb_0(1,3); transb_0(2,3); transb_0(3,3)];
    z1 = [transb_1(1,3); transb_1(2,3); transb_1(3,3)];
    z2 = [transb_2(1,3); transb_1(2,3); transb_1(3,3)];
    % z2 is facing upwards but the movement is down, it will be negative
    z3 = [transb_3(1,3); transb_3(2,3); transb_3(3,3)];

    p0 = [transb_0(1,4); transb_0(2,4); transb_0(3,4)];
    p1 = [transb_1(1,4); transb_1(2,4); transb_1(3,4)];
    %p2 = [transb_2(1,4); transb_1(1,4); transb_1(1,4)];
    p3 = [transb_3(1,4); transb_3(2,4); transb_3(3,4)];
    p4 = [transb_4(1,4); transb_4(2,4); transb_4(3,4)];

    jacobian = [cp(z0, (p4-p0)) cp(z1, (p4-p1)) -z2 cp(z3, p4-p3);
                z0 z1 zeros(3,1) z3];
    
    j = jacobian([1:3,6],:);
    xe_dot = j*q_dot;
    xe_dot = double(xe_dot);
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
