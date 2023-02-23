% Getting transformation Matrix
function [x] = direct_kin(q)
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

    p = transb_4(1:3,4);
    t = theta1 + theta2 + theta4;

    x = [p;t];

    %x = subs(x, [theta1 theta2 d3 theta4], [q(1) q(2) q(3) q(4)]);
    x = double(x);
end

% Homogenous Transformation Matrix Function

function [mat] = DH_table(d, alpha, theta, a)
    mat = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
           sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
           0 sin(alpha) cos(alpha) d;
           0 0 0 1];
end