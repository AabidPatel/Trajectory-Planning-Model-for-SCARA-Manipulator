function [jd]=jacobian_dot(q, q_dot)
    theta1 = q(1);
    theta2 = q(2);
    theta1_d = q_dot(1);
    theta2_d = q_dot(2);

    jacobian_dot = [-0.5*cos(theta1)*(theta1_d)-0.5*cos(theta1+theta2)*(theta1_d+theta2_d) -0.5*cos(theta1+theta2)*(theta1_d+theta2_d) 0 0;
                    -0.5*sin(theta1)*(theta1_d)-0.5*sin(theta1+theta2)*(theta1_d+theta2_d) -0.5*sin(theta1+theta2)*(theta1_d+theta2_d) 0 0;
                    0 0 0 0;
                    0 0 0 0];
    
    jd = jacobian_dot*q_dot;
    jd = double(jd);

end
