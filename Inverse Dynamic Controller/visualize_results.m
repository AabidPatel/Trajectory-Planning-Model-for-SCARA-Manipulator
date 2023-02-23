function y = visualize_results(t, q, vel_error, pos_error)
    
%     tracking = tracking';
%     tracking_theta = tracking_theta';
    addpath("visualization/")

    t1 = q(1,:,:);
    t2 = q(2,:,:);
    d_3 = q(3,:,:);
    t4 = q(4,:,:);
    theta1(:,1) = t1(1,1,:);
    theta2(:,1) = t2(1,1,:);
    d3(:,1) = d_3(1,1,:);
    theta4(:,1) = t4(1,1,:);

    v1 = vel_error(1,:,:);
    v2 = vel_error(2,:,:);
    v3 = vel_error(3,:,:);
    v4 = vel_error(4,:,:);
    ve1(:,1) = v1(1,1,:);
    ve2(:,1) = v2(1,1,:);
    ve3(:,1) = v3(1,1,:);
    ve4(:,1) = v4(1,1,:);

    p1 = pos_error(1,:,:);
    p2 = pos_error(2,:,:);
    p3 = pos_error(3,:,:);
    p4 = pos_error(4,:,:);
    pe1(:,1) = p1(1,1,:);
    pe2(:,1) = p2(1,1,:);
    pe3(:,1) = p3(1,1,:);
    pe4(:,1) = p4(1,1,:);

    figure(1)
    subplot(5,1,1); plot(t, pe1); title('joint 1 Position Error');
    subplot(5,1,2); plot(t, pe2); title('joint 2 Position Error');
    subplot(5,1,3); plot(t, pe3); title('joint 3 Position Error');
    subplot(5,1,4); plot(t, pe4); title('joint 4 Position Error');
    subplot(5,1,5);

    figure(2)
    subplot(5,1,1); plot(t, ve1); title('joint 1 Velocity Error');
    subplot(5,1,2); plot(t, ve2); title('joint 2 Velocity Error');
    subplot(5,1,3); plot(t, ve3); title('joint 3 Velocity Error');
    subplot(5,1,4); plot(t, ve4); title('joint 4 Velocity Error');
    subplot(5,1,5);

    figure(3)
    subplot(5,1,1); plot(t, theta1); title('Joint 1 trajectory');
    subplot(5,1,2); plot(t, theta2); title('Joint 2 trajectory');
    subplot(5,1,3); plot(t, d3); title('Joint 3 trajectory');
    subplot(5,1,4); plot(t, theta4); title('Joint 4 trajectory');
    subplot(5,1,5);
end