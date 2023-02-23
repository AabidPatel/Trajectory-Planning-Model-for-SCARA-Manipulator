function [q_dot_dot]=jacobian_inverse(x, q)
    % Inversing Jacobian Matrix
    [J, ~] = jacobian(q, 0);
    Jacobian = J([1:3,6],:);
    q_dot_dot = Jacobian\x;
    q_dot_dot = double(q_dot_dot);
end
