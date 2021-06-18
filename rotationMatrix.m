function o_R = rotationMatrix(a,b,c)
    aq = angle2quat(a, b, c);
    q4 = aq(1); q1 = aq(2); q2 = aq(3); q3 = aq(4);
    o0 = q1^2 - q2^2 - q3^2 + q4^2;
    o1 = 2*(q1*q2 - q4*q3);
    o2 = 2*(q1*q3 + q4*q2);
    o3 = 2*(q1*q2 + q4*q3);
    o4 = -q1^2 + q2^2 - q3^2 + q4^2;
    o5 = 2*(q2*q3 - q4*q1);
    o6 = 2*(q3*q1 - q4*q2);
    o7 = 2*(q3*q2 + q4*q1);
    o8 = 2*q1^2 - q2^2 + q3^2 + q4^2;
    o_R = [o0 o1 o2; o3 o4 o5; o6 o7 o8 ];
    %o_R = quat2rotm(aq);
end








