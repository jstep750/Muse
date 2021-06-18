function r = axisAngle2RotMat_a(n1, n2, a)
    e = cross(n1,n2);
    t = acos(dot(n1,n2)/(norm(n1)*norm(n2)));
    axang = [e(1), e(2), e(3), a*t];
    r = axang2rotm(axang);
end