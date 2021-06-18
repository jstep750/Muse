function r = axisAngle2RotMat(n1, n2)
    e = cross(n1,n2);
    t = acos(dot(n1,n2)/(norm(n1)*norm(n2)));
    axang = [e(1), e(2), e(3), t];
    r = axang2rotm(axang);
end