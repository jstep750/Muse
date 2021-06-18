function t = degDiff(a,b)
    r = acos(dot(a,b)/(norm(a)*norm(b)));
    t = rad2deg(r);
end
