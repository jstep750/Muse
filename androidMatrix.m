function and_R = androidMatrix(A, E)
    A = [A(1), A(2), A(3)];
    H = cross(E,A);
    nH = [H(1)/norm(H), H(2)/norm(H), H(3)/norm(H)];
    nA = [A(1)/norm(A), H(2)/norm(H), H(3)/norm(H)];
    M = cross(nA, nH);
    
    and_R = [nH(1) nH(2) nH(3); M(1) M(2) M(3); nA(1) nA(2) nA(3);];
end



