function r = relativeDegree(A, B, C)
    r = 0;
    while C*(A^r)*B == 0
        r = r+1;
    end
    r = r+1;
end