function resample_id = low_variance_resample (w, L)
    r = randn(1)/L;
    resample_id = w;
    c = w(1);
    i = 1;
    for m = 1:L 
        u = r + (m - 1) / L;
        while u > c
            i = i+1;
            c = c + w(i);
        end
        resample_id(m) = i;
    end

end