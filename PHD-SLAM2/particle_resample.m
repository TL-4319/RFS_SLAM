function resample_id = particle_resample(w,L)     

    [notused,sort_id] = sort(-w);
    rv = rand(L,1);
    j = 0;
    threshold = 0;
    resample_id = [];
    while ~isempty(rv)
        j = j + 1;
        threshold = threshold + w(1,sort_id(j));
        rv_len = length(rv);
        idx = find(rv > threshold);
        resample_id = [ resample_id; sort_id(j)*ones(rv_len-length(idx),1) ];
        rv = rv(idx);
        if j == size(sort_id,2)
            rv = [];
        end
    end
    resample_id = resample_id';    
    
end