function meas_vol = sphere_meas_vol(max_range, min_range, HFOV, VFOV)
    meas_vol = (2/3) * (max_range^3 - min_range^3) * HFOV * sin(VFOV/2);
end