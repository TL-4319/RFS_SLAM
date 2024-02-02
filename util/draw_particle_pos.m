function draw_particle_pos (particle, fig_num)
    figure(fig_num)
    hold on
    for i=1:size(particle,2)
        scatter3(particle(i).pos(1), particle(i).pos(2), particle(i).pos(3),'r.');
    end
    hold off
end