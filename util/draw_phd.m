function draw_phd(GM_mu, GM_cov, GM_inten,range,landmark,name)
        [X,Y,phd_surf] = phd2surf(GM_mu, GM_inten, GM_cov, ...
            range, range);
        s=surf(X,Y,phd_surf);
        s.EdgeColor = 'none';
        %set(gca, 'Zdir', 'reverse')
        set(gca, 'Ydir', 'reverse')
        clim([0 3])
        hold on
        scatter3(landmark(1,:),landmark(2,:),ones(1,size(landmark,2)) * 3,'k')
        hold off
        xlabel("X");
        ylabel("Y");
        zlabel("Z");
        xlim ([min(landmark(1,:)), max(landmark(1,:))])
        ylim([min(landmark(2,:)), max(landmark(2,:))])
        zlim([0 3])
        view([0,90])
        colorbar
        grid off
        axis square
        title(name)
end