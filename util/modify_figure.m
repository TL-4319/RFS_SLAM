function modify_figure(fontsize)
grid on; grid minor;
ax=gca;
set(ax,'FontName','Times','Fontsize',fontsize) %,'FontWeight','bold');
box on
set(gcf,'color',[1 1 1]) %to make the backgroung white
set(gca, 'FontName', 'Arial')
end