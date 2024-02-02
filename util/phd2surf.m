function [X,Y,phd_surf] = phd2surf (gm_mu, gm_inten, gm_cov, x_range, y_range)
    x_vec = x_range(1):0.5:x_range(2);
    y_vec = y_range(1):0.5:y_range(2);
    [X, Y] = meshgrid(x_vec,y_vec);

    mesh = [X(:) Y(:)];
    phd_surf = zeros(size(X));
    for i = 1:size(gm_inten,2)
        curve = mvnpdf (mesh, gm_mu(1:2,i)', gm_cov(1:2,1:2,i));
        curve = reshape(curve, length(X), length(Y));
        max_height = max(curve,[],"all");
        if max_height ~= 0
            curve = curve .* gm_inten(i) ./ max_height;
            phd_surf = phd_surf + curve;
        end
    end
end