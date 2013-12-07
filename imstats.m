function [cen_x, cen_y, area] = imstats(binary_im)
    X_hist=sum(binary_im,1); 
    Y_hist=sum(binary_im,2); 
    [height, width] = size(binary_im);

    X=1:width;
    Y=1:height; 
    cen_x = round(sum(X .* X_hist)  / sum(X_hist)); 
    cen_y = round(sum(Y' .* Y_hist) / sum(Y_hist));
    
    area = sum(sum(binary_im));
end