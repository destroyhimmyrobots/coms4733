function bw = thresh(im, x, y)
    [height, width, ~] = size(im);
    window_size = 10;%min([round(sqrt(0.003 * (height * width))), 20])
    wind = im(y-window_size:y+window_size, x-window_size:x+window_size, :);
    r = wind(:, :, 1);
    g = wind(:, :, 2);
    b = wind(:, :, 3);
    
    max_r = max(max(r));
    max_g = max(max(g));
    max_b = max(max(b));
    
    min_r = min(min(r)); 
    min_g = min(min(g));  
    min_b = min(min(b));  
    
    r_layer = im(:, :, 1) <= max_r & im(:, :, 1) >= min_r;
    g_layer = im(:, :, 2) <= max_g & im(:, :, 2) >= min_g;
    b_layer = im(:, :, 3) <= max_b & im(:, :, 3) >= min_b;
    
    bw = r_layer & g_layer & b_layer;
    
    L = bwlabel(bw, 8);
    size(L)
    
    label = L(y, x);
    bw = L == label;
end

