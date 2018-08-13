function [ data ] = arrangeData( disparityMap, colorMap, uPatchSize, vPatchSize )
    [ height, width ] = size( disparityMap );
    data        = ones( height*width, 7 );  % [ u v d 1 r g b ]
    i = 1;
    for u = max([1;(uPatchSize-1)/2]):uPatchSize:width
        for v = max([1;(uPatchSize-1)/2]):vPatchSize:height
            data(i,1)   = u;
            data(i,2)   = v;
            data(i,3)   = disparityMap( v, u );
            data(i,5)   = colorMap( v, u, 1 );
            data(i,6)   = colorMap( v, u, 2 );
            data(i,7)   = colorMap( v, u, 3 );
            i           = i+1;
        end
    end
end