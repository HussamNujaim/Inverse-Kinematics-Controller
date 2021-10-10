function out=CheckVector(in)
%
% Vectors are columns

    if all(size(in)>1)
        error('Vector expected')
    elseif size(in,1)==1
        out = in';
    else
        out = in;
    end

end