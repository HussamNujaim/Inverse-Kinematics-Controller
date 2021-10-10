function pinvJ = pinv_damped(J)

[U,S,V] = svd(J);

threshold = 0.01;
lambda = 0.01;

for i=1:size(J,1)
    eigval(i) = S(i,i);
end

Sinv = zeros(size(J,2), size(J,1));

for i=1:size(J,1)

    p(i) = cosRialzato(eigval(i), lambda, threshold);
    Sinv(i,i) = eigval(i)/(eigval(i)^2 + p(i));
   
    
end

pinvJ = V * Sinv * U';

end

function reg = cosRialzato(sigma, lambda, threshold)

    sigma = abs(sigma);
    
    if sigma < threshold
        
        tmp = (sigma/threshold)* pi;
        reg = lambda * (0.5 * cos(tmp) + 0.5);
        fprintf('Smorzo\n')
    else
        reg = 0;
        
    end

end