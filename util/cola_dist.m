function [dist varargout]= cola_dist(X,Y,c,p)

% This is the MATLAB code for COLA distance proposed in
% 
% P. Barrios, M. Adams, et al, "Metrics for Evaluating Feature-Based Mapping Performance," IEEE Trans. Robotics, Vol. 33, No. 1, 2016.
% http://ba-ngu.vo-au.com/vo/SVV08_OSPA.pdf
% Compute COLA distance between two finite sets X and Y
% Inputs: X,Y-   matrices of column vectors
%        c  -   cut-off parameter
%        p  -   p-parameter for the metric
% Output: scalar distance between X and Y
% Note: the Euclidean 2-norm is used as the "base" distance on the region

if nargout ~=1 & nargout ~=3
   error('Incorrect number of outputs'); 
end

if isempty(X) & isempty(Y)
    dist = 0;

    if nargout == 3
        varargout(1)= {0};
        varargout(2)= {0};
    end
    
    return;
end

if isempty(X) | isempty(Y)
    dist = c;

    if nargout == 3
        varargout(1)= {0};
        varargout(2)= {c};
    end
    
    return;
end


%Calculate sizes of the input point patterns
n = size(X,2);
m = size(Y,2);

%%
% TODO CHANGE COST FUNCTION TO REFLECT COLA METRICS
%Calculate cost/weight matrix for pairings - fast method with vectorization
XX= repmat(X,[1 m]);
YY= reshape(repmat(Y,[n 1]),[size(Y,1) n*m]);
D = reshape(sqrt(sum((XX-YY).^2)),[n m]);
D = min(c,D).^p;

% %Calculate cost/weight matrix for pairings - slow method with for loop
% D= zeros(n,m);
% for j=1:m
%     D(:,j)= sqrt(sum( ( repmat(Y(:,j),[1 n])- X ).^2 )');
% end
% D= min(c,D).^p;

%Compute optimal assignment and cost using the Hungarian algorithm
[assignment,cost]= Hungarian(D);

%Calculate final distance
dist= ( 1/max(m,n)*( c^p*abs(m-n)+ cost ) ) ^(1/p);

%Output components if called for in varargout
if nargout == 3
    varargout(1)= {(1/max(m,n)*cost)^(1/p)};
    varargout(2)= {(1/max(m,n)*c^p*abs(m-n))^(1/p)};
end
    
