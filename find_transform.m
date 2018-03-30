function [rot,trans] = find_transform(q,p)
%FIND_TRANSFORM this function returns optimal transformation between p and q 
%
% Example usage: [rot, trans] = find_transforrm(q,p);
% The function return optimal transform q = rot*p + trans;

[rot,trans] = eq_point(q,p,ones(1,length(q)));
end

