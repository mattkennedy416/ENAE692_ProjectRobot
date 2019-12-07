



function jointPositions = forwardKinematics(dh, pT, includeOrigin)
% need to get the updated position for the new q

jointPositions = zeros(3,size(dh,1));
T_total = eye(4);
for link = 1:size(dh,1)
   T_total = T_total * Transform( dh.theta_i(link), dh.alpha_i1(link), dh.a_i1(link), dh.d_i(link) );
   jointPositions(:,link) = transpose(T_total(1:3,4));
end

% and then to get the tool tip at the end
% pT(3) becomes the d and pT(1) becomes the a? 
T_total = T_total * Transform( 0, 0, pT(3), pT(1) );
jointPositions(:,link+1) = transpose(T_total(1:3,4));

if nargin == 3
    jointPositions = cat(2, [0;0;0], jointPositions);
end


end







