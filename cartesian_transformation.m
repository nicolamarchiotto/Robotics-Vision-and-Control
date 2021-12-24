function xq = cartesian_transformation(dh, qq)

for i=1:length(qq)
    xq(i,:)= kinematics(dh, qq(i,:));
end

end