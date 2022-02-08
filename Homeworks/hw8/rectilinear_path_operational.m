function [pos,vel,acc,jerk]=rectilinear_path_operational(pi,pf,sampling)
    dist=norm(pf-pi);
    s=0:sampling:dist;
    
    pos=pi+s.*(pf-pi)/dist;
    
    vel=ones(3,size(s,2)).*(pf-pi)/dist;
    acc=zeros(3,size(s,2));
    jerk=zeros(3,size(s,2));
    
end