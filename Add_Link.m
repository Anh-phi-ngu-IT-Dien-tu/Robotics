function [A,R]=Add_Link(theta,dz,dx,alpha)
a=Homogeneous_Transition;
A=a.Rot_z(theta)*a.Trans_z(dz)*a.Trans_x(dx)*a.Rot_x(alpha);
R=a.Rot_z(theta)*a.Rot_x(alpha);
end