classdef Homogeneous_Transition
   properties
        Value 
   end
   methods
       function y=Rot_z(obj,angle_z)
           y=[cos(angle_z) -sin(angle_z) 0 0;
               sin(angle_z) cos(angle_z) 0 0;
               0 0 1 0;
               0 0 0 1];
       end
       function y=Rot_y(obj,angle_y)
            y=[cos(angle_y) 0 -sin(angle_y) 0;
               0 1 0 0;
               sin(angle_y) 0 cos(angle_y) 0;
               0 0 0 1];
       end
       function y=Rot_x(obj,angle_x)
           y=[1 0 0 0;
              0 cos(angle_x) -sin(angle_x) 0;
              0 sin(angle_x) cos(angle_x) 0;
               0 0 0 1];
       end
       function y=Trans_z(obj,dz)
           y=[1 0 0 0;
              0 1 0 0;
              0 0 1 dz;
              0 0 0 1];
       end
       function y=Trans_y(obj,dy)
           y=[1 0 0 0;
              0 1 0 dy;
              0 0 1 0;
              0 0 0 1];
       end
       function y=Trans_x(obj,dx)
           y=[1 0 0 dx;
              0 1 0 0;
              0 0 1 0;
              0 0 0 1];
       end
   end
end