% d1=3;
% d2=0;
% d3=0;
% a1=0;
% a2=2;
% a3=2;
% alpha1=pi/2;
% alpha2=0;
% alpha3=0;
% 
% syms d1 a2 a3 theta1 theta2 theta3
% 
% [A0_1,R0_1]=Add_Link(theta1,d1,a1,alpha1);
% [A1_2,R1_2]=Add_Link(theta2,d2,a2,alpha2);
% [A2_3,R2_3]=Add_Link(theta3,d3,a3,alpha3);
% 
% A0_3=simplify(A0_1*A1_2*A2_3);

syms c1 a2 c2 a3 c23 s1 s2 s23 d1
pxw=c1*(a2*c2+a3*c23);
pwy=s1*(a2*c2+a3*c23);
pwz=a2*s2+a3*s23+d1;

(pxw^2+pwy^2+pwz^2)





