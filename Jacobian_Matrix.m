function ve=Jacobian_Matrix(vtheta1,vtheta2,vtheta3,theta1,theta2,theta3)
%% declare links constain
    d1=3;
    d2=0;
    d3=0;
    a1=0;
    a2=2;
    a3=2;
    alpha1=pi/2;
    alpha2=0;
    alpha3=0;
%% create links matrix(Denavit-Hartenberg matrix)
    [A0_1,R0_1]=Add_Link(theta1,d1,a1,alpha1);
    [A1_2,R1_2]=Add_Link(theta2,d2,a2,alpha2);
    [A2_3,R2_3]=Add_Link(theta3,d3,a3,alpha3);

    %%extract zi,pi of joint
    A0_2=A0_1*A1_2;
    A0_3=A1_2*A2_3;

    z0=[0;0;1];
    p0=[0;0;0];

    z1=A0_1(1:3,3);
    p1=A0_1(1:3,4);

    z2=A0_2(1:3,3);
    p2=A0_2(1:3,4);

    z3=A0_3(1:3,3);
    p3=A0_3(1:3,4);

%%Calculate Jacobian matrix
    
    j1_1=cross(z0,(p3-p0));
    j1_2=cross(z1,(p3-p1));
    j1_3=cross(z2,(p3-p2));

    
    j2_1=z0;
    j2_2=z1;
    j2_3=z2;

    J=[j1_1 j1_2 j1_3;j2_1 j2_2 j2_3];

    ve=J*[vtheta1;vtheta2;vtheta3];

   

