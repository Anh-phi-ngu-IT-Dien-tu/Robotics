function [x,y,z]=Forward_Kinetic_No_Graphic(theta1,theta2,theta3)
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
    PO=[0;0;0;1];
    [A0_1,R0_1]=Add_Link(theta1,d1,a1,alpha1);
    [A1_2,R1_2]=Add_Link(theta2,d2,a2,alpha2);
    [A2_3,R2_3]=Add_Link(theta3,d3,a3,alpha3);
    
    %% Calculate joints, end-effector
    P1=A0_1*PO;
    P2=A0_1*A1_2*PO;
    P3=A0_1*A1_2*A2_3*PO;
    
    x=P3(1,1);
    y=P3(2,1);
    z=P3(3,1);
end