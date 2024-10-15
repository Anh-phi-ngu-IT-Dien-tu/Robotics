function [P1,P2,P3,RPY1,RPY2,RPY3]=Forward_Kinatic(handles,theta1,theta2,theta3)
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

    RPY1=ROLL_PITCH_YAW(R0_1);
    RPY2=ROLL_PITCH_YAW(R0_1*R1_2);
    RPY3=ROLL_PITCH_YAW(R0_1*R1_2*R2_3);
    
    VX=[1;0;0;1];
    VY=[0;1;0;1];
    VZ=[0;0;1;1];
    VX1=R0_1*VX;
    VY1=R0_1*VY;
    VZ1=R0_1*VZ;
    VX2=R0_1*R1_2*VX;
    VY2=R0_1*R1_2*VY;
    VZ2=R0_1*R1_2*VZ;
    VX3=R0_1*R1_2*R2_3*VX;
    VY3=R0_1*R1_2*R2_3*VY;
    VZ3=R0_1*R1_2*R2_3*VZ;

    
    
    %% Show robot on UI
    plot3(handles,0,0,0,'or');
    xlim(handles,[-4,4])
    ylim(handles,[-4,4])
    zlim(handles,[0,7])
    xlabel(handles,'x')
    ylabel(handles,'y')
    zlabel(handles,'z')
    
    hold(handles,'on')
    grid(handles,'on')
    
    quiver3(handles,PO(1,1),PO(2,1),PO(3,1),VX(1,1),VX(2,1),VX(3,1),'LineWidth',2,'Color','black')
    quiver3(handles,PO(1,1),PO(2,1),PO(3,1),VY(1,1),VY(2,1),VY(3,1),'LineWidth',2,'Color','cyan')
    quiver3(handles,PO(1,1),PO(2,1),PO(3,1),VZ(1,1),VZ(2,1),VZ(3,1),'LineWidth',2,'Color','green')
    
    quiver3(handles,P1(1,1),P1(2,1),P1(3,1),VX1(1,1),VX1(2,1),VX1(3,1),'LineWidth',2,'Color','black')
    quiver3(handles,P1(1,1),P1(2,1),P1(3,1),VY1(1,1),VY1(2,1),VY1(3,1),'LineWidth',2,'Color','cyan')
    quiver3(handles,P1(1,1),P1(2,1),P1(3,1),VZ1(1,1),VZ1(2,1),VZ1(3,1),'LineWidth',2,'Color','green')
    
    quiver3(handles,P2(1,1),P2(2,1),P2(3,1),VX2(1,1),VX2(2,1),VX2(3,1),'LineWidth',2,'Color','black')
    quiver3(handles,P2(1,1),P2(2,1),P2(3,1),VY2(1,1),VY2(2,1),VY2(3,1),'LineWidth',2,'Color','cyan')
    quiver3(handles,P2(1,1),P2(2,1),P2(3,1),VZ2(1,1),VZ2(2,1),VZ2(3,1),'LineWidth',2,'Color','green')
    
    quiver3(handles,P3(1,1),P3(2,1),P3(3,1),VX3(1,1),VX3(2,1),VX3(3,1),'LineWidth',2,'Color','black')
    quiver3(handles,P3(1,1),P3(2,1),P3(3,1),VY3(1,1),VY3(2,1),VY3(3,1),'LineWidth',2,'Color','cyan')
    quiver3(handles,P3(1,1),P3(2,1),P3(3,1),VZ3(1,1),VZ3(2,1),VZ3(3,1),'LineWidth',2,'Color','green')
    
    line(handles,[0,P1(1,1)],[0,P1(2,1)],[0,P1(3,1)],'LineWidth',3,'color','red');
    plot3(handles,P1(1,1),P1(2,1),P1(3,1),'or');
    line(handles,[P1(1,1),P2(1,1)],[P1(2,1),P2(2,1)],[P1(3,1),P2(3,1)],'LineWidth',3,'color','blue');
    plot3(handles,P2(1,1),P2(2,1),P2(3,1),'or');
    line(handles,[P2(1,1),P3(1,1)],[P2(2,1),P3(2,1)],[P2(3,1),P3(3,1)],'LineWidth',3,'color','ma');
    plot3(handles,P3(1,1),P3(2,1),P3(3,1),'or');
    hold(handles,'off')
end