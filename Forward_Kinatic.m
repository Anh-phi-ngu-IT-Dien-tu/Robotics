function [P1,P2,P3,RPY1,RPY2,RPY3]=Forward_Kinatic(handles,theta1,theta2,theta3,cylinderonoff)
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
    [A0_1,R0_1]=Add_Link(theta1,d1,a1,alpha1)
    [A1_2,R1_2]=Add_Link(theta2,d2,a2,alpha2);
    [A2_3,R2_3]=Add_Link(theta3,d3,a3,alpha3);
    
    %% Calculate joints, end-effector
    P1=A0_1*PO;
    P2=A0_1*A1_2*PO;
    P3=A0_1*A1_2*A2_3*PO;
    
    A0_2=A0_1*A1_2;
    A0_3=A0_1*A1_2*A2_3;

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
    
    if(cylinderonoff==1)
        [X,Y,Z]=cylinder(handles,0.2);
        surf(handles,X,Y,Z*3);
        fill3(handles,X(1,:),Y(1,:),Z(1,:)*3,'k','FaceAlpha',0.8,'EdgeAlpha',1);
        fill3(handles,X(2,:),Y(2,:),Z(2,:)*3,'k','FaceAlpha',0.8,'EdgeAlpha',1);
        
        Z=Z-0.5;
    
        cylin1 = A0_1(1:3,1:3) * [X(:)'; Y(:)'; Z(:)'];
        x= reshape(cylin1(1, :), size(X)) + A0_1(1,4);
        y = reshape(cylin1(2, :), size(Y)) + A0_1(2,4);
        z = reshape(cylin1(3, :), size(Z)) + A0_1(3,4);
        surf(handles,x, y, z);
        fill3(handles,x(1,:),y(1,:),z(1,:),'k','FaceAlpha',0.8,'EdgeAlpha',1);
        fill3(handles,x(2,:),y(2,:),z(2,:),'k','FaceAlpha',0.8,'EdgeAlpha',1);
        
        [X_1,Y_1,Z_1]=cylinder(handles,0.2);
        Z_1=Z_1*2;
        a=Homogeneous_Transition;
        R_cylider=a.Rot_y(pi/2);
    
        cylin2 = A0_2(1:3,1:3) * [X(:)'; Y(:)'; Z(:)'];
        x= reshape(cylin2(1, :), size(X)) + A0_2(1,4);
        y = reshape(cylin2(2, :), size(Y)) + A0_2(2,4);
        z = reshape(cylin2(3, :), size(Z)) + A0_2(3,4);
        surf(handles,x, y, z);
        fill3(handles,x(1,:),y(1,:),z(1,:),'k','FaceAlpha',0.8,'EdgeAlpha',1);
        fill3(handles,x(2,:),y(2,:),z(2,:),'k','FaceAlpha',0.8,'EdgeAlpha',1);
        
        cylin2k = A0_2(1:3,1:3)*R_cylider(1:3,1:3) * [X_1(:)'; Y_1(:)'; Z_1(:)'];
        x_1= reshape(cylin2k(1, :), size(X_1)) + A0_2(1,4);
        y_1 = reshape(cylin2k(2, :), size(Y_1)) + A0_2(2,4);
        z_1 = reshape(cylin2k(3, :), size(Z_1)) + A0_2(3,4);
        surf(handles,x_1, y_1, z_1);
        fill3(handles,x_1(1,:),y_1(1,:),z_1(1,:),'k','FaceAlpha',0.8,'EdgeAlpha',1);
        fill3(handles,x_1(2,:),y_1(2,:),z_1(2,:),'k','FaceAlpha',0.8,'EdgeAlpha',1);
    
        %%
        % cylin3 = A0_3(1:3,1:3) * [X(:)'; Y(:)'; Z(:)'];
        % x= reshape(cylin3(1, :), size(X)) + A0_3(1,4);
        % y = reshape(cylin3(2, :), size(Y)) + A0_3(2,4);
        % z = reshape(cylin3(3, :), size(Z)) + A0_3(3,4);
        % surf(handles,x, y, z);
        % fill3(handles,x(1,:),y(1,:),z(1,:),'k','FaceAlpha',0.8,'EdgeAlpha',1);
        % fill3(handles,x(2,:),y(2,:),z(2,:),'k','FaceAlpha',0.8,'EdgeAlpha',1);
        %%
    
        cylin3k = A0_3(1:3,1:3)*R_cylider(1:3,1:3) * [X_1(:)'; Y_1(:)'; Z_1(:)'];
        x_1= reshape(cylin3k(1, :), size(X_1)) + A0_3(1,4);
        y_1 = reshape(cylin3k(2, :), size(Y_1)) + A0_3(2,4);
        z_1 = reshape(cylin3k(3, :), size(Z_1)) + A0_3(3,4);
        surf(handles,x_1, y_1, z_1);
        fill3(handles,x_1(1,:),y_1(1,:),z_1(1,:),'k','FaceAlpha',0.8,'EdgeAlpha',1);
        fill3(handles,x_1(2,:),y_1(2,:),z_1(2,:),'k','FaceAlpha',0.8,'EdgeAlpha',1);
    else
    end

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