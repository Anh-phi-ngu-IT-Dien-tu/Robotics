clc
clf
a=Homogeneous_Transition

d1=15;
d2=0;
d3=0;
a1=0;
a2=5;
a3=5;
alpha1=pi/2;
alpha2=0;
alpha3=0;
PO=[0;0;0;1];
plot3(0,0,0,'o');
grid on
hold on
for theta1=0:pi/5:pi/2
    A0_1=Add_Link(theta1,d1,a1,alpha1);
    P1=A0_1*PO;
    hline1=line([0,P1(1,1)],[0,P1(2,1)],[0,P1(3,1)],'color','red');
    hpoint1=plot3(P1(1,1),P1(2,1),P1(3,1),'or');
    for theta2=0:pi/10:2*pi
        A1_2=Add_Link(theta2,d2,a2,alpha2);
        P2=A0_1*A1_2*PO;
        hline2=line([P1(1,1),P2(1,1)],[P1(2,1),P2(2,1)],[P1(3,1),P2(3,1)],'color','blue');
        hpoint2=plot3(P2(1,1),P2(2,1),P2(3,1),'or');
        for theta3=0:pi/10:2*pi
            A2_3=Add_Link(theta3,d3,a3,alpha3);
            P3=A0_1*A1_2*A2_3*PO;
            hline3=line([P2(1,1),P3(1,1)],[P2(2,1),P3(2,1)],[P2(3,1),P3(3,1)],'color','green');
            hpoint3=plot3(P3(1,1),P3(2,1),P3(3,1),'or');
            pause(0.05)
            delete(hline3)
            delete(hpoint3)
        end
        delete(hline2)
        delete(hpoint2)
    end
    delete(hline1)
    delete(hpoint1)
end
print('end')
close all