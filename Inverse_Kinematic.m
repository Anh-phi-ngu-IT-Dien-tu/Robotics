function [theta1,theta2,theta3,valid]=Inverse_Kinematic(x3,y3,z3)
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


    %% calculate 
    z3=z3-d1;
    y3=y3=d1;
    x3=x3-d1;

    A0_3=[x3;y3;z3;1]*pinv([0;0;0;1])

    c3=(x3^2+y3^2+z3^2-a2^2-a3^2)/(2*a2*a3);
    if(sqrt(x3^2+y3^2+z3^2)>a2+a3||sqrt(x3^2+y3^2+z3^2)<abs(a2-a3))
        theta1=0;
        theta2=0;
        theta3=0;
        valid=0;
        return
    else
    end

    

    theta3=acos(c3);
    s3=sin(theta3);

    c2=(sqrt(x3^2+y3^2)*(a2+a3*c3)+z3*a3*s3)/(a2^2+a3^2+2*a2*a3*c3);

    s2=-(sqrt(x3^2+y3^2)*(a3*s3)+z3*(a2+a3*c3))/(a2^2+a3^2+2*a2*a3*c3);

    theta2=atan2(s2,c2);

    theta1=atan2(y3,x3);

    valid=1;

    



