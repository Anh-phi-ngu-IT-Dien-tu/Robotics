function RPY=ROLL_PITCH_YAW(R)
si=atan2(R(3,2),R(3,3));
theta=atan2(-R(3,1),sqrt(power(R(3,2),2)+power(R(3,3),2)));
phi=atan2(R(2,1),R(1,1));
RPY=[si;theta;phi];
end