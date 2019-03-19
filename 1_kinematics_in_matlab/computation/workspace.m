syms theta1 theta2 theta3 theta4
l1 = 4; l2 =4; l3 = 4;
T0404 = [[ cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)), - cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) - cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)),  sin(theta1), cos(theta1)*cos(theta2)*(l2 + l3*cos(theta3)) - l3*cos(theta1)*sin(theta2)*sin(theta3)];
[ cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)), - cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) - sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)), -cos(theta1), cos(theta2)*sin(theta1)*(l2 + l3*cos(theta3)) - l3*sin(theta1)*sin(theta2)*sin(theta3)];
[                         cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)),                           cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)),            0,                    l1 + sin(theta2)*(l2 + l3*cos(theta3)) + l3*cos(theta2)*sin(theta3)];
[                                                                                                                                                         0,                                                                                                                                                           0,            0,                                                                                      1]];

num = 1;
[th1min, th1max, th2min, th2max, th3min, th3max, th4min, th4max] = deal(-20, 20, -20, 20, -20, 20, -20, 20);
inter = 1;
p_num = (th1max/inter - th1min/inter + 1) * (th2max/inter - th2min/inter + 1) * (th3max/inter - th3min/inter + 1) * (th4max/inter - th4min/inter + 1);
[x, y, z] = deal(p_num);

for th1=th1min:inter:th1max
    for th2= th2min:inter:th2max
        for th3=th3min:inter:th3max
            for th4=th4min:inter:th4max
                
                th1rad = th1 * pi / 180;
                th2rad = th1 * pi / 180;
                th3rad = th1 * pi / 180;
                th4rad = th1 * pi / 180;
                
                T04 = subs(T0404,[theta1 theta2 theta3 theta4],[th1rad th2rad th3rad th4rad]);
                T04 = eval(T04);
                
                x(num) = T04(1,4);
                y(num) = T04(2,4);
                z(num) = T04(3,4);
                num=num+1;
                
                %plot3(x,y,z,'r*');hold on;
            end
        end
    end
end
plot3(x,y,z,'r*')
