function [Px,Py] = draw_veh(xi,yi,yaw,width,length,color,line_w)

% c.g.

x=0;
y=0;

w=width;
l=length;

CG = [x;y];

P1 = [x+w/2; y-l/2];
P2 = [x+w/2; y+l/2];
P3 = [x-w/2; y+l/2];
P4 = [x-w/2; y-l/2];

% tire
T1_P1 = [P1(1,1)+w/15; P1(2,1)-l/10+l/5];
T1_P2 = [P1(1,1)+w/15; P1(2,1)+l/10+l/5];
T1_P3 = [P1(1,1)-w/15; P1(2,1)+l/10+l/5];
T1_P4 = [P1(1,1)-w/15; P1(2,1)-l/10+l/5];

T2_P1 = [P2(1,1)+w/15; P2(2,1)-l/10-l/5];
T2_P2 = [P2(1,1)+w/15; P2(2,1)+l/10-l/5];
T2_P3 = [P2(1,1)-w/15; P2(2,1)+l/10-l/5];
T2_P4 = [P2(1,1)-w/15; P2(2,1)-l/10-l/5];

T3_P1 = [P3(1,1)+w/15; P3(2,1)-l/10-l/5];
T3_P2 = [P3(1,1)+w/15; P3(2,1)+l/10-l/5];
T3_P3 = [P3(1,1)-w/15; P3(2,1)+l/10-l/5];
T3_P4 = [P3(1,1)-w/15; P3(2,1)-l/10-l/5];

T4_P1 = [P4(1,1)+w/15; P4(2,1)-l/10+l/5];
T4_P2 = [P4(1,1)+w/15; P4(2,1)+l/10+l/5];
T4_P3 = [P4(1,1)-w/15; P4(2,1)+l/10+l/5];
T4_P4 = [P4(1,1)-w/15; P4(2,1)-l/10+l/5];

D_P1 = [x+w/2;y+l/4];
D_P2 = [x    ;y+l/2];
D_P3 = [x-w/2;y+l/4];



T1_set = [T1_P1 T1_P2 T1_P3 T1_P4];
T2_set = [T2_P1 T2_P2 T2_P3 T2_P4];
T3_set = [T3_P1 T3_P2 T3_P3 T3_P4];
T4_set = [T4_P1 T4_P2 T4_P3 T4_P4];
D_set = [D_P1 D_P2 D_P3];

P_set = [CG P1 P2 P3 P4 T1_set T2_set T3_set T4_set D_set];

T=[cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];

P_set_n = T*P_set;

P_set_n(1,:) = P_set_n(1,:)+xi;
P_set_n(2,:) = P_set_n(2,:)+yi;

xx = [P_set_n(1,2:5) P_set_n(1,2)];
yy = [P_set_n(2,2:5) P_set_n(2,2)];

CGx = [P_set_n(1,1)];
CGy = [P_set_n(2,1)];

T1xx=P_set_n(1,6:9);
T1yy=P_set_n(2,6:9);

T2xx=P_set_n(1,10:13);
T2yy=P_set_n(2,10:13);

T3xx=P_set_n(1,14:17);
T3yy=P_set_n(2,14:17);

T4xx=P_set_n(1,18:21);
T4yy=P_set_n(2,18:21);

Dxx=[P_set_n(1,22:24) P_set_n(1,22)];
Dyy=[P_set_n(2,22:24) P_set_n(2,22)];

plot(xx,yy,color,LineWidth=line_w);
hold on
plot(Dxx,Dyy,color,LineWidth=2);
plot(CGx,CGy,'k.',LineWidth=2);
fill(T1xx,T1yy,'k');
fill(T2xx,T2yy,'k');
fill(T3xx,T3yy,'k');
fill(T4xx,T4yy,'k');

Px = CGx;
Py = CGy;

end

