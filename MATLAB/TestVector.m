clear 

U = [7,5];
norm_U = norm(U);

u = U/norm_U;

v = null(u);


figure(1)
plot([0, U(1)],[0, U(2)],'r')
hold on
plot([0,u(1)],[0,u(2)],'b--')
plot([0,v(1)],[0,v(2)],'g--')
axis('equal')