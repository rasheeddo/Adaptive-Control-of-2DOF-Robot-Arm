function [Q] = INV(px,py)
    
    a1 = 211; %mm
    a2 = 171; %mm
    
    L = sqrt(px^2 + py^2);

    pi_Q2 = acos( (px^2 + py^2 - a1^2 - a2^2)/(-2*a1*a2)); % Solution2
    Q2 = pi - pi_Q2;            % Solution1

    gam1 = asin( (a2*sin(Q2))/L );

    alp = atan2(py,px);
   
    Q1 = alp - gam1;   % Solution1
    
    Q = [Q1,Q2];

end

