function [err_rho_temp,err_alpha_temp,x_temp,y_temp] = calc_LeaderFollower(x,y,R_loc,j,phi,p_LA,FA,dT,k,rho,alpha,rho_d,alpha_d,fullIntegral)

x_temp(1) = x(j,1);
y_temp(1) = y(j,1);

for i = 2:length(x)
        L_LV = sqrt((x(R_loc,i)-x(R_loc,i-1))^2 + (y(R_loc,i)-y(R_loc,i-1))^2)/dT;
        LA = atan2((y(R_loc,i)-y(R_loc,i-1)),(x(R_loc,i)-x(R_loc,i-1)));
        L_AV = (LA-p_LA)/dT;
        p_LA = LA;
        L_Speed = [L_LV; L_AV];
        
        formationDerivative = (k*eye(2))*[(rho_d(j-1)-rho);(alpha_d(j-1)-alpha)];
        followerSpeed = [-(1/cos(alpha)) , 0; -(tan(alpha)/rho) , -1] * ...
            (formationDerivative - [-(cos(phi)) , 0; (sin(phi)/rho) , 0] * L_Speed);
        
        fullDerivative = [-cos(alpha) , 0; sin(alpha)/rho , -1; sin(alpha)/rho , 0]*...
            followerSpeed + [-cos(phi) , 0; sin(phi)/rho , 0; sin(phi)/rho , -1]*...
            L_Speed;
        fullIntegral = fullIntegral + fullDerivative*dT;
        rho = fullIntegral(1);
        alpha = fullIntegral(2);
        phi = fullIntegral(3);
        
        err_rho_temp(1,i-1) = rho_d(j-1)-rho;
        err_alpha_temp(1,i-1) = alpha_d(j-1)-alpha;
        
        
        x_temp(1,i) = x(R_loc,i) - rho*cos(alpha + FA);
        y_temp(1,i) = y(R_loc,i) - rho*sin(alpha + FA);
        FA = phi - alpha + LA - pi;
        
        newHeading = FA + followerSpeed(2)*dT;
        dS = followerSpeed(1)*dT;
end

end

