function [u_k,omega_k1,q_k1, alpha, att_const] = cvx_sdp2(w,v_b, cos_th, dt, J,  q_N, q_k, omega_N,omega_k)
   
    G1 = [zeros(3,3),eye(3),zeros(3,4)];
    gamma1 = 5 * pi/180;  % angularvelocity (rad/s) 

    G2 = [eye(3),zeros(3,7)];
    gamma2 = 1 ; % torque (mN.m)
    
    omega_skew = [        0    , omega_k(3), -omega_k(2), omega_k(1);
                    -omega_k(3),     0     ,  omega_k(1), omega_k(2);
                     omega_k(2),-omega_k(1),     0      , omega_k(3);
                    -omega_k(1),-omega_k(2), -omega_k(3),     0    ];
    

    Rk1 = 0.5 * dt * omega_skew;
    
    % Update q(k) to q(k+1)
    q_k1 = expm(Rk1) * q_k;
    q_k1 = q_k1/norm(q_k1);

    % q(k+1) to q(k+2) 
    Rk2 = 0.5 * dt * [ -q_k1(4),  q_k1(3), -q_k1(2);
                       -q_k1(3), -q_k1(4),  q_k1(1);
                        q_k1(2), -q_k1(1), -q_k1(4);
                        q_k1(1),  q_k1(2),  q_k1(3)];

    Fk = [-dt*eye(3),  J ,    zeros(3,4);
           zeros(4,3), Rk2,  eye(4)];

    
    J1 = J(1,1);
    J2 = J(2,2);
    J3 = J(3,3);
    
    
    yk = [  J1*omega_k(1) + dt*(J2-J3)*omega_k(2)*omega_k(3);
            J2*omega_k(2) + dt*(J3-J2)*omega_k(3)*omega_k(1);
            J3*omega_k(3) + dt*(J1-J2)*omega_k(1)*omega_k(2);
            q_k1];
    
    A = v_b * w' + w * v_b' - (v_b' * w + cos_th ) * eye(3);
    b = cross(w,v_b);
    d = v_b' * w - cos_th;
    A_tilde = [A,  b;
               b', d];
    mu = max(eig(-A_tilde)) + 1e-6;
    
    H = [zeros(4,6), eye(4,4)];

    Q_N = [q_N(4),  q_N(3), -q_N(2), -q_N(1);
          -q_N(3),  q_N(4),  q_N(1), -q_N(2);
           q_N(2), -q_N(1),  q_N(4), -q_N(3);
           q_N(1),  q_N(2),  q_N(3),  q_N(4)];

    C = [eye(3),zeros(3,1)];  

    Ek = [zeros(3,3), zeros(3,3), zeros(3,4), zeros(3,1);
          zeros(3,3), eye(3),     zeros(3,4), -omega_N  ;
          zeros(4,3), zeros(4,3), zeros(4,4), zeros(4,1);
          zeros(1,3), -omega_N',  zeros(1,4), omega_N'*omega_N] + ...
         [zeros(6,6), zeros(6,4), zeros(6,1);
          zeros(4,6), Q_N'*(C'*C)*Q_N ,  zeros(4,1);
          zeros(1,6), zeros(1,4), zeros(1,1)];

    sq_Ek = sqrtm(Ek) ;
    

    cvx_begin SDP quiet
    cvx_solver mosek

        variable xk(10,1)
        variable alp 


        minimize(alp)
        
        subject to 

        Fk * xk - yk == 0 ;             % dynamics

        abs(G1 * xk) <= gamma1 * [1;1;1];        % angular_velocity (xk(4:6))

        abs(G2 * xk) <= gamma2 * [1;1;1];        % torque (xk(1:3))
        

        [alp, (sq_Ek * [xk;1])';
        (sq_Ek * [xk;1]), eye(11)]  >= 0;               % desired q


        [mu , (H * xk)';
        (H * xk), inv(mu * eye(4) + A_tilde)  ]  >= 0;         % boresight
      
    cvx_end

    % Update
    u_k = xk(1:3);
    omega_k1 = xk(4:6);
    q_k2 = xk(7:10)/norm(xk(7:10));
    alpha = alp;

    att_const = q_k1'* A_tilde * q_k1;

end