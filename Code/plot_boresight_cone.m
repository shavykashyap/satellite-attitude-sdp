% function plot_boresight_cone(theta_min, v_b, w, initialAttitude, finalAttitude)

    % Load and extract data

    quatTS = simOut.sat_q;
    % quatTS = simOut.q_ref;

    % quatTS = data.data.get('Actual Attitude').Values;   % Simulink.Timeseries
    % t       = quatTS.Time;                         % Nx1 vector (seconds)
    q_val       = quatTS.Data;                         % Nx4 matrix [q1 q2 q3 q4] (scalar first)

    % q_val = q_traj;
    % Normalize all quaternions
    for k = 1:size(q_val, 1)
        q_val(k, :) = q_val(k, :) / norm(q_val(k, :));
    end
    
    q_init = initialAttitude.';
    q_init = q_init / norm(q_init);
    q_fin = finalAttitude.';
    q_fin = q_fin/ norm(q_fin);

    % Plots
    
    % Pre-allocate
    N  = size(q_val,1);
    vI = zeros(N,3);
 
    % convert attitude to inertial coordinates (trajectory)
    for k = 1:N
        v = rotateVector(v_b, q_val(k,:)).';   % 1×3 row
        vI(k,:) = (v / norm(v)).'; 
    end

    % Convert initial and final conditions to inertial coordinates
    v_init = rotateVector(v_b,q_init).';
    v_init = v_init / norm(v_init);
    v_fin = rotateVector(v_b,q_fin).';
    v_fin = v_fin/ norm(v_fin);
    

    % Rotate cone from local axis to sun vector 
    M = 40;                          % number of segments for the rim
    phi = linspace(0,2*pi,M+1);      % angles around the rim
    
    % Rim coordinates on the unit sphere
    r = sin(theta_min);
    z = cos(theta_min)*ones(size(phi));
    x = r*cos(phi);
    y = r*sin(phi);
    rim = [x; y; z];   % 3×(M+1)

    w_unit = w/norm(w);

    z_axis = [0; 0; 1];             % local axis
    if norm(cross(z_axis,w_unit)) < 1e-8
        R_w = eye(3);           % already aligned
    else
        v  = cross(z_axis,w_unit);
        c  = dot(z_axis,w_unit);
        s  = norm(v);
        vx = [  0   -v(3)  v(2);
               v(3)   0   -v(1);
              -v(2) v(1)    0 ];
        R_w = eye(3) + vx + vx*vx*((1-c)/(s^2));   % Rodrigues formula 
    end
    
    rim_rot = R_w * rim;  % 3×(M+1) rotated cone rim

    % Plot
    figure;
    clf;
    hold on;
    axis equal
    grid on;
    set(gcf,'Color','w')          % white background
    
    % sphere
    [XS,YS,ZS] = sphere(120);
    surf(XS,YS,ZS, ...
         'FaceAlpha',0.08, ...
         'EdgeColor','none', ...
         'FaceColor',[0.5 0.5 0.5]);
    
    % Trajectory 
    plot3(vI(:,1), vI(:,2), vI(:,3), 'Color',[0 0.45 0.74],'LineWidth',1.6,'Marker','.','MarkerSize',12);     % Trajectory points                  % Trajectory
   
    scatter3(0,0,0,120,'k','filled','Marker','p');         % Satellite (marked as star)
    
    hStart = scatter3(vI(1,1),vI(1,2),vI(1,3), ...
                  80,'MarkerFaceColor',[0.93 0.69 0.13], ...
                     'MarkerEdgeColor','none');                       % trajectory start 
    hEnd = scatter3(vI(end,1), vI(end,2), vI(end,3), ...
                80,'MarkerFaceColor',[0.46 0.26 0.70]	, ...
                'MarkerEdgeColor','none');                      % end
    
    hSun = quiver3(0, 0, 0,                  ... % tail at origin
            w(1), w(2), w(3),         ...
            0,                        ... % no autoscaling
            'Color',[0.83 0.14 0.37],'LineWidth',2);                                                 % Sun Vector

    % Initial and final conditions quaternions 
    % scatter3(v_init(1,1), v_init(1,2), v_init(1,3),80,'yellow','filled');           % initial
    hDesired = scatter3(v_fin(1,1), v_fin(1,2), v_fin(1,3),80,'MarkerFaceColor',[0.00 0.75 0.75],'MarkerEdgeColor','none');                % final
    
    idx = 1:20:N;                % every 20-th sample
    m   = numel(idx);            % number of arrows you’ll draw
    
    xi = zeros(m,1);             % column m×1
    yi = xi;                     % same size
    zi = xi;
    
    ui = vI(idx,1);              % column m×1
    vi = vI(idx,2);
    wi = vI(idx,3);
    
    quiver3( xi, yi, zi, ...     % bases
             ui, vi, wi, ...     % arrow components
             0, ...              % no autoscaling
             'Color',[0 0.45 0.74], 'LineWidth',0.7 );    % vectors defining the trajectory
    
    
    
    % plot the rotated cone
    plot3(rim_rot(1,:), rim_rot(2,:), rim_rot(3,:),'Color', [0.83 0.14 0.37], 'LineWidth',1.4); % Rim at the top 
   
    for i = 1:M
        plot3([0 rim_rot(1,i)],[0 rim_rot(2,i)],[0 rim_rot(3,i)], ...
              'Color',[0.83 0.14 0.37],'LineWidth',1);
    end         % cone 

    xlabel('x');
    ylabel('y');
    zlabel('z');
    title('Boresight Vector Projected onto a unit sphere');
    view(135,25);

    legend([hStart hEnd hDesired hSun],{'Start','End','Desired', 'Sun vector'}, ...
       'Location','northeastoutside','Box','off');
 % end





% %% Build the Q–space coordinates           (scalar = q4)
% Q = [ q_val(:,1)./q_val(:,4) , ...
%       q_val(:,2)./q_val(:,4) , ...
%       q_val(:,3)./q_val(:,4) ];
% 
% figure; clf; hold on; grid on; axis equal;
% plot3(Q(:,1),Q(:,2),Q(:,3),'-b','LineWidth',1.2);         % trajectory
% plot3(Q(1,1),Q(1,2),Q(1,3),'ko','MarkerFaceColor','y');   % initial
% plot3(Q(end,1),Q(end,2),Q(end,3),'ks','MarkerFaceColor','k'); % final
% 
% xlabel('q_1 / q_4');
% ylabel('q_2 / q_4');
% zlabel('q_3 / q_4');
% title('Attitude trajectory in Q–space');
% view(45,20);        % a pleasant viewing angle
