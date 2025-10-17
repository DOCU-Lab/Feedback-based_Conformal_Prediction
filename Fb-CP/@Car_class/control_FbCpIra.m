function control_FbCpIra(obj)
%Fb-CP-IRA    

    %Posterior risk calculation
    if obj.i == 0
        beta = 0;       
    else
        rob_position = obj.x(1:2,1);      
        obs_position = squeeze(obj.predicate_data_cali_2(:,obj.i,obj.i,:,:));   
        score_seque = zeros(size(obs_position,1),1);   
        for j = 1:size(obs_position,1)     
            this_score = zeros(size(obs_position,2),1);
            for k = 1:size(obs_position,2)  
                this_score(k) = norm(squeeze(obs_position(j,k,:)) - rob_position') - 2*obj.rr+obj.rs;
            end
            score_seque(j,1) = min(this_score);
        end
        beta = (1+sum(score_seque<0))/(1+size(obs_position,1));
        if beta > obj.alpha_seque(obj.i,obj.i)
            beta = obj.alpha_seque(obj.i,obj.i);
        end
    end

    %allocable risk
    obj.alpha_remain = obj.alpha_remain - beta;

    %begain control & risk allocation

    %control parameters
    cycle_time = 1;                         %iterations number   
    solvertime = 0;                         %solver time
    J_now = NaN;                            %cost
    eta = 0.5;                              %iteration step
    delta = 0.01;                           %Convergence threshold
    pres_thers = 0.01;                      %Active threshold
    alpha = obj.alpha_optim_init;           %init risk allocation
    u_init = obj.u_optim_init;              %init control
    x_init = obj.x_optim_init;              %init state

    %begin interation
    while true  
        %begin control
        yalmip("clear");
        %variable
        u_optim = sdpvar((obj.K-obj.i)*obj.num_u, 1, 'full');
        x_optim = sdpvar((obj.K-obj.i)*obj.num_x, 1, 'full');
        slack = sdpvar(obj.K-obj.i, 3, 'full');
    
        assign(u_optim,u_init);
        assign(x_optim,x_init);

        %constraints
        E_m = obj.E_m(obj.num_ucon*obj.i+1:end, obj.num_u*obj.i+1:end);
        e_m = obj.e_m(obj.num_ucon*obj.i+1:end, 1);
        F_m = obj.F_m(obj.num_xcon*obj.i+1:end, obj.num_x*obj.i+1:end);
        f_m = obj.f_m(obj.num_xcon*obj.i+1:end, 1);
        Con = [E_m*u_optim <= e_m, F_m*x_optim <= f_m];
        Con = [Con, x_optim(1,1) == obj.x(1,1) + (cos(obj.x(3,1))*obj.x(4,1))*obj.Ts,...
                    x_optim(2,1) == obj.x(2,1) + (sin(obj.x(3,1))*obj.x(4,1))*obj.Ts,...
                    x_optim(3,1) == obj.x(3,1) + ((tan(u_optim(1,1))/obj.rr)*obj.x(4,1))*obj.Ts,...
                    x_optim(4,1) == obj.x(4,1) + (u_optim(2,1))*obj.Ts];
        if obj.K-obj.i >= 2
            for i = 2:obj.K-obj.i
                Con = [Con, x_optim((i-1)*obj.num_x+1,1) == x_optim((i-2)*obj.num_x+1,1) + (cos(x_optim((i-2)*obj.num_x+3,1))*x_optim((i-2)*obj.num_x+4,1))*obj.Ts,...
                            x_optim((i-1)*obj.num_x+2,1) == x_optim((i-2)*obj.num_x+2,1) + (sin(x_optim((i-2)*obj.num_x+3,1))*x_optim((i-2)*obj.num_x+4,1))*obj.Ts,...
                            x_optim((i-1)*obj.num_x+3,1) == x_optim((i-2)*obj.num_x+3,1) + ((tan(u_optim((i-1)*obj.num_u+1,1))/obj.rr)*x_optim((i-2)*obj.num_x+4,1))*obj.Ts,...
                            x_optim((i-1)*obj.num_x+4,1) == x_optim((i-2)*obj.num_x+4,1) + (u_optim((i-1)*obj.num_u+2,1))*obj.Ts];
            end
        end
        %collision avoidance
        for j = 1:obj.K-obj.i
            score = squeeze(obj.score_seque_cali_1(:, obj.i+1, obj.i+j));
            position = squeeze(obj.predicate_this_test(obj.i+1, obj.i+j, :, :));
            p = ceil((1+size(score,1))*(1-alpha(j,1)));
            if p > size(score,1)
                p = size(score,1);
            end
            C = score(p);
            obj.c_seque(obj.i+1,obj.i+j) = C;   
            obj.alpha_seque(obj.i+1,obj.i+j) = alpha(j,1);   
            for k = 1:size(position,1)
                Con = [Con, ((x_optim(obj.num_x*(j-1)+1:obj.num_x*j-2) - position(k,:)')' * (x_optim(obj.num_x*(j-1)+1:obj.num_x*j-2) - position(k,:)') + slack(j,k) >= (C + 2*obj.rr+obj.rs)^2): "Con_"+j+"_"+k, slack(j,k)>=0];
            end
        end
        % target constraint
        Con = [Con, norm(x_optim(obj.num_x*(obj.K-obj.i-1)+1:obj.num_x*(obj.K-obj.i)-2) - obj.x_goal(1:2)) <= obj.rr];

        QQ = obj.QQ(obj.num_x*obj.i+1:end, obj.num_x*obj.i+1:end);
        RR = obj.RR(obj.num_u*obj.i+1:end, obj.num_u*obj.i+1:end);

        %objective function
        if obj.i == 0 && cycle_time <= 10     
            obje = u_optim'*RR*u_optim + (x_optim - repmat(obj.x_goal,obj.K-obj.i,1))'*QQ*(x_optim - repmat(obj.x_goal,obj.K-obj.i,1)) + 10^(cycle_time-3)*sum(sum(slack));
        else
            obje = u_optim'*RR*u_optim + (x_optim - repmat(obj.x_goal,obj.K-obj.i,1))'*QQ*(x_optim - repmat(obj.x_goal,obj.K-obj.i,1)) + 100000000*sum(sum(slack));
        end
    
        %solver ops
        ops = sdpsettings('solver','ipopt', 'verbose', 2, 'usex0',1);
        ops.ipopt.max_iter = 10000;
        result = optimize(Con,obje,ops)
        solvertime = solvertime + result.solvertime;
        if result.problem == 0 || result.problem == 16 
            u_optim_value = value(u_optim);
            x_optim_value = value(x_optim);
            u_init = u_optim_value;
            x_init = x_optim_value;
        else
            u_optim_value = u_init;
            x_optim_value = x_init;
            % pause;
            if obj.i ~= 0
                break   
            end
        end

        %Convergence judgment
        J_prev = J_now;              
        J_now = value(obje);           
        if obj.i == 0
            if (cycle_time > 12) && ((norm(J_prev - J_now) < delta) || (J_now > J_prev))
                disp("Reaching the convergence threshold")
                break
            end
        else
            if norm(J_prev - J_now) < delta || J_now > J_prev
                disp("Reaching the convergence threshold")
                break
            end
        end
        
        %Not converged, risk allocation
        %Determine whether the constraint is active (flag = 0 unactive; flag = 1 active)
        slack_optim_value = value(slack);
        active_flag = zeros(obj.K-obj.i, 1);
        for j = 1:obj.K-obj.i
            flag = 0;      
            for k = 1:size(position,1)     
                [pres,~] = check(Con(['Con_', num2str(j), '_', num2str(k)]));
                if pres - slack_optim_value(j,k) <= pres_thers        
                    flag = 1;
                    break;
                end
            end
            active_flag(j,1) = flag;
        end

        %for unactive constraint
        for j = 1:obj.K-obj.i
            if ~active_flag(j,1)
                score = squeeze(obj.score_seque_cali_1(:, obj.i+1, obj.i+j));
                position = squeeze(obj.predicate_this_test(obj.i+1, obj.i+j, :, :));
                min_r_list = zeros(size(position,1),1);
                for k = 1:size(position,1)
                    min_r_list(k,1) = norm(x_optim_value(obj.num_x*(j-1)+1:obj.num_x*j-2) - position(k,:)')^2 + slack_optim_value(j,k) - pres_thers;
                end
                score = score + (2*obj.rr+obj.rs) - sqrt(min(min_r_list));
                alpha(j,1) = (1-eta)*alpha(j,1) + eta*(1+sum(score>0))/(1+obj.data_num_cail_1);
            end
        end
        
        %for active constraint
        alpha_residual = obj.alpha_remain - sum(alpha);
        for j = 1:obj.K-obj.i
            if active_flag(j,1)
                alpha(j,1) = alpha(j,1) + alpha_residual/sum(active_flag);
            end
        end
        
        cycle_time = cycle_time + 1;
    end
    
    %having obtained optimal constrain and risk allocation
    obj.u_optim_init = u_optim_value(obj.num_u+1:end,1);
    obj.x_optim_init = x_optim_value(obj.num_x+1:end,1);
    obj.alpha_optim_init = alpha(2:end,1);

    % optimal control
    obj.u = u_optim_value(1:obj.num_u,1);

    for j = 1:obj.K-obj.i
        obj.x_pred_seque(obj.i+1,obj.i+j,:) = x_optim_value(obj.num_x*(j-1)+1:obj.num_x*j); 
        obj.cost_seque(obj.i+1,obj.i+j) = u_optim_value(obj.num_u*(j-1)+1:obj.num_u*j,1)' * RR(obj.num_u*(j-1)+1:obj.num_u*j,obj.num_u*(j-1)+1:obj.num_u*j) * u_optim_value(obj.num_u*(j-1)+1:obj.num_u*j,1)...
        + (x_optim_value(obj.num_x*(j-1)+1:obj.num_x*j,1) - obj.x_goal)' * QQ(obj.num_x*(j-1)+1:obj.num_x*j,obj.num_x*(j-1)+1:obj.num_x*j) * (x_optim_value(obj.num_x*(j-1)+1:obj.num_x*j,1) - obj.x_goal);
    end

    obj.time_seque(obj.i+1,1) = solvertime;

    obj.i = obj.i + 1;
    disp(obj.method_name)
    disp(obj.alpha)
    disp(obj.i)
end

