function control_Scp(obj)
%S-CP

    alpha_each_time = obj.alpha / obj.K;
    
    %begin control
    yalmip("clear");
    %define variable
    u_optim = sdpvar((obj.K-obj.i)*obj.num_u, 1, 'full');
    x_optim = sdpvar((obj.K-obj.i)*obj.num_x, 1, 'full');
    slack = sdpvar(obj.K-obj.i, 3, 'full');

    if obj.i > 0
        assign(u_optim,obj.u_optim_init);
        assign(x_optim,obj.x_optim_init);
    end

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
        p = ceil((1+size(score,1))*(1-alpha_each_time));
        C = score(p);
        obj.c_seque(obj.i+1,obj.i+j) = C;    
        obj.alpha_seque(obj.i+1,obj.i+j) = alpha_each_time;   
        for k = 1:size(position,1)
            Con = [Con, ((x_optim(obj.num_x*(j-1)+1:obj.num_x*j-2) - position(k,:)')' * (x_optim(obj.num_x*(j-1)+1:obj.num_x*j-2) - position(k,:)') + slack(j,k) >= (C + 2*obj.rr+obj.rs)^2): "Con_"+j+"_"+k, slack(j,k)>=0];
        end
    end
    % target constraint
    Con = [Con, norm(x_optim(obj.num_x*(obj.K-obj.i-1)+1:obj.num_x*(obj.K-obj.i)-2) - obj.x_goal(1:2)) <= obj.rr];

    QQ = obj.QQ(obj.num_x*obj.i+1:end, obj.num_x*obj.i+1:end);
    RR = obj.RR(obj.num_u*obj.i+1:end, obj.num_u*obj.i+1:end);

    %objective function
    obje = u_optim'*RR*u_optim + (x_optim - repmat(obj.x_goal,obj.K-obj.i,1))'*QQ*(x_optim - repmat(obj.x_goal,obj.K-obj.i,1)) + 100000000*sum(sum(slack));

    %solver ops
    ops = sdpsettings('solver','ipopt', 'verbose', 2, 'usex0',1);
    ops.ipopt.max_iter = 10000;

    result = optimize(Con,obje,ops);
    if result.problem == 0 || result.problem == 16     
        u_optim_value = value(u_optim);
        x_optim_value = value(x_optim);
    else
        u_optim_value = obj.u_optim_init;
        x_optim_value = obj.x_optim_init;
        % pause;
    end

    obj.u_optim_init = u_optim_value(obj.num_u+1:end,1);
    obj.x_optim_init = x_optim_value(obj.num_x+1:end,1);

    %optimal control
    obj.u = u_optim_value(1:obj.num_u,1);

    for j = 1:obj.K-obj.i
        obj.x_pred_seque(obj.i+1,obj.i+j,:) = x_optim_value(obj.num_x*(j-1)+1:obj.num_x*j);
        obj.cost_seque(obj.i+1,obj.i+j) = u_optim_value(obj.num_u*(j-1)+1:obj.num_u*j,1)' * RR(obj.num_u*(j-1)+1:obj.num_u*j,obj.num_u*(j-1)+1:obj.num_u*j) * u_optim_value(obj.num_u*(j-1)+1:obj.num_u*j,1)...
        + (x_optim_value(obj.num_x*(j-1)+1:obj.num_x*j,1) - obj.x_goal)' * QQ(obj.num_x*(j-1)+1:obj.num_x*j,obj.num_x*(j-1)+1:obj.num_x*j) * (x_optim_value(obj.num_x*(j-1)+1:obj.num_x*j,1) - obj.x_goal);
    end

    obj.time_seque(obj.i+1,1) = result.solvertime;

    obj.i = obj.i + 1;
    disp(obj.method_name)
    disp(obj.alpha)
    disp(obj.i)
end

