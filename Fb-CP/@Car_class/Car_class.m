classdef Car_class < handle
    %Car class

    properties 
        method_name
        i = 0

        x       
        u       
        x_start 
        x_goal  
        x_trajectory    

        u_optim_init       
        x_optim_init        
        alpha_optim_init   


        alpha                             %total risk
        alpha_remain

        %Data
        predicate_this_test
        %Calibration data 1
        actual_data_cali_1              
        predicate_data_cali_1            
        score_seque_cali_1               
        data_num_cail_1                   
        %Calibration data 2
        actual_data_cali_2                
        predicate_data_cali_2     
        data_num_cail_2

        %record
        cost_seque            
        time_seque                
        x_pred_seque           
        c_seque                     
        alpha_seque                
    end

    properties

        ts = 0.0125;               
        Ts = 0.125;                      
        K = 20                   

        rr = 0.2;                
        rs = 0.1;                

        Eu = [-1,0;1,0;0,-1;0,1;];  
        eu = [pi/6;pi/6;5;5];
        Fx = [-1,0,0,0;1,0,0,0;    
              0,-1,0,0;0,1,0,0;
              0,0,-1,0;0,0,1,0;
              0,0,0,-1;0,0,0,1;];   
        fx = [10;10;10;10;2*pi;2*pi;0;10];   

        R = [100,0;0,1];
        Q = [0,0,0,0;0,1,0,0;0,0,0,0;0,0,0,0];

        num_x  
        num_u 
        num_xcon   
        num_ucon   

        QQ    
        RR    
  
        E_m   
        e_m     
        F_m     
        f_m   

    end

    methods     
        function obj = Car_class(x_start, x_goal, predicate_this_test, actual_data_cali, predicate_data_cali, score_seque_cali, alpha_init, method_name)

            [num_ucon, num_u] = size(obj.Eu);
            [num_xcon, num_x] = size(obj.Fx);

            obj.num_x = num_x;
            obj.num_u = num_u;
            obj.num_xcon = num_xcon;
            obj.num_ucon = num_ucon;

            obj.x_start = x_start;
            obj.x_goal = x_goal;
            obj.x = x_start;
            obj.u = [0;0];     
            obj.x_trajectory = x_start;

            obj.alpha = alpha_init;
            obj.predicate_this_test = predicate_this_test;   
            number_cail_all = 10000;
            obj.method_name = method_name;
            if method_name == "Scp"  %S-CP
                obj.actual_data_cali_1 = actual_data_cali(1:number_cail_all,:,:,:);
                obj.predicate_data_cali_1 = predicate_data_cali(1:number_cail_all,:,:,:,:);
                obj.score_seque_cali_1 = sort(score_seque_cali(1:number_cail_all,:,:),1);
                obj.data_num_cail_1 = number_cail_all;
                obj.data_num_cail_2 = 0;
            elseif method_name == "FbCpAra" || method_name == "FbCpIra"  %Fb-CP
                number_cail_1 = 2000;
                obj.actual_data_cali_1 = actual_data_cali(1:number_cail_1,:,:,:);
                obj.predicate_data_cali_1 = predicate_data_cali(1:number_cail_1,:,:,:,:);
                obj.score_seque_cali_1 = sort(score_seque_cali(1:number_cail_1,:,:),1);
                obj.data_num_cail_1 = number_cail_1;
                
                obj.actual_data_cali_2 = actual_data_cali(number_cail_1+1:number_cail_all,:,:,:);
                obj.predicate_data_cali_2 = predicate_data_cali(number_cail_1+1:number_cail_all,:,:,:,:);        
                obj.data_num_cail_2 = number_cail_all-number_cail_1;
            else
                error("Error method number")
            end
            obj.c_seque = zeros(obj.K,obj.K);  
            obj.alpha_seque = zeros(obj.K,obj.K);
            obj.alpha_remain = obj.alpha;       

            obj.u_optim_init = zeros(obj.K*obj.num_u,1);
 
            obj.x_optim_init = zeros(obj.K*obj.num_x,1);
            obj.x_optim_init(1,1) = x_start(1,1) + (cos(x_start(3,1))*x_start(4,1))*obj.Ts;
            obj.x_optim_init(2,1) = x_start(2,1) + (sin(x_start(3,1))*x_start(4,1))*obj.Ts;
            obj.x_optim_init(3,1) = x_start(3,1);
            obj.x_optim_init(4,1) = x_start(4,1);
            for j = 1:obj.K-1
                obj.x_optim_init(obj.num_x*j+1,1) = obj.x_optim_init(obj.num_x*(j-1)+1,1) + (cos(obj.x_optim_init(obj.num_x*(j-1)+3,1))*obj.x_optim_init(obj.num_x*(j-1)+4,1))*obj.Ts;
                obj.x_optim_init(obj.num_x*j+2,1) = obj.x_optim_init(obj.num_x*(j-1)+2,1) + (sin(obj.x_optim_init(obj.num_x*(j-1)+3,1))*obj.x_optim_init(obj.num_x*(j-1)+4,1))*obj.Ts;
                obj.x_optim_init(obj.num_x*j+3,1) = obj.x_optim_init(obj.num_x*(j-1)+3,1);
                obj.x_optim_init(obj.num_x*j+4,1) = obj.x_optim_init(obj.num_x*(j-1)+4,1);
            end

            obj.alpha_optim_init = (obj.alpha/obj.K)*ones(obj.K,1); 

            obj.cost_seque = zeros(obj.K,obj.K);

            obj.time_seque = zeros(obj.K, 1);

            obj.x_pred_seque = zeros(obj.K,obj.K,obj.num_x);

            obj.E_m = zeros(num_ucon*(obj.K),num_u*(obj.K));
            obj.e_m = zeros(num_ucon*(obj.K),1);
            for i = 1:(obj.K)
                obj.E_m(num_ucon*(i-1)+1:num_ucon*i,num_u*(i-1)+1:num_u*i) = obj.Eu;
                obj.e_m(num_ucon*(i-1)+1:num_ucon*i,1) = obj.eu;
            end
            obj.F_m = zeros(num_xcon*(obj.K),num_x*(obj.K));
            obj.f_m = zeros(num_xcon*(obj.K),1);
            for i = 1:(obj.K)
                obj.F_m(num_xcon*(i-1)+1:num_xcon*i,num_x*(i-1)+1:num_x*i) = obj.Fx;
                obj.f_m(num_xcon*(i-1)+1:num_xcon*i,1) = obj.fx;
            end

            Q = obj.Q;
            obj.QQ = zeros(num_x*(obj.K),num_x*(obj.K));
            obj.RR = zeros(num_u*(obj.K),num_u*(obj.K));
            for i = obj.K:-1:1
                obj.QQ((i - 1)*num_x + 1:i*num_x, (i - 1)*num_x + 1:i*num_x) = Q;
                obj.RR((i - 1)*num_u + 1:i*num_u, (i - 1)*num_u + 1:i*num_u) = obj.R;
                Q = Q*0.5;
            end
        end


        %Output Function
        function current_x = get_x(obj)
            current_x = obj.x;
        end

        function car_actural = get_traj(obj) 
            car_actural = obj.x_trajectory(1:2,:);
        end

        function c_seque = get_c(obj) 
            c_seque = obj.c_seque;
        end

        function alpha_seque = get_alpha(obj) 
            alpha_seque = obj.alpha_seque;
        end

        function x_pred_seque = get_pred_traj(obj)
            x_pred_seque = obj.x_pred_seque;
        end

        function cost_seque = get_cost(obj)
            cost_seque = obj.cost_seque;
        end

        function time_seque = get_time(obj)
            time_seque = obj.time_seque;
        end

    end    
end
