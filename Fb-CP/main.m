%Main
%Importing Data
%actual_data(4D)
%predicate_data(5D)
%score_seque(3D)
%robot_start_point(2D)
%robot_goal_point(2D)
actual_data = importdata(".\data_file\actual_data.mat");
predicate_data = importdata(".\data_file\predicate_data.mat");
score_seque = importdata(".\data_file\score_seque.mat");
robot_start_point = importdata(".\data_file\robot_start_point.mat");
robot_goal_point = importdata(".\data_file\robot_goal_point.mat");
%Randomly shuffle
randIndex = importdata(".\data_file\randIndex.mat");
actual_data = actual_data(randIndex,:,:,:);
predicate_data = predicate_data(randIndex,:,:,:,:);
score_seque = score_seque(randIndex,:,:);
robot_start_point = robot_start_point(randIndex,:);
robot_goal_point = robot_goal_point(randIndex,:);
%Calibration data
actual_data_cali = actual_data(1:12000,:,:,:);
predicate_data_cali = predicate_data(1:12000,:,:,:,:);
score_seque_cali = score_seque(1:12000,:,:);
%test data
actual_data_test = actual_data(12001:end,:,:,:);
predicate_data_test = predicate_data(12001:end,:,:,:,:);
robot_start_point_test = robot_start_point(12001:end,:);
robot_goal_point_test = robot_goal_point(12001:end,:);

%simulation
T = 200;
alpha = 0.05;
alpha_name = "alpha_05";
method_list = ["Scp", "FbCpAra", "FbCpIra"];

for j = 1:1000
    %test data
    test_name = "test_"+j;
    x_start = squeeze(robot_start_point_test(j,:,:));
    x_goal = squeeze(robot_goal_point_test(j,:,:));
    actual_this_test = squeeze(actual_data_test(j,:,:,:));
    predicate_this_test = squeeze(predicate_data_test(j,:,:,:,:));
    if ~exist(".\test_data\"+ test_name +"\","dir")
        mkdir(".\test_data\"+ test_name +"\");
    end
    save(".\test_data\"+ test_name +"\actual_obs_traj.mat","actual_this_test")
    save(".\test_data\"+ test_name +"\predicate_obs_traj.mat","predicate_this_test")

    if ~exist(".\test_data\"+ test_name +"\"+ alpha_name +"\","dir")
        mkdir(".\test_data\"+ test_name +"\"+ alpha_name +"\");
    end

    for l = 1:3
        %different method
        method_name = method_list(l);
        if ~exist(".\test_data\"+ test_name +"\"+ alpha_name +"\"+ method_name +"\","dir")
            mkdir(".\test_data\"+ test_name +"\"+ alpha_name +"\"+ method_name +"\");
        end
        
        %Instantiate the car class
        car = Car_class(x_start', x_goal', predicate_this_test, actual_data_cali, predicate_data_cali, score_seque_cali, alpha, method_name);

        %begin simulation
        for i = 0:T-1
            if mod(i,10) == 0
                car.control();
            end
            car.step();
        end
        
        %get result
        robot_traj = car.get_traj();
        c_seque = car.get_c();
        pred_traj = car.get_pred_traj();
        cost_seque = car.get_cost();
        alpha_seque= car.get_alpha();
        time_seque = car.get_time();
       
        %save result
        save(".\test_data\"+ test_name +"\"+ alpha_name +"\"+ method_name +"\robot_traj.mat","robot_traj")
        save(".\test_data\"+ test_name +"\"+ alpha_name +"\"+ method_name +"\c_seque.mat","c_seque")
        save(".\test_data\"+ test_name +"\"+ alpha_name +"\"+ method_name +"\pred_traj.mat","pred_traj")
        save(".\test_data\"+ test_name +"\"+ alpha_name +"\"+ method_name +"\cost_seque.mat","cost_seque")
        save(".\test_data\"+ test_name +"\"+ alpha_name +"\"+ method_name +"\alpha_seque.mat","alpha_seque")
        save(".\test_data\"+ test_name +"\"+ alpha_name +"\"+ method_name +"\time_seque.mat","time_seque")
    end
end



