cost_closed_seque_vanilla = zeros(1,1000);
cost_closed_seque_interative = zeros(1,1000);
cost_closed_seque_vanilla_poster = zeros(1,1000);

time_seque_vanilla = zeros(1,1000);
time_seque_interative = zeros(1,1000);
time_seque_vanilla_poster = zeros(1,1000);

alpha_name = "alpha_05";
for i = 1:1000
    i
    test_name = "test_"+i;
    %cost
    cost_seque_vanilla = importdata(".\test_data\"+ test_name +"\"+ alpha_name +"\"+ "Scp" +"\cost_seque.mat");
    cost_seque_interative = importdata(".\test_data\"+ test_name +"\"+ alpha_name +"\"+ "FbCpIra" +"\cost_seque.mat");
    cost_seque_vanilla_poster = importdata(".\test_data\"+ test_name +"\"+ alpha_name +"\"+ "FbCpAra" +"\cost_seque.mat");
    
    cost_closed_vanilla = trace(cost_seque_vanilla);
    cost_closed_interative = trace(cost_seque_interative);
    cost_closed_vanilla_poster = trace(cost_seque_vanilla_poster);

    cost_closed_seque_vanilla(1,i) = cost_closed_vanilla;
    cost_closed_seque_interative(1,i) = cost_closed_interative;
    cost_closed_seque_vanilla_poster(1,i) = cost_closed_vanilla_poster;
   
end

% average cost
sum_cost_vanilla = sum(cost_closed_seque_vanilla)/1000;
sum_cost_interative = sum(cost_closed_seque_interative)/1000;
sum_cost_vanilla_poster = sum(cost_closed_seque_vanilla_poster)/1000;


