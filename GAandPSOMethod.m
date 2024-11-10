% Main script to choose between GA and PSO
clc;
clear;

% Prompt user to select the algorithm
choice = input('Enter 1 to run GA or 2 to run PSO: ');

switch choice
    case 1
       
        runGA();
    case 2
        
        runPSO();
    otherwise
        disp('Invalid choice. Please enter 1 or 2.');
end
function runGA()
    % Set the lower and upper bounds for the variables 
    lb = [0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1]; 
    ub = [2, 2, 2, 2, 2, 4, 2, 2, 1, 1, 1, 1, 1, 1]; 

    % Set the integer constraints for n_ij 
    intcon = 1:8; 

    % Set the options for the GA 
    options = optimoptions('ga', ... 
        'Display', 'off', ... 
        'PlotFcn', @gaplotbestf, ... 
        'PopulationSize', 200, ... 
        'MaxGenerations', 800, ... 
        'MutationFcn', {@mutationadaptfeasible}, ... 
        'CrossoverFraction', 0.8, ... 
        'FunctionTolerance', 1e-8, ... 
        'ConstraintTolerance', 1e-6, ... 
        'NonlinearConstraintAlgorithm', 'penalty', ... 
        'OutputFcn', @gaOutputFunction);  % Added OutputFcn

    % Run the GA 
    [x, fval] = ga(@objectiveFunction, 14, [], [], [], [], lb, ub, @nonlinearConstraints, intcon, options); 

    % Display the results 
    disp('Optimized Variables:'); 
    disp(x); 
    disp('Objective Function Value:'); 
    disp(fval); 

    % Calculate and display f_ij values 
    [f12, f14, f15, f23, f24, f26, f35, f46] = calculateFij(x); 
    disp('Actual flow values (not per unit):');
    disp(['f12 = ', num2str(f12)]); 
    disp(['f14 = ', num2str(f14)]); 
    disp(['f15 = ', num2str(f15)]); 
    disp(['f23 = ', num2str(f23)]); 
    disp(['f24 = ', num2str(f24)]); 
    disp(['f26 = ', num2str(f26)]); 
    disp(['f35 = ', num2str(f35)]); 
    disp(['f46 = ', num2str(f46)]); 

    % Calculate and display power generation at each bus
    gen = calculateGeneration([f12, f14, f15, f23, f24, f26, f35, f46]);
    disp('Power generation at each bus (in actual values):');
    disp(['Bus 1: ', num2str(gen(1)), ' MW']);
    disp(['Bus 2: ', num2str(gen(2)), ' MW']);
    disp(['Bus 3: ', num2str(gen(3)), ' MW']);
    disp(['Bus 4: ', num2str(gen(4)), ' MW']);
    disp(['Bus 5: ', num2str(gen(5)), ' MW']);
    disp(['Bus 6: ', num2str(gen(6)), ' MW']);

    % Display the number of lines between each bus pair
    disp('Number of lines between each bus pair:');
    disp(['Bus 1 to Bus 2: ', num2str(x(1))]);
    disp(['Bus 1 to Bus 4: ', num2str(x(2))]);
    disp(['Bus 1 to Bus 5: ', num2str(x(3))]);
    disp(['Bus 2 to Bus 3: ', num2str(x(4))]);
    disp(['Bus 2 to Bus 4: ', num2str(x(5))]);
    disp(['Bus 2 to Bus 6: ', num2str(x(6))]);
    disp(['Bus 3 to Bus 5: ', num2str(x(7))]);
    disp(['Bus 4 to Bus 6: ', num2str(x(8))]);

    % Plot the network
    plotNetwork(x);
end

function [state, options, optchanged] = gaOutputFunction(options, state, flag)
    optchanged = false;
    if strcmp(flag, 'done')
        [c, ceq] = nonlinearConstraints(state.Best(end,:));
        if all(c <= 0) && all(abs(ceq) <= options.ConstraintTolerance)
            disp('Constraints are satisfied.');
        end
    end
end


%% ===============================================================PSO=======================================================


% Function to run the Particle Swarm Optimization (PSO)
function runPSO()
    % Set the lower and upper bounds for the variables 
    lb = [0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1]; 
    ub = [2, 2, 2, 2, 2, 4, 2, 2, 1, 1, 1, 1, 1, 1]; 

    % Set the options for the PSO 
    options = optimoptions('particleswarm', ... 
        'Display', 'iter', ... 
        'PlotFcn', @pswplotbestf, ... 
        'SwarmSize', 100, ... 
        'MaxIterations', 1100, ... 
        'FunctionTolerance', 1e-8);

    % Run the PSO 
    nvars = 14;
    [x, fval] = particleswarm(@penalizedObjectiveFunction, nvars, lb, ub, options); 

    % Round the integer variables (number of lines) to the nearest integer
    x(1:8) = round(x(1:8));

    % Display the results 
    disp('Optimized Variables:'); 
    disp(x); 
    disp('Objective Function Value:'); 
    disp(fval); 

    % Calculate and display f_ij values 
    [f12, f14, f15, f23, f24, f26, f35, f46] = calculateFij(x); 
    disp('Actual flow values (not per unit):');
    disp(['f12 = ', num2str(f12)]); 
    disp(['f14 = ', num2str(f14)]); 
    disp(['f15 = ', num2str(f15)]); 
    disp(['f23 = ', num2str(f23)]); 
    disp(['f24 = ', num2str(f24)]); 
    disp(['f26 = ', num2str(f26)]); 
    disp(['f35 = ', num2str(f35)]); 
    disp(['f46 = ', num2str(f46)]); 

    % Calculate and display power generation at each bus
    gen = calculateGeneration([f12, f14, f15, f23, f24, f26, f35, f46]);
    disp('Power generation at each bus (in actual values):');
    disp(['Bus 1: ', num2str(gen(1)), ' MW']);
    disp(['Bus 2: ', num2str(gen(2)), ' MW']);
    disp(['Bus 3: ', num2str(gen(3)), ' MW']);
    disp(['Bus 4: ', num2str(gen(4)), ' MW']);
    disp(['Bus 5: ', num2str(gen(5)), ' MW']);
    disp(['Bus 6: ', num2str(gen(6)), ' MW']);

    % Display the number of lines between each bus pair
    disp('Number of lines between each bus pair:');
    disp(['Bus 1 to Bus 2: ', num2str(x(1))]);
    disp(['Bus 1 to Bus 4: ', num2str(x(2))]);
    disp(['Bus 1 to Bus 5: ', num2str(x(3))]);
    disp(['Bus 2 to Bus 3: ', num2str(x(4))]);
    disp(['Bus 2 to Bus 4: ', num2str(x(5))]);
    disp(['Bus 2 to Bus 6: ', num2str(x(6))]);
    disp(['Bus 3 to Bus 5: ', num2str(x(7))]);
    disp(['Bus 4 to Bus 6: ', num2str(x(8))]);

    % Plot the network
    plotNetwork(x);
end


% Define the penalized objective function 
function v = penalizedObjectiveFunction(x) 
    % Objective function value
    v = objectiveFunction(x); 
    
    % Nonlinear constraints
    [c, ceq] = nonlinearConstraints(x); 
    
    % Penalty for violating constraints
    penalty = 1e6; % Large penalty value
    v = v + penalty * sum(max(c, 0)) + penalty * sum(ceq.^2);
end


%%---------------------

% Define the objective function 
function v = objectiveFunction(x) 
    % Extract n_ij from the input vector x 
    n12 = x(1); 
    n14 = x(2); 
    n15 = x(3); 
    n23 = x(4); 
    n24 = x(5); 
    n26 = x(6); 
    n35 = x(7); 
    n46 = x(8); 
    
    % Calculate the objective function value 
    v = 40*n12 + 60*n14 + 20*n15 + 20*n23 + 40*n24 + 30*n26 + 20*n35 + 30*n46; 
end 

% Define the nonlinear constraints 
function [c, ceq] = nonlinearConstraints(x) 
    % Extract n_ij and theta_i from the input vector x 
    n12 = x(1); 
    n14 = x(2); 
    n15 = x(3); 
    n23 = x(4); 
    n24 = x(5); 
    n26 = x(6); 
    n35 = x(7); 
    n46 = x(8); 
    theta1 = x(9); 
    theta2 = x(10); 
    theta3 = x(11); 
    theta4 = x(12); 
    theta5 = x(13); 
    theta6 = x(14); 

    % Equations 
    eq1 = -2.5 * (1 + n12) * (theta1 - theta2) - (5/3) * (1 + n14) * (theta1 - theta4) - 5 * (1 + n15) * (theta1 - theta5) + 0.30; 
    eq2 = 2.5 * (1 + n12) * (theta1 - theta2) - 5 * (1 + n23) * (theta2 - theta3) - 2.5 * (1 + n24) * (theta2 - theta4) - (10/3) * n26 * (theta2 - theta6) + 2.40; 
    eq3 = 5 * (1 + n23) * (theta2 - theta3) - 5 * (1 + n35) * (theta3 - theta5) - 1.25; 
    eq4 = (5/3) * (1 + n14) * (theta1 - theta4) + (5/2) * (1 + n24) * (theta2 - theta4) - (10/3) * n46 * (theta4 - theta6) + 1.60; 
    eq5 = 5 * (1 + n15) * (theta1 - theta5) + 5 * (1 + n35) * (theta3 - theta5) + 2.40; 
    eq6 = (10/3) * n26 * (theta2 - theta6) + (10/3) * n46 * (theta4 - theta6) - 5.45; 

    % Constraints 
    epsilon = 1e-6; % Tolerance for equality constraints 
    c1 = abs(2.5 * (1 + n12) * (theta1 - theta2)) - (1 + n12); 
    c2 = abs((5/3) * (1 + n14) * (theta1 - theta4)) - 0.8 * (1 + n14); 
    c3 = abs(5 * (1 + n15) * (theta1 - theta5)) - (1 + n15); 
    c4 = abs(5 * (1 + n23) * (theta2 - theta3)) - (1 + n23); 
    c5 = abs(2.5 * (1 + n24) * (theta2 - theta4)) - (1 + n24); 
    c6 = abs((10/3) * n26 * (theta2 - theta6)) - n26; 
    c7 = abs(5 * (1 + n35) * (theta3 - theta5)) - (1 + n35); 
    c8 = abs((10/3) * n46 * (theta4 - theta6)) - n46; 

    % Combine all constraints 
    c = [c1; c2; c3; c4; c5; c6; c7; c8; ... 
         eq1 - epsilon; -eq1 - epsilon; ... 
         eq2 - epsilon; -eq2 - epsilon; ... 
         eq3 - epsilon; -eq3 - epsilon; ... 
         eq4 - epsilon; -eq4 - epsilon; ... 
         eq5 - epsilon; -eq5 - epsilon; ... 
         eq6 - epsilon; -eq6 - epsilon]; 
    ceq = []; 
end 

% Function to calculate f_ij values 
function [f12, f14, f15, f23, f24, f26, f35, f46] = calculateFij(x) 
    n12 = x(1); 
    n14 = x(2); 
    n15 = x(3); 
    n23 = x(4); 
    n24 = x(5); 
    n26 = x(6); 
    n35 = x(7); 
    n46 = x(8); 
    theta1 = x(9); 
    theta2 = x(10); 
    theta3 = x(11); 
    theta4 = x(12); 
    theta5 = x(13); 
    theta6 = x(14); 

    % Base values for conversion from per unit to actual values
    base_power = 100;

    % Calculate f_ij values in per unit
    f12_pu = 2.5 * (1 + n12) * (theta1 - theta2);
    f14_pu = (5/3) * (1 + n14) * (theta1 - theta4);
    f15_pu = 5 * (1 + n15) * (theta1 - theta5);
    f23_pu = 5 * (1 + n23) * (theta2 - theta3);
    f24_pu = 2.5 * (1 + n24) * (theta2 - theta4);
    f26_pu = (10/3) * n26 * (theta2 - theta6);
    f35_pu = 5 * (1 + n35) * (theta3 - theta5);
    f46_pu = (10/3) * n46 * (theta4 - theta6);

    % Convert f_ij values from per unit to actual values
    f12 = f12_pu * base_power;
    f14 = f14_pu * base_power;
    f15 = f15_pu * base_power;
    f23 = f23_pu * base_power;
    f24 = f24_pu * base_power;
    f26 = f26_pu * base_power;
    f35 = f35_pu * base_power;
    f46 = f46_pu * base_power;
end

% Function to calculate power generation at each bus
function gen = calculateGeneration(flows)
    % Extract flow values
    f12 = flows(1);
    f14 = flows(2);
    f15 = flows(3);
    f23 = flows(4);
%     f24 = flows(5);
    f26 = flows(6);
    f35 = flows(7);
    f46 = flows(8);
    
    % Initialize generation array
    gen = zeros(1, 6);
    
    % Calculate power generation at each bus
    % Buses 2, 4, and 5 have zero generation
    gen(1) = f12 + f14 + f15;
    gen(2) = 0; % Power generation at Bus 2 is zero
    gen(3) = -f23 + f35;
    gen(4) = 0; % Power generation at Bus 4 is zero
    gen(5) = 0; % Power generation at Bus 5 is zero
    gen(6) = -f26 - f46;
end

% Function to plot the network
function plotNetwork(x)
    % Extract n_ij values
    n12 = x(1); 
    n14 = x(2); 
    n15 = x(3); 
    n23 = x(4); 
    n24 = x(5); 
    n26 = x(6); 
    n35 = x(7); 
    n46 = x(8); 

    % Define the bus positions
    busPos = [0 1; 1 2; 1 0; 2 2; 2 0; 3 1];

    % Plot the buses
    figure;
    hold on;
    plot(busPos(:,1), busPos(:,2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    text(busPos(:,1), busPos(:,2), {'Bus 1','Bus 2','Bus 3','Bus 4','Bus 5','Bus 6'}, 'VerticalAlignment','bottom', 'HorizontalAlignment','right');

    % Plot the connections
    connections = [1 2; 1 4; 1 5; 2 3; 2 4; 2 6; 3 5; 4 6];
    n_values = [n12, n14, n15, n23, n24, n26, n35, n46];
    
    for i = 1:size(connections, 1)
        if n_values(i) > 0
            plot([busPos(connections(i,1),1), busPos(connections(i,2),1)], ...
                 [busPos(connections(i,1),2), busPos(connections(i,2),2)], 'b-', 'LineWidth', 2);
        end
    end

    % Enhance plot
    xlabel('X-coordinate');
    ylabel('Y-coordinate');
    title('Bus Network');
    axis equal;
    grid on;
    hold off;
end
