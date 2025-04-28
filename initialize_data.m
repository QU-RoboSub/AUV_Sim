% Initialize Data - Read CSV and apply variables from vars.mat
function initialize_data()
    % File paths
    csv_file_path = 'thruster_data.csv';
    vars_file_path = 'vars.mat';
    
    % Read the CSV file containing the PWM-force-voltage relationship
    data = readmatrix(csv_file_path);
    
    % Extract the relevant columns
    voltage_values = unique(data(:, 1));  % Voltage is in column A (1st column)
    pwm_values = unique(data(:, 2));      % PWM is in column B (2nd column)
    force = data(:, 7);       % Force is in column G (7th column)

    % Initialize force_values with the correct size based on the input grids
    num_rows = length(voltage_values);   % Number of voltage grid rows
    num_cols = length(pwm_values);       % Number of PWM grid columns
    force_values = zeros(num_rows, num_cols);   % Initialize force_values matrix

    % Loop over each row of the voltage grid
    for i = 1:num_rows
        % Assign the appropriate segment from the force vector to the i-th row of force_values
        start_idx = (i-1) * num_cols + 1;    % Calculate the starting index for each row
        end_idx = i * num_cols;              % Calculate the ending index for each row
        force_values(i, :) = force(start_idx:end_idx);  % Assign the force values for this row
    end
    
    % Convert force from Kgf to Newtons (1 kgf = 9.81 N)
    force_values = force_values * 9.81;

    % Save the loaded data into workspace variables
    assignin('base', 'voltage_values', voltage_values);
    assignin('base', 'pwm_values', pwm_values);
    assignin('base', 'force_values', force_values);
    
    % Load additional variables from vars.mat (if exists)
    if isfile(vars_file_path)
        vars = load(vars_file_path);  % Load the .mat file
        % Apply variables from vars.mat to the workspace
        field_names = fieldnames(vars);
        for i = 1:numel(field_names)
            assignin('base', field_names{i}, vars.(field_names{i}));
        end
    else
        warning('vars.mat file not found at: %s', vars_file_path);
    end
    
    disp('Data initialization complete.');
end
