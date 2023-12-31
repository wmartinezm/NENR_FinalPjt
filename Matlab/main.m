% SNR Data
flexor_data = [30.28, 44.32, 33.02];
extensor_data = [30.63, 34.2, 37.2];

% Boxplot
figure;
boxplot([flexor_data', extensor_data'], 'Labels', {'Flexor', 'Extensor'});
title('SNR for Recorded Electrodes');
ylabel('SNR');

% Statistical Test (e.g., t-test)
[~, p_value, ~, stats] = ttest2(flexor_data, extensor_data);
text(1.5, max([flexor_data, extensor_data]) + 1, ['p-value = ', num2str(p_value)], 'HorizontalAlignment', 'center');

% Indicate if there is a statistical difference
if p_value < 0.05
    text(1.5, max([flexor_data, extensor_data]) + 2, 'Statistically Different', 'HorizontalAlignment', 'center', 'Color', 'r');
end
%% Statistical Analysis
% Data for Time-to-Completion 
% Consolidated data for Time to Completion
time_data = [
    % Participant 1
    7.84, 5.43, 4.9,   % Level 1
    5.44, 11.92, 9.5,  % Level 2
    12.86, 10.99, 11.36; % Level 3
    
    % Participant 2
    6.25, 9.46, 6.6,   % Level 1
    8.29, 8.01, 13.49, % Level 2
    15.17, 11.89, 18.04; % Level 3
    
    % Participant 3
    4.4, 5.44, 10.48,  % Level 1
    7.02, 6.84, 7.5,   % Level 2
    10.61, 13.13, 11.9 % Level 3
];

% Create boxplot
figure;
boxplot(time_data, 'Labels', {'Level 1', 'Level 2', 'Level 3'});
title('Time to Completion vs. Levels');
ylabel('Time (s)');

% Statistical Tests
p_values_time_levels = zeros(2, 3);

for i = 1:3
    % Compare Level i with Level i+1
    [~, p_values_time_levels(1, i), ~, ~] = ttest2(time_data(i,:), time_data(i+3, :));
    
    % Compare Level i with Level i+2
    [~, p_values_time_levels(2, i), ~, ~] = ttest2(time_data(i,:), time_data(i+6,:));
    
end

% Display p-values
disp('P-values for Time to Completion between Levels:');
disp(p_values_time_levels);

% Display statistical difference indicators
for i = 1:3
    if p_values_time_levels(1, i) < 0.05
        text(i, max(time_data(:)) + 1, '*', 'HorizontalAlignment', 'center', 'Color', 'r');
    end
    
    if p_values_time_levels(2, i) < 0.05
        text(i, max(time_data(:)) + 2, '**', 'HorizontalAlignment', 'center', 'Color', 'r');
    end
end


%% Statistical Analysis
% Number of moves V1
% Consolidated data for Number of Moves
moves_data = [
    % Participant 1
    8, 8, 8; % Level 1
    8, 12, 12; % Level 2
    16, 14, 14; % Level 3
    
    % Participant 2
    8, 10, 8; % Level 1
    9, 10, 10; % Level 2
    15, 13, 17; % Level 3
    
    % Participant 3
    8, 8, 14; % Level 1
    10, 9, 9; % Level 2
    15, 18, 19; % Level 3
];

% Create boxplot for Number of Moves
figure;
boxplot(moves_data, 'Labels', {'Level 1', 'Level 2', 'Level 3'});
title('Number of Moves vs. Levels');
ylabel('Number of Moves');

% Statistical Tests
p_values_moves_levels = zeros(2, 3);

for i = 1:3
    % Compare Level i with Level i+1
    [~, p_values_time_levels(1, i), ~, ~] = ttest2(time_data(i,:), time_data(i+3, :));
    
    % Compare Level i with Level i+2
    [~, p_values_time_levels(2, i), ~, ~] = ttest2(time_data(i,:), time_data(i+6,:));    
end

% Display p-values
disp('P-values for Number of Moves between Levels:');
disp(p_values_moves_levels);

% Display statistical difference indicators
for i = 1:3
    if p_values_moves_levels(1, i) < 0.05
        text(i, max(moves_data(:)) + 1, '*', 'HorizontalAlignment', 'center', 'Color', 'r');
    end
    
    if p_values_moves_levels(2, i) < 0.05
        text(i, max(moves_data(:)) + 2, '**', 'HorizontalAlignment', 'center', 'Color', 'r');
    end
end

%% Statistical Analysis
% Number of moves V2
% Consolidated data for Number of Moves
moves_data = [
    % Participant 1
    8, 8, 8,   % Level 1
    8, 12, 12,  % Level 2
    16, 14, 14; % Level 3
    
    % Participant 2
    8, 10, 8,   % Level 1
    9, 10, 10,  % Level 2
    15, 13, 17; % Level 3
    
    % Participant 3
    8, 8, 14,   % Level 1
    10, 9, 9,   % Level 2
    15, 18, 19; % Level 3
];

% Minimum moves per level
min_moves = [8, 8, 13];

% Create boxplot
figure;
% Create boxplot for Level 1
subplot(1, 3, 1);
boxplot([moves_data([1,4,6], :)', min_moves(1)*ones(3, 1)], 'Labels', {'P1', 'P2', 'P3', 'Min'});
title('Number of Moves vs. Level 1');
ylabel('Number of Moves');
hold on;
% Similar modifications for and Level 3
subplot(1, 3, 2);
boxplot([moves_data([2,5,8], :)', min_moves(2)*ones(3, 1)], 'Labels', {'P1', 'P2', 'P3', 'Min'});
title('Number of Moves vs. Level 2');
ylabel('Number of Moves');
hold on;
% Similar modifications for Level 3
subplot(1, 3, 3);
boxplot([moves_data([3,6,9], :)', min_moves(3)*ones(3, 1)], 'Labels', {'P1', 'P2', 'P3', 'Min'});
title('Number of Moves vs. Level 3');
ylabel('Number of Moves');


% Statistical Tests and Indicators
p_values_moves_levels = zeros(3, 3);

for i = 1:3
    [~, p_values_moves_levels(i, 1), ~, ~] = ttest2(moves_data(:, i), min_moves(i));

    % Compare Level i with Level i+1
    [~, p_values_time_levels(1, i), ~, ~] = ttest2(time_data(i,:), time_data(i+3, :));
    
    % Compare Level i with Level i+2
    [~, p_values_time_levels(2, i), ~, ~] = ttest2(time_data(i,:), time_data(i+6,:));    
end

% Display p-values
disp('P-values for Number of Moves between Levels and Minimum Moves:');
disp(p_values_moves_levels);

% Display statistical difference indicators
for i = 1:3
    if p_values_moves_levels(i, 1) < 0.05
        text(i, max(moves_data(:)) + 1, '*', 'HorizontalAlignment', 'center', 'Color', 'r');
    end
end
