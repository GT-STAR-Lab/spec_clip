function [result] = rollout(matrix, start_state, timestep)
    states = length(start_state);
    result = zeros(states, timestep + 1);
    result(:, 1) = start_state;
    
    state = start_state;
    for i = 1:timestep
        new_state = matrix * state;
        result(:, i + 1) = new_state;
        state = new_state;
    end
end

