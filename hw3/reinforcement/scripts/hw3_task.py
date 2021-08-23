def get_q_value(alpha, gamma, reward, q_s_a, q_s_dash_a_dash):

    raise NotImplementedError
    
def get_epsilon(current_epsilon, episode, epsilon_task):

    if epsilon_task == 1:

        return max(0.05, 1.0 - 0.05 * episode)
    elif epsilon_task == 2:
   
        raise NotImplementedError
    else:
    
        # Should not be here.
        assert False
    
def alpha(current_alpha, episode, step):

    return current_alpha
    
def pruned_actions(location, action_list, pick_locations, place_locations):

    raise NotImplementedError
