def compute_g(algorithm, node, goal_state):
    """
        Evaluates the g() value.
        
        Parameters
        ===========
            algorithm: str
                The algorithm type based on which the g-value will be computed.
            node: Node
                The node whose g-value is to be computed.
            goal_state: State
                The goal state for the problem.
                
        Returns
        ========
            int
                The g-value for the node.
    """
    
    if algorithm == "bfs":
    
        return NotImplementedError
    elif algorithm == "ucs":
    
        raise NotImplementedError
    elif algorithm == "gbfs":
    
        raise NotImplementedError
    elif algorithm == "astar":
    
        raise NotImplementedError
    elif algorithm == "custom-astar":
    
        # Implement the custom heuristic here.
        raise NotImplementedError
        
    # Should never reach here.
    assert False
    return float("inf")
    
def compute_h(algorithm, node, goal_state):
    """
        Evaluates the h() value.
        
        Parameters
        ===========
            algorithm: str
                The algorithm type based on which the h-value will be computed.
            node: Node
                The node whose h-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The h-value for the node.
    """
    
    if algorithm == "bfs":
    
        return 0
    elif algorithm == "ucs":
    
        raise NotImplementedError
    elif algorithm == "gbfs":
    
        raise NotImplementedError
    elif algorithm == "astar":
    
        raise NotImplementedError
    elif algorithm == "custom-astar":
    
        # Implement the custom heuristic here.
        raise NotImplementedError
        
    # Should never reach here.
    assert False
    return float("inf")
