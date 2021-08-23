class PriorityQueue:
    """
        A class that implements the Priority Queue for search algorithms.
        
        This priority queue is a min-priority queue.
    """
    

    def __init__(self):
        """
            Initializes this priority queue.
        """
        
        raise NotImplementedError

    def push(self, priority, node):
        """
            Pushes a node into the priority queue.
            
            Parameters
            ===========
                priority: int
                    The priority of the entry.
                node: Node
                    The node to be pushed.
        """
        
        raise NotImplementedError

    def is_empty(self):
        """
            Returns
            ========
                bool
                    True if the priority queue is empty, False otherwise.
                    
            Note
            =====
                It is possible that the priority queue contains elements but
                is empty. In this case, all elements in the priority queue are
                invalid elements.
        """
        
        raise NotImplementedError

    def pop(self):
        """
            Returns
            ========
                Node
                    The node with the minimum priority value.
            
            Raises
            =======
                IndexError
                    If the priority queue was empty or did not have any valid
                    elements.
        """

        raise NotImplementedError
        
    def contains(self, state):
        """
            Parameters
            ===========
                state: State
                    The state to be checked.
                    
            Returns
            ========
                bool
                    True if the state is present in the priority queue, False
                    otherwise.
        """
        
        raise NotImplementedError
