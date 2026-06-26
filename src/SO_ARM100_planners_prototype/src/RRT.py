#! usr/bin/python3

from abc import ABC, abstractmethod

from .MotionValidator import MotionValidator
from .Tree import Node, Tree

class RRTParameters:
    def __init__( self, max_iterations = 100, bias_factor = 0.2, tolerance = 1e-4 ):
        self.max_iterations = max_iterations
        self.bias_factor = bias_factor
        self.tolerance = tolerance

class IMotionPlanner(ABC):
    @abstractmethod
    def plan( self, q_init, q_final ):
        pass

class DataStructureMotionPlanner(IMotionPlanner):
    def __init__( self, validator ):
        self.validator = validator

    def plan( self, q_init, q_final ):
        self.build( q_init, q_final )
        self.generate()

    @abstractmethod
    def build( self, q_init, q_final ) -> bool:
        pass

    @abstractmethod
    def generate( self ) -> None:
        pass

class RRT(DataStructureMotionPlanner):
    def __init__( self, parameters, validator ):
        self.parameters = parameters
        self.validator = validator
        self.tree = Tree()
        self.closest_to_final_node = None
        self.path = []

    def build( self, q_init, q_final ):
        self.tree.clear()
        self.solution = None

        initial_node = Node( q_init ) 
        final_node = Node( q_final )

        self.tree.add( initial_node )

        for _ in range( self.parameters.max_iterations ):
            rand_node = Node.random( len( q_init ) )
            bias_node = self.bias_to_final_node( rand_node, final_node )
            nearest_node = self.tree.closest( bias_node )

            if ( nearest_node is None or self.validator.is_valid( nearest_node.q, bias_node.q ) ):
                self.tree.add( bias_node )

                if ( final_node.distance( bias_node ) < self.parameters.tolerance ):
                    self.closest_to_final_node = bias_node
                    return True
            
        return False
    
    def generate( self ) -> None:
        self.path.clear()

        if ( self.closest_to_final_node is None ):
            return
        
        self.path.append( self.closest_to_final_node.q )
        
        while self.closest_to_final_node.parent is not None:
            self.path.append( self.closest_to_final_node.parent.q )

        self.path.reverse()

    def bias_to_final_node( self, rand_node, final_node ):
        direction = final_node.q - rand_node.q
        bias_q = rand_node + self.parameters.bias_factor * direction
        return Node( bias_q )
    