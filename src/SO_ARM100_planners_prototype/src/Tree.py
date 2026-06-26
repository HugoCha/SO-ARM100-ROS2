#! usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np
import random as rand

class Node:
    def __init__( self, q ):
        self.q = q
        self.parent = None

    def distance( self, node ):
        return np.linalg.norm( self.q - node.q )
    
    def __str__( self ):
        return "q: " + str( self.q )

    @classmethod
    def random( cls, size ):
        q = np.array( [rand.uniform( -np.pi, np.pi ) for _ in range( size )] ) 
        return Node( q )


class Tree:
    def __init__( self ):
        self.nodes = []

    def add( self, node ):
        parent_node = self.closest( node )
        new_node = Node( node.q )
        new_node.parent = parent_node

        self.nodes.append( new_node )
    
    def clear( self ):
        self.nodes.clear()

    def closest( self, node ):
        if ( len( self.nodes ) <= 0 ):
            return None

        parent_idx = 0
        min_dist = np.inf
        for idx in range( len( self.nodes ) ):
            n = self.nodes[idx]
            dist = n.distance( node )
            if ( dist < min_dist ):
                parent_idx = idx
                min_dist = dist

        return self.nodes[parent_idx]
    
    def plot(self):

        plt.figure()

        for node in self.nodes:

            if node.parent is not None:

                x = [node.parent.q[0], node.q[0]]
                y = [node.parent.q[1], node.q[1]]

                plt.plot(x, y, 'b-')

            plt.plot(node.q[0], node.q[1], 'ro')

        plt.xlabel("q0")
        plt.ylabel("q1")
        plt.axis('equal')
        plt.show()

def main():
    tree = Tree()

    for _ in range( 100 ):
        tree.add( Node.random( 2 ) )
    # n1 = Node( np.array([1,1]))
    # n2 = Node.random(2)
    # n3 = Node( np.array([-1,1]))
    # n4 = Node( np.array([0.5,1]))

    # tree.add( n1 )
    # tree.add( n2 )
    # tree.add( n3 )
    # tree.add( n4 )

    tree.plot()

if __name__ == "__main__":
    main()
