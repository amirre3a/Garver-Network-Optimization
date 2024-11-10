# Garver-Network-Optimization

The primary objective of this study is to optimize the transmission network expansion for a 6-bus Garver network, focusing on balancing load demands while minimizing costs associated with line expansions. To address this, two distinct methods were employed:

1. Heuristic Methods in MATLAB:  
   I applied heuristic optimization techniques, specifically Genetic Algorithms (GA) and Particle Swarm Optimization (PSO), to explore potential solutions. These methods were chosen for their effectiveness in navigating complex, non-linear solution spaces. In MATLAB, the GA leveraged evolutionary principles such as crossover and mutation to simulate natural selection, while PSO utilized a collaborative swarm-based approach where solutions "learn" from the best-performing members of the population. Both methods aimed to identify cost-efficient configurations for expanding the network.

2. Linear Programming in Python:  
   Alongside the heuristic approach, a linear programming solution was developed in Python to establish a baseline. This method used a deterministic approach, setting linear constraints and objectives to address the expansion problem. Leveraging Python's extensive libraries, the linear programming solution sought to minimize total expansion costs and ensure load demands were met under specified network constraints. 

   To implement this solution:
   - _cvxpy_ was used for defining and solving the linear optimization problem.
   - _numpy_ and _pandas_ provided efficient data handling and manipulation capabilities.
   - _networkx_ facilitated modeling of the network's topology, enabling clear representation of bus connections and line capacities.
   - _matplotlib.pyplot_ allowed for visualizing the network configuration and the results.
   - _defaultdict_ from collections streamlined data structure management for network data, improving efficiency and readability.

This comprehensive setup in Python allowed for a detailed analysis and comparison with heuristic methods, offering insight into the differences in computational efficiency, solution quality, and the trade-offs between linear and heuristic approaches in transmission network expansion planning.

