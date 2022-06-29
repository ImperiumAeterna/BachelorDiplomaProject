# Half-Cooperative-Dijkstra-Pathfinding
Half-cooperative Dijkstra algorithm modification. Bachelor degree diploma project by Vladislav Kogan.

Pathfinding remains to be an interesting research topic closely related to the efficient allocation of agents’ resourses. The goal of pathfinding is usually to find the shortest path from one graph point to another. Many real-world applications can be viewed as pathfinding problems, such as motion planning, logistics, and decision making.

Many algorithms have been proposed to solve pathfinding-related problems. Solving a pathfinding planning problem is usually relatively straightforward with one agent, but is greatly complicated by the possibility of conflicts when many agents are present at the same time. Moreover, the speed of solving problems depends significantly on the scale and available processor resources.

The aim of this thesis is to implement, based on existing methods of solving route planning problems, a method for rational route planning for several agents, who trying to avoid mutual conflicts. The thesis analyzes existing algorithms for solving the problem, analyzes and evaluates existing tools for their implementation, and implements some of the algorithms. As a result of the thesis work, a transitive pathfinding algorithm for multiple agents, who trying to avoid mutual conflicts, is designed and implemented.

Modification realised by using grids with weights as an example. For this purpose some pieces of code by Amit Patel provided Apache 2.0 licence are used.

Half-cooperative Dijkstra provides multiple agents information to avoiding each other using weights changes according to each new agent's path. It's half-cooperative due to agents not provided about specific position of each other, just path in general.

Bachelor degree diploma work defense done 20th of June 2022. Result of defense is 95/100 points ("A" grade in ECTS grading scale).
