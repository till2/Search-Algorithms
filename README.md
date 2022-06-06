# Search-Algos-Visual
## _Dynamically generated visualizations of Graph-Search Algorithms._

## How to use?
The program only needs the Graphs __Edges__. Nodes are implicitly generated.
Further __Edge-Costs__ and __Heuristics__ to the goal can be specified.

## Sample output image

<img src="https://github.com/till2/Search-Algos-Visual/blob/main/assets/a_star.png?raw=true" width="600" height="400"/>

## Text output
Here the __A_star__ algorithm was run with __start = A__ and __goal = G__.

```
> python3.9 a_star.py 

start: A
goal: G

current:    A
frontier:  [('B', 'heur:3', 'path:1', 'total:4'), ('C', 'heur:4', 'path:6', 'total:10')] 

current:    B
frontier:  [('C', 'heur:4', 'path:6', 'total:10'), ('C', 'heur:4', 'path:3', 'total:7'), ('E', 'heur:2', 'path:5', 'total:7')] 

current:    C
frontier:  [('C', 'heur:4', 'path:6', 'total:10'), ('E', 'heur:2', 'path:5', 'total:7'), ('D', 'heur:1', 'path:7', 'total:8'), ('E', 'heur:2', 'path:4', 'total:6'), ('G', 'heur:0', 'path:9', 'total:9')] 

current:    E
frontier:  [('C', 'heur:4', 'path:6', 'total:10'), ('E', 'heur:2', 'path:5', 'total:7'), ('D', 'heur:1', 'path:7', 'total:8'), ('G', 'heur:0', 'path:9', 'total:9'), ('F', 'heur:5', 'path:5', 'total:10')] 

current:    E
frontier:  [('C', 'heur:4', 'path:6', 'total:10'), ('D', 'heur:1', 'path:7', 'total:8'), ('G', 'heur:0', 'path:9', 'total:9'), ('F', 'heur:5', 'path:5', 'total:10'), ('F', 'heur:5', 'path:6', 'total:11')] 

current:    D
frontier:  [('C', 'heur:4', 'path:6', 'total:10'), ('G', 'heur:0', 'path:9', 'total:9'), ('F', 'heur:5', 'path:5', 'total:10'), ('F', 'heur:5', 'path:6', 'total:11'), ('C', 'heur:4', 'path:10', 'total:14'), ('G', 'heur:0', 'path:8', 'total:8')] 

goal found: G
Goal path: ['A', 'B', 'C', 'D', 'G']
Cost: 8
```
