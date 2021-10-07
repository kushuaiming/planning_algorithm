# A* Algorithm
## 1. pseudocode
**Input**: A graph<br>
**Output**: A path between start and goal nodes<br>
**repeat**<br>
　Pick $n_{best}$ from $O$ such that $f(n_{best}) \leq f(n)$, $\forall n \in O$.<br>
　Remove $n_{best}$ from $O$ and add to C.<br>
　If $n_{best} =  q{goal}$, EXIT.<br>
　Expand $n_{best}$: for all $x \in Star(n{best})$ that are not in C.<br>
　**if** $x \notin O$ **then**<br>
　　add $x$ to $O$.<br>
　**else if** $g(n{best})$ + c(n{best}, x) < g(x)$ **then**<br>
　　update $x$'s backpointer to point to $n{best}$]<br>
　**end if**<br>
**util** $O$ is empty.<br>

$O$ is a priority queue.<br>
$C$ is a set of Node which has been processed.<br>
Star($n$) represents the set of nodes which are adjacent to n.<br>
$c(n_1, n_2)$ is the length of edge connecting $n_1$ and $n_2$.<br>
$g(n)$ is the total length of a back pointer path from n to $q_{start}$.<br>
$h(n)$ is the heuristic cost function, which returns the estimated cost of shortest path from $n$ to $q_{goal}$.<br>
$f(n) = g(n) + h(n)$ is the estimated cost of shortest path from $q_{start}$ to $q_{goal}$ via $n$.<br>
