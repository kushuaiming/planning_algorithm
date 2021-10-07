# A* 算法
## 1. 伪代码
**输入**: 图数据结构<br>
**输出**: 从起点到终点的一条路径<br>
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

$O$ 是一个优先队列.<br>
$C$ 存储已经处理好的节点.<br>
Star($n$) 代表邻近n的所有节点.<br>
$c(n_1, n_2)$ $n_1$和$n_2$之间边的长度.<br>
$g(n)$ 从 n 到 $q_{start}$的路径总长度.<br>
$h(n)$是启发式代价函数, 返回从 $n$ 到 $q_{goal}$理论上的最短距离(两点之间的直线).<br>
$f(n) = g(n) + h(n)$ 从 $q_{start}$ 到 $q_{goal}$且通过$n$的最短距离估计.<br>
