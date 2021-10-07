# 路径规划算法
## 1. 如何使用
### 1.1 环境配置
Ubuntu20.04(建议) 或者 Ubunt18.04<br>
[安装 OpenCV](https://docs.opencv.org/4.5.3/d7/d9f/tutorial_linux_install.html)(建议安装OpenCV4)

### 1.2 编译代码
```
mkdir build
cd build
cmake ..
make -j10
./path_planning
```
### 1.3 补充说明
向右为X轴正方向, 向下为Y轴正方向

## 2. 自动驾驶技术中路径规划算法的分类
<table>
    <tr>
        <td>分类</td>
        <td width="200">算法</td>
        <td>说明</td>
    </tr>
    <tr>
        <td rowspan="3">Graph search based planners</td>
        <td>Dijkstra's Algorithm</td>
        <td>
            Known nodes/cells search space with associated weights<br/>
            Grid and node/cells weights computation according to the environment
        </td>
    </tr>
    <tr>
        <td>A* algorithm family</td>
        <td>
            Anytime D* with Voronoi cost functions Hybrid-heuristics A*<br/>
            A* with Voronoi/Lattice enviroment represeantation. PAO*
        </td>
    </tr>
    <tr>
        <td>State Lattices</td>
        <td>
            Enviroment decomposed in a local variable grid, depending on the complexity of the maneuver<br/>
            Spatio-temporal lattices(considering time adn velocity dimensions)
        </td>
    </tr>
    <tr>
        <td>Sampling based planners</td>
        <td>RRT</td>
        <td>
            Physical and logical bias are used to generate the random-tree<br/>
            Anytime planning with RRT*<br/>
            Trajectory coordination with RRT
        </td>
    </tr>
    <tr>
        <td rowspan="5">Interpolating curve planners</td>
        <td>Line ad circle</td>
        <td>Road fitting and interpolation of known waypoints</td>
    </tr>
    <tr>
        <td>Clothoid Curves</td>
        <td>
            Piecewise trajectory generation with straight, clothoid and circular segments<br/>
            Off-line generation of clothoid primitives from which the best will be taken in on-line evaluation
        </td>
    </tr>
    <tr>
        <td>Polynomial Curves</td>
        <td>
            Cubic order polynomial curves<br/>
            Higher order polynomial curves
        </td>
    </tr>
    <tr>
        <td>Bezier Curves</td>
        <td>
            Selection of the optimal control points location for the situation in hand<br/>
            Rational Bezier curves impletation
        </td>
    </tr>
    <tr>
        <td>Spline Curves</td>
        <td>Polynomial piecewise implementation Basis splines(b-splines)</td>
    </tr>
    <tr>
        <td>Numerical optimization approaches</td>
        <td>Function optimization</td>
        <td>
            Trajectory generation optimizing parameters such as speed, steering speed, rollover constraints, lateral accelerations, jerk(lateral comort optimization), among others
        </td>
    </tr>
</table>
