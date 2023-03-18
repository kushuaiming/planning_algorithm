# 路径规划算法
## 1. 如何使用
### 1.1 环境配置
[Ubuntu20.04](https://ubuntu.com/)(建议) 或者 Ubunt18.04<br>
[CMake](https://cmake.org/)<br>
[OpenCV](https://docs.opencv.org/4.5.3/d7/d9f/tutorial_linux_install.html)(建议安装OpenCV4)

### 1.2 编译代码
```
mkdir build
cd build
cmake ..
make -j10
./a_star
./rrt
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
        <td rowspan="3">基于图搜索的算法</td>
        <td>Dijkstra算法</td>
        <td>
            已知带有权重的节点搜索空间<br/>
            节点的权重和周围的环境有关
        </td>
    </tr>
    <tr>
        <td>A*算法</td>
        <td>
            A*在Dijkstra算法的基础上增加了启发式代价函数<br/>
        </td>
    </tr>
    <tr>
        <td>Lattices</td>
        <td>
            根据环境的复杂性不同, 将其建模到本地的离散化网格中<br/>
            时间-空间lattice, 考虑时间和速度两个维度
        </td>
    </tr>
    <tr>
        <td>基于取样的算法</td>
        <td>RRT</td>
        <td>
            用物理和逻辑上的偏差来生成随即搜索树<br/>
        </td>
    </tr>
    <tr>
        <td rowspan="5">曲线插值</td>
        <td>直线和圆</td>
        <td>利用已知的点进行插值形成路径</td>
    </tr>
    <tr>
        <td>回旋线</td>
        <td>
            利用直线、螺旋曲线、圆弧等分段组成轨迹<br/>
            离线生成原始曲线然后放到线上进行评估
        </td>
    </tr>
    <tr>
        <td>多项式曲线</td>
        <td>
            三阶多项式曲线<br/>
        </td>
    </tr>
    <tr>
        <td>贝塞尔曲线</td>
        <td>
            为现有的情况选择最佳的控制点<br/>
            有理贝塞尔曲线的实现
        </td>
    </tr>
    <tr>
        <td>样条曲线</td>
        <td>分段的多项式函数组成样条曲线</td>
    </tr>
    <tr>
        <td>数值优化</td>
        <td>函数优化</td>
        <td>
            优化生成的轨迹的参数(比如速度、转向速度、滚动约束、侧向加速度、Jerk(速度的导数, 表示舒适性)等等)<br/>
        </td>
    </tr>
</table>
