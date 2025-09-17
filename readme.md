## LosslessConvex无损凸化
### 2025/9/17
复现了祖师爷Acikmese这篇论文的第一个实验，文章来源：[Convex Programming Approach to Powered Descent Guidance for Mars Landing](https://arc.aiaa.org/doi/10.2514/1.27553)
- 能实现：对发动机深度节流的非凸性进行无损凸化
- 不足：未引入滑翔角约束和高度>0的约束
### 2025/9/18
Todolist:
- 修改results存储变量（速度，位置）减少不必要的计算
- 增加一个findbest函数，将线性搜索代码分离出来