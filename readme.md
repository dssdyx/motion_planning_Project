the Project of motion planning , changed to 2d situation 

开始的路径搜索算法A*，在实践过程中，current node拓展时忽略被堵住的对角节点，或者对角节点旁边的节点有障碍，也会被忽略，注意得到的path，要包含头尾

simply算法是RDP，流程和作业注释的一样

使用mono poly closed form，构建mini jerk的轨迹，约束头尾 pva，中部p

safecheck采用迭代0.01s去判断，返回发生碰撞的第一个j segment

reopt不采用插入中点的方法，因为在simly过程中可能由于resolution设置的问题导致A星对边界的绕行节点被忽略，无论怎么插入中点都会陷入无限迭代，所以这里再次调用A星，并且在迭代过程中对resolution做迭代减小处理

遇到的问题：

没有速度和加速度约束，单纯的mono poly是满足不了这个条件的，所以要用bspline，其凸包性可以保证va在一定范围内

轨迹离obs太近，浪费很多free space，因为轨迹生成基于A星的节点，这个节点是贴着obs拓展的，所以这是无法避免的，要想解决这个问题，从硬性上需要对A星的节点进行拉离操作，具体做法应类似人工势场法，需要搞一个类似ESDF距离场的玩意，但维护这个场，很消耗资源，所以具体怎么做需要讨论一下，从软性上可以对traj加一个软约束项，但需要在A星节点被拉离的条件下操作，不然没意义

因为程序会在exec traj过程中进行replan，每次replan都导致uav状态突变，飞行不稳定，这个原因可能是时间分配没做好
