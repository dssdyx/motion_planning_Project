the Project of motion planning , changed to 2d situation 
开始的路径搜索算法A*，在实践过程中，current node拓展时忽略被堵住的对角节点，或者对角节点旁边的节点有障碍，也会被忽略，注意得到的path，要包含头尾
simply算法是RDP，流程和作业注释的一样
使用mono poly closed form，构建mini jerk的轨迹，约束头尾 pva，中部p
safecheck采用迭代0.01s去判断，返回发生碰撞的第一个j segment
reopt不采用插入中点的方法，因为在simly过程中可能由于resolution设置的问题导致A星对边界的绕行节点被忽略，无论怎么插入中点都会陷入无限迭代，所以这里再次调用A星，并且在迭代过程中对resolution做迭代减小处理
