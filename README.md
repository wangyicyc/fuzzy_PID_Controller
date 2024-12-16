# Summary
This is a fuzzy PID controller implementation in Cpp
## Description
包含以下文件：
- rules.h: 包含模糊规则的定义
- fuzzy_pid.h: 包含模糊PID控制器的类定义
- fuzzy_pid.cpp: fuzzy_pid.h的实现
- CMakeLists.txt: 编译脚本
其中，==rules.h==的创建是为了更好的分类，便于读者集中注意力于控制器的实现
其次，==CMakeLists.txt==文件用于编译项目，当该项目是在ros的功能包中使用，因此若直接下载无法直接编译

## indroduction about fuzzy PID controller
本人深感网络上关于模糊PID控制器的资料较少，因此在此对模糊PID控制器的原理及其实现进行简要介绍。
概言之模糊PID可由如此形容：
input->fuzzy->get the membership->rules->output
其中，input为输入，fuzzy为模糊处理，membership为模糊集，rules为规则，output为输出。
- 首先，input包含误差error和误差变化率error_dot。
- 然后，fuzzy处理input，将其映射到模糊集membership中。
  - 简单来说，input可以同时属于多个模糊集(如PB、PM，可以同时隶属于几个模糊集可自行设定)
  - fuzzy处理就是将input映射到membership中，计算出input隶属于各个模糊集的程度
- 接着，rules根据membership计算输出，也称为defuzzy处理。
  - 简单来说，rules根据membership计算出输出。
- 最后，output为PID控制器的delta(增量)输出。