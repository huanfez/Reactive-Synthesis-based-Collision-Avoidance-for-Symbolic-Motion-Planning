# Reactive-Synthesis-based-Collision-Avoidance-for-Symbolic-Motion-Planning
Symbolic motion planning for multi-robot collision avoidance problem. The scenarios is 4 agents move in a dynamic obstacle environment. Each agent take other agents as obstacles in multi-agent system collision avoidance. The collision was solved by decomposing concatenated inter collision situations into atmoic collision behaviors.

Setup:
1)You need to setup the environment variables in your computer for NuXMV
enter into the system and user environment variables interface, add the NuXMV include and bin

user Variables --> new 
variable: NUXMV_LIBRARY_PATH
value : "Your NuXMV directory"nuXmv-1.1.1-win64\share\nuXmv

system Variables -->path -->edit
add new by
"Your NuXMV directory"\nuXmv-1.1.1-win64\bin

Note: After this step, the NuXMV should not be moved or deleted

2) Modify the line 111 in the main file "ExAbsCollabTest_RT_v5.m" to be
filePath = 'Your own code directory'
