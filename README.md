# Reactive-Synthesis-based-Collision-Avoidance-for-Symbolic-Motion-Planning
Symbolic motion planning for multi-agent collision avoidance problem. The scenario is 4 agents move in a dynamic-obstacle environment. Each agent takes the other agents as obstacles in multi-agent system collision avoidance. The collision was solved by decomposing concatenated inter collision situations into atmoic collision behaviors.

Setup:
Add the environment variables for NuXMV,
1.Enter into the system and user environment variables interface, add the NuXMV include and bin

user Variables --> new 
variable: NUXMV_LIBRARY_PATH
value : "NuXMV directory"nuXmv-1.1.1-win64\share\nuXmv

system Variables -->path -->edit
add new by
"NuXMV directory"\nuXmv-1.1.1-win64\bin
