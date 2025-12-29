# Block Map Lite
The voxel map with low memory usage. It manages the voxels by reading and writing, only the voxels in the vicinity of the robot are maintained and queriable. This bag is the original version of [RipNeon](https://github.com/NKU-MobFly-Robotics/RipNeon).


<!-- ## Files
[eden_fsm.cpp](./src/eden_fsm.cpp): FSM, hold->exploration planning->traj excuting->exploration planning ....  
[eden.cpp](./src/eden.cpp): contains map updating, exploration target planning, trajectory planning, etc.  
[eden_node.cpp](./src/eden_node.cpp): the node.


## Important Functions
- void **EdenPlan**():  
Exploration planning + trajctory planning.

- bool **ViewPointsCheck**(double t):  
Check if current explored viewpoint is still good(have enough gain/not reached/blocked...).  

- bool **TrajCheck**():  
Check if current excuting trajectory is still feasible. -->
<!-- 

## Parameters (in yaml)
**Computation/dir**: the path you want to write the data  
**Computation/data_num**: the number of data you want to write  
**Computation/data_nameX**(X=1, 2, ..., **Computation/data_num** - 1): the file name of the X-th data (use **EndTimer()** to write)  
**Computation/vdata_num**: the number of data you want to write  
**Computation/vdata_nameX**(X=1, 2, ..., **Computation/data_num** - 1): the file name of the X-th data (use **AddVolume/SetVolume()** to write)  

For example: 
```
Computation/dir: /home/charliedog/data/ablation/pillars/proposed_vt_c

Computation/data_num: 7

Computation/data_name0: success_plan
Computation/data_name1: fail_plan
Computation/data_name2: target_plan
Computation/data_name3: traj_plan
Computation/data_name4: yaw_plan
Computation/data_name5: cover_plan
Computation/data_name6: corridor_plan

Computation/vdata_num: 4
Computation/vdata_name0: volume
Computation/vdata_name1: traj
Computation/vdata_name2: vel
Computation/vdata_name3: consistency
``` -->