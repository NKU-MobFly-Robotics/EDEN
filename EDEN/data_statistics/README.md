# Data Writer

This bag enables writing the wanted data (and its corresponding timestamp) to the specific files. You can see how to use it in [eden.cpp](../Exploration/eden/src/eden.cpp)



## Important Functions
- void **StartTimer**(int dataid):  
Start to count how much time the program takes ultil call **EndTimer**(int dataid). The 'dataid' must be the same. 

- void **EndTimer**(int dataid):  
Stop counting and write the time cost to a file(**Computation/data_nameX** X=dataid). The 'dataid' must be the same, and **EndTimer**(int dataid) must be called before. 

- void **AddVolume**(double v, int id):  
Add v with the 'id-th' data. Write the accumulated data to the 'id-th' file(**Computation/vdata_nameX** X=id).

- void **SetVolume**(double v, int id):  
Write v to the 'id-th' file(**Computation/vdata_nameX** X=id).

## Parameters (in yaml)
**Computation/dir**: the path you want to write the data  
**Computation/data_num**: the number of data you want to write  
**Computation/data_nameX**(X=1, 2, ..., **Computation/data_num** - 1): the file name of the X-th data (use **EndTimer()** to write)  
**Computation/vdata_num**: the number of data you want to write  
**Computation/vdata_nameX**(X=1, 2, ..., **Computation/vdata_num** - 1): the file name of the X-th data (use **AddVolume/SetVolume()** to write)  

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
```