# OMPRRT: Open-MP based rapidly exploring random trees 
### *(pronounced Ompert. Like Robert with Omp.)*

## cmd for execution
```
./main -f inputs/map_1.txt -n 4 -r 0 -d 5
```
-f for input file
-n for thread_num
-r for RTT or RTT* (0 and 1(default))
-d for distance for every step


## cmd for visualization
```
python visualize.py -i inputs/map_3.txt -n outputs/map_3_4_nodes.txt -p outputs/map_3_4_path.txt
```