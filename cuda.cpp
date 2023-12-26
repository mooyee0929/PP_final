#include <stdlib.h>
#include <time.h>
#include "main.h"
#include "dataStructs.h"
#include "util.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <algorithm>    // std::m
#include <chrono>
#include <map>
#include <string>
using namespace std::chrono;

//#define openMP 1

point_t start, goal;
int max_num_of_nodes, num_of_obstacles;
int map_dim_x, map_dim_y;
int dist_to_grow;
node_t final_winning_node;

std::map<int, std::string> step = {
    {0, "generate random point"},
    {1, "find nearest node"},
    {2, "grow from node"},
    {3, "check collid"},
    {4, "RRT*"},
    {5, "check arrive goal"},
};

//copied from assignment 3
static int _argc;
static const char **_argv;

const char *get_option_string(const char *option_name, const char *default_value) {
    for (int i = _argc - 2; i >= 0; i -= 2)
        if (strcmp(_argv[i], option_name) == 0)
            return _argv[i + 1];
    return default_value;
}

int get_option_int(const char *option_name, int default_value) {
    for (int i = _argc - 2; i >= 0; i -= 2)
        if (strcmp(_argv[i], option_name) == 0)
            return atoi(_argv[i + 1]);
    return default_value;
}



static void show_help(const char *program_path) {
    printf("Usage: %s OPTIONS\n", program_path);
    printf("\n");
    printf("OPTIONS:\n");
    printf("\t-f <input_filename> (required)\n");
    printf("\t-n <num_of_threads> (required)\n");
    printf("\t-r <int: 0 = run RRT, 1 = run RRT*> (required)\n");
    printf("\t-d <int: distance to grow: default 5> (required)\n");
}


bool growFromNode(node_t *node, point_t direction, int dist_to_grow, node_t *new_node_return) {
    int x_dist = direction.x - node->point.x;
    int y_dist = direction.y - node->point.y;
    float dist = euclideanDistance(node->point, direction);
    float dist_ratio = dist_to_grow / dist;

    node_t new_node;
    new_node.point.x = node->point.x + (x_dist * dist_ratio);
    new_node.point.y = node->point.y + (y_dist * dist_ratio);
    new_node.cost = node->cost + dist; // Keep track of the cost of the path
    new_node.parent = node;
    if (new_node.point.x < 0 || new_node.point.x >= map_dim_x || new_node.point.y < 0 || new_node.point.y >= map_dim_y)
        return false;
    *new_node_return = new_node;
    return true;
}

point_t generateRandomPoint(int map_dim_x, int map_dim_y){
    point_t point;
    point.x = rand() % map_dim_x;
    point.y = rand() % map_dim_y;
    return point;
}

inline bool LineIntersectsLine(node_t l1p1, node_t l1p2, node_t l2p1, node_t l2p2)
{
        float a = (l2p2.point.x - l2p1.point.x) * (l1p1.point.y - l2p1.point.y) - (l2p2.point.y - l2p1.point.y) * (l1p1.point.x - l2p1.point.x);
        float b = (l1p2.point.x - l1p1.point.x) * (l1p1.point.y - l2p1.point.y) - (l1p2.point.y - l1p1.point.y) * (l1p1.point.x - l2p1.point.x);
        float d = (l2p2.point.y - l2p1.point.y) * (l1p2.point.x - l1p1.point.x) - (l2p2.point.x - l2p1.point.x) * (l1p2.point.y - l1p1.point.y);

        if( d == 0 &&(a==0 || b==0))
        {
            return false;
        }

        float ua = a/d;
        float ub = b/d;

        if( (ua>=0 && ua<=1) && (ub>=0 && ub<=1))
        {
            return true;
        }

        return false;
}

// Collision checker based off of by: https://stackoverflow.com/questions/5514366/how-to-know-if-a-line-intersects-a-rectangle
inline bool doesOverlapCollide(rect_t *obstacles, int num_of_obstacles, node_t nearnode, node_t newnode){
    int padding = 1;
    for(int i = 0; i < num_of_obstacles; i++){
        rect_t o = obstacles[i];
        // two node on the same side
        int x1_pad = o.x1-padding;
        int x2_pad = o.x2+padding;
        int y1_pad = o.y1-padding;
        int y2_pad = o.y2+padding;
        if(newnode.point.x < x1_pad && nearnode.point.x < x1_pad  ||
            newnode.point.x > x2_pad && nearnode.point.x > x2_pad  ||
            newnode.point.y < y1_pad && nearnode.point.y < y1_pad  ||
            newnode.point.y > y2_pad && nearnode.point.y > y2_pad){
            continue;
        }
        //one node in the rec
        if(newnode.point.x >= x1_pad && newnode.point.x <= x2_pad  && newnode.point.y >= y1_pad && newnode.point.y <= y2_pad ||
            newnode.point.x >= x1_pad && newnode.point.x <= x2_pad  && newnode.point.y >= y1_pad && newnode.point.y <= y2_pad){
            return true;
        }
        node_t node0,node1,node2,node3;
        node0.point.x = x1_pad;
        node0.point.y = y1_pad;
        node1.point.x = x1_pad;
        node1.point.y = y2_pad;
        node2.point.x = x2_pad;
        node2.point.y = y1_pad;
        node3.point.x = x2_pad;
        node3.point.y = y2_pad;

        if(LineIntersectsLine(nearnode, newnode, node0,node1) ||
               LineIntersectsLine(nearnode, newnode, node0,node2) ||
               LineIntersectsLine(nearnode, newnode, node1,node3) ||
               LineIntersectsLine(nearnode, newnode, node2,node3)){
            return true;
        }
    }
    return false;
}


extern bool findNearestNodeToCoordinateCUDA(point_t coordinate, node_t *list_of_nodes, int num_of_nodes, node_t **nearest_node,
                                     int map_dim_x, int map_dim_y, int dist_to_grow);
 
int main(int argc, const char *argv[]) {    
    // Source: https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree#Algorithm
    //
    // Algorithm BuildRRT
    // Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq
    // Output: RRT graph G
    //
    // G.init(qinit)
    // for k = 1 to K do
    //     qrand ← RAND_CONF()
    //     qnear ← NEAREST_VERTEX(qrand, G)
    //     qnew ← NEW_CONF(qnear, qrand, Δq)
    //     G.add_vertex(qnew)
    //     G.add_edge(qnear, qnew)
    // return G
    //
    // In the algorithm above, "RAND_CONF" grabs a random configuration qrand in C. 
    // This may be replaced with a function "RAND_FREE_CONF" that uses samples in Cfree, 
    // while rejecting those in Cobs using some collision detection algorithm.
    // 
    // "NEAREST_VERTEX" is a function that runs through all vertices v in graph G, 
    // calculates the distance between qrand and v using some measurement function thereby 
    // returning the nearest vertex.
    // 
    // "NEW_CONF" selects a new configuration qnew by moving an incremental distance Δq 
    // from qnear in the direction of qrand. (According to [4] in holonomic problems, this 
    // should be omitted and qrand used instead of qnew.)

    _argc = argc - 1;
    _argv = argv + 1;

    const char *input_filename = get_option_string("-f", NULL);
    int num_of_threads = get_option_int("-n", 1);
    int rrt_star_flag = get_option_int("-r", 0);
    dist_to_grow = get_option_int("-d", 5);
    
    //omp_set_num_threads(num_of_threads);

    //int nthreads = omp_get_num_threads();
    //printf("Number of threads = %d\n", nthreads);

    if (input_filename == NULL) {
        printf("Error: You need to specify -f.\n");
        show_help(argv[0]);
        return 1;
    }

    printf("Number of threads: %d\n", num_of_threads);
    printf("Input file: %s\n", input_filename);


    // Parse input filepath for basename
    char *input_basename = strdup(input_filename);
    // printf("%s\n",input_basename);
    char *basename_ptr = strrchr(input_basename, '/');
    // printf("%s\n",basename_ptr);
    if (basename_ptr != NULL) {
        basename_ptr++;
        input_basename = basename_ptr;
    }
    char *ext = strrchr(input_basename, '.');
    if (ext != NULL)
        *ext = '\0';


    FILE *input = fopen(input_filename, "r");

    if (!input) {
        printf("Unable to open file: %s.\n", input_filename);
        return 1;
    }

    fscanf(input, "%d %d\n", &map_dim_x, &map_dim_y);
    fscanf(input, "%d\n", &max_num_of_nodes);
    fscanf(input, "%d %d\n", &start.x, &start.y);
    fscanf(input, "%d %d\n", &goal.x, &goal.y);
    fscanf(input, "%d\n", &num_of_obstacles);
    printf("Dimensions: %d x %d\n", map_dim_x, map_dim_y);
    printf("Max number of nodes: %d\n", max_num_of_nodes);
    printf("Start: (%d, %d)\n", start.x, start.y);
    printf("Goal: (%d, %d)\n", goal.x, goal.y);
    printf("Number of obstacles: %d\n", num_of_obstacles);


    // initialize map with obstacles
    rect_t obstacles[num_of_obstacles];
    for(int obs_incr = 0; obs_incr < num_of_obstacles; obs_incr++){
        fscanf(input, "%d %d %d %d\n", &obstacles[obs_incr].x1, &obstacles[obs_incr].y1, &obstacles[obs_incr].x2, &obstacles[obs_incr].y2);
    }

    // initialize start and goal
    node_t list_of_nodes[max_num_of_nodes];
    list_of_nodes[0] = (node_t) {
        .point = start,
        .cost = 0,
        .parent = NULL
        // .child = NULL
    };

    node_t nodegoal;
    nodegoal = (node_t) {
        .point = goal,
        .cost = 0,
        .parent = NULL
        // .child = NULL
    };

    // random seed based on current time
    // srand (time(NULL));
    srand(0); // TODO: REVERT THIS OR ELSE RANDOMNESS IS NOT RANDOM


    int num_nodes_generated = 0;
    uint32_t best_cost = 0;
    int num_of_winning_nodes = 0;

    uint32_t timers[10] = {0};
    auto start_time = std::chrono::high_resolution_clock::now();
    for(num_nodes_generated = 1; num_nodes_generated < max_num_of_nodes; num_nodes_generated++){

        node_t *nearest_vertex;
        point_t random_point;
        node_t candidate_node;
        
        auto start_1 = high_resolution_clock::now();
        auto end_1 = high_resolution_clock::now();
        auto end_2 = high_resolution_clock::now();
        auto end_3 = high_resolution_clock::now();

        do{
            // genereate a random point

            // timing code
            start_1 = high_resolution_clock::now();

            random_point = generateRandomPoint(map_dim_x, map_dim_y);
            // printf("Random point: (%d, %d)\n", random_point.x, random_point.y);

            // timing code
            end_1 = high_resolution_clock::now();

            // find the nearest vertex
            bool res = findNearestNodeToCoordinateCUDA(random_point, list_of_nodes, num_nodes_generated, &nearest_vertex, map_dim_x, map_dim_y, dist_to_grow);
            if (!res) {
                // Closest node is too close...just skip this iteration
                continue;
            }

            // timing code
            end_2 = high_resolution_clock::now();
            
            // generate a new node by growing from the nearest vertex
            res = growFromNode(nearest_vertex, random_point, dist_to_grow, &candidate_node);

            // timing code
            end_3 = high_resolution_clock::now();

            if (!res) {
                // Closest node is out of map...just skip this iteration
                // But I think this won't happen because they already limit the range of random_points
                continue;
            }
        // check if node is valid
        } while(doesOverlapCollide(obstacles, num_of_obstacles, *nearest_vertex, candidate_node));


        auto end_4 = high_resolution_clock::now();
        // run RRT* refinement
        //if(rrt_star_flag){
        //    run_rrt_star(&candidate_node, list_of_nodes, num_nodes_generated, obstacles, num_of_obstacles);
        //}
        auto end_5 = high_resolution_clock::now();

        assert(candidate_node.point.x >= 0 && candidate_node.point.x < map_dim_x);
        assert(candidate_node.point.y >= 0 && candidate_node.point.y < map_dim_y);

        // add node
        list_of_nodes[num_nodes_generated] = candidate_node;

        // check if the new node is the goal
        if(closerThanDistSquared(candidate_node.point, goal, pow(dist_to_grow, 2))){
            //printf("Found goal \n");
            if (num_of_winning_nodes == 0) {
                best_cost = candidate_node.cost;
                final_winning_node = candidate_node;
                num_of_winning_nodes++;
            }
            else{
                if(candidate_node.cost < best_cost){
                    best_cost = candidate_node.cost;
                    final_winning_node = candidate_node;
                }
                num_of_winning_nodes++;
            }
        }

        auto end_6 = high_resolution_clock::now();

        timers[0] += duration_cast<microseconds>(end_1 - start_1).count();
        timers[1] += duration_cast<microseconds>(end_2 - end_1).count();
        timers[2] += duration_cast<microseconds>(end_3 - end_2).count();
        timers[3] += duration_cast<microseconds>(end_4 - end_3).count();
        timers[4] += duration_cast<microseconds>(end_5 - end_4).count();
        timers[5] += duration_cast<microseconds>(end_6 - end_5).count();

    }
    auto end_time = std::chrono::high_resolution_clock::now();

    printf("\n number of winning nodes: %d \n\n", num_of_winning_nodes);

    for(int i = 0; i < 6; i++){
        printf("%s time : %ld\n", step[i].c_str() ,timers[i]);
    }

    printf("\n\n");

    printf("overall time: %ld\n", duration_cast<microseconds>(end_time - start_time).count());

    /////////////////////////////////////////////////////////////


    // Save the graph/nodes to a file
    char nodes_output[256];
    snprintf(nodes_output, 256, "outputs/%s_%d_nodes.txt", input_basename, num_of_threads);

    FILE *nodes_output_file = fopen(nodes_output, "w");

    fprintf(nodes_output_file, "%d\n", num_nodes_generated);
    for(int node_iter = 0; node_iter < num_nodes_generated; node_iter++) {
        int parent_index = (list_of_nodes[node_iter].parent - list_of_nodes);
        fprintf(nodes_output_file, "%d %d %d\n", list_of_nodes[node_iter].point.x, list_of_nodes[node_iter].point.y, parent_index);
    }
    printf("Done writing nodes list to file: %s\n", nodes_output);

    // print out stats
    printf("Number of nodes generated: %d \n", num_nodes_generated);

    if (num_of_winning_nodes == 0) {
        printf("Max number of nodes reached and no path found.\n");
        printf("Exiting gracefully without saving path. Please increase max num nodes \n");
        return -1;
    }

    
    /////////////////////////////////////////////////////////////
    

    // traverse the path backwards and print out the points
    node_t current_node = final_winning_node;
    int path_length = 0;
    while(current_node.point.x != start.x || current_node.point.y != start.y){
        int parent_index = (current_node.parent - list_of_nodes);// / sizeof(node_t);
        // printf("%d %d %d\n", current_node.point.x, current_node.point.y, parent_index);
        current_node = list_of_nodes[parent_index];
        path_length++;
    }
    printf("Path length: %d\n", path_length);
    printf("Path Cost: %d\n", final_winning_node.cost);


    // Save the path to a file
    char path_output[256];
    snprintf(path_output, 256, "outputs/%s_%d_path.txt", input_basename, num_of_threads);
    FILE *path_output_file = fopen(path_output, "w");

    fprintf(path_output_file, "%d\n", path_length);
    fprintf(path_output_file, "%d\n", final_winning_node.cost);

    // traverse again to print
    fprintf(path_output_file, "%d %d %d\n", nodegoal.point.x, nodegoal.point.y, 0);

    current_node = final_winning_node;
    int parent_index;
    while(current_node.point.x != start.x || current_node.point.y != start.y){
        fprintf(path_output_file, "%d %d %d\n", current_node.point.x, current_node.point.y, parent_index);
        parent_index = (current_node.parent - list_of_nodes);// / sizeof(node_t);
        current_node = list_of_nodes[parent_index];
    }
    fprintf(path_output_file, "%d %d %d\n", current_node.point.x, current_node.point.y, parent_index);

    printf("Writing nodes list to file: %s\n", path_output);
    
}
