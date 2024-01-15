#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#include <stdint.h>

typedef struct{
    int x1;
    int x2; 
    int y1;
    int y2;
} rect_t;

typedef struct{
    int x;
    int y;
} point_t;

typedef struct node{
    point_t point;
    uint32_t cost;
    struct node *parent;
} node_t;

typedef struct findnearthread
{
    point_t coordinate;
    node_t *list_of_nodes;
    int points_per_thread;
    int offset;

}threadstrcut0;

typedef struct mincostthread
{
    node_t *node_to_refine;
    node_t *list_of_nodes;
    rect_t *obstacles;
    int points_per_thread;
    int offset;


}threadstrcut1;

typedef struct changecostthread
{
    node_t *node_to_refine;
    node_t *list_of_nodes;
    rect_t *obstacles;
    int points_per_thread;
    int offset;


}threadstrcut2;

#endif