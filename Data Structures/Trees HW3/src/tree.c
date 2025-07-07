#include "tree.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

TreeNode *init_node(GameState *gs){
    TreeNode *node = (TreeNode*)malloc(sizeof(TreeNode));
    if (node == NULL) return NULL;

    node->num_children = -1;
    node->game_state = gs;
    node->children = NULL;

    return node;
}

TreeNode *init_tree(GameState *gs, int depth){
    if(get_game_status(gs)!=3){
        TreeNode *root = init_node(gs);
        return root; 
    }
    TreeNode *root = init_node(gs);
    if(depth==1) return root;

    bool moves[gs->width];
    for(int i=0;i<gs->width;i++){
        moves[i]=false;
    }

    int moveNumber = available_moves(gs, moves);
    root->num_children=moveNumber;
    root->children = (TreeNode**)malloc(moveNumber*sizeof(TreeNode*));
    if(root->children==NULL) return NULL;
    
    int index=0;
    for(int i=0;i<gs->width;i++){
        if(moves[i]==true){
            GameState *newState = make_move(gs, i);
            root->children[index] = init_tree(newState, depth - 1);
            index++;
        }
    }
    return root;
}

void free_tree(TreeNode *root){
    if (root == NULL) return;
    

    for (int i = 0; i < root->num_children; i++) {
        free_tree(root->children[i]);
    }
    
    free(root->children);
    free_game_state(root->game_state);
    free(root);
}

void expand_tree(TreeNode *root){
    if(get_game_status(root->game_state)!=3){
        return;
    }
    if (root->children == NULL) {
        bool moves[root->game_state->width];
        for(int i=0;i<root->game_state->width;i++){
            moves[i]=false;
        }

        int moveNumber = available_moves(root->game_state, moves);

        root->num_children=moveNumber;
        root->children = (TreeNode**)malloc(moveNumber*sizeof(TreeNode*));
        if(root->children==NULL) return;

        int index=0;
        for(int i=0;i<root->game_state->width;i++){
            if(moves[i]){
                GameState *newState = make_move(root->game_state, i);
                root->children[index] = init_node(newState);
                index++;
            }
        }
    }else{
        for(int i=0;i<root->num_children;i++){
            expand_tree(root->children[i]);
        }
    }
}

int node_count(TreeNode *root){
    int number=1;
    for(int i=0;i<root->num_children;i++){
        number+=node_count(root->children[i]);
    }
    return number;
}

void print_tree(TreeNode *root, int level){
    return;
}

