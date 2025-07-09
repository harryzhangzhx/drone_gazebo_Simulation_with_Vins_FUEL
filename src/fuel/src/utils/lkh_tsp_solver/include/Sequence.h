#ifndef SEQUENCE_H
#define SEQUENCE_H

/*
 * This header specifies the interface for the use of node sequences.
 *
 * The functions BestKOptMove and BacktrackKOptMove are implemented
 * by means of such sequences.
 */

#include "LKH.h"

extern Node** t;      /* The sequence of nodes to be used in a move */
extern Node** T;      /* The currently best t's */
extern Node** tSaved; /* For saving t when using the BacktrackKOptMove function */
extern int* p;        /* The permutation corresponding to the sequence in which
                   the t's occur on the tour */
extern int* q;        /* The inverse permutation of p */
extern int* incl;     /* Array: incl[i] == j, if (t[i], t[j]) is an inclusion edge */
extern int* cycle;    /* Array: cycle[i] is cycle number of t[i] */
extern GainType* G;   /* For storing the G-values in the BestKOptMove function */
extern int K;         /* The value K for the current K-opt move */
 
extern int FeasibleKOptMove(int k);
extern void FindPermutation(int k);
extern int Cycles(int k);
 
extern int Added(const Node* ta, const Node* tb);
extern int Deleted(const Node* ta, const Node* tb);
 
extern void MarkAdded(Node* ta, Node* tb);
extern void MarkDeleted(Node* ta, Node* tb);
extern void UnmarkAdded(Node* ta, Node* tb);
extern void UnmarkDeleted(Node* ta, Node* tb);

#endif
