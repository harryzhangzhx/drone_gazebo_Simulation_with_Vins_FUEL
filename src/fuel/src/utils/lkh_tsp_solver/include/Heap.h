#ifndef _HEAP_H
#define _HEAP_H

/*
 * This header specifies the interface for the use of heaps.
 */

#include "LKH.h"

extern void MakeHeap(int Size);
extern void HeapInsert(Node* N);
extern void HeapDelete(Node* N);
extern Node* HeapDeleteMin(void);
extern void HeapLazyInsert(Node* N);
extern void Heapify(void);
extern void HeapSiftUp(Node* N);
extern void HeapSiftDown(Node* N);

#endif
