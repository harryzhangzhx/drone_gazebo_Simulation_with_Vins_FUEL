#include "LKH.h"
#include "Sequence.h"
#include "Genetic.h"

/*      
 * The FreeStructures function frees all allocated structures.
 */

#define Free(s) { free(s); s = 0; }


int MaxPopulationSize; /* The maximum size of the population */
int PopulationSize;    /* The current size of the population */

CrossoverFunction Crossover;

int** Population;  /* Array of individuals (solution tours) */
GainType* Fitness; /* The fitness (tour cost) of each individual */

void AddToPopulation(GainType Cost);
void ApplyCrossover(int i, int j);
void FreePopulation();
int HasFitness(GainType Cost);
int LinearSelection(int Size, double Bias);
GainType MergeTourWithIndividual(int i);
void PrintPopulation();
void ReplaceIndividualWithTour(int i, GainType Cost);
int ReplacementIndividual(GainType Cost);
 
void ERXT();
void FreeStructures()
{
    FreeCandidateSets();
    FreeSegments();
    if (NodeSet) {
        int i;
        for (i = 1; i <= Dimension; i++) {
            Node *N = &NodeSet[i];
            Free(N->MergeSuc);
            N->C = 0;
        }
        Free(NodeSet);
    }
    Free(CostMatrix);
    Free(BestTour);
    Free(BetterTour);
    Free(SwapStack);
    Free(HTable);
    Free(Rand);
    Free(CacheSig);
    Free(CacheVal);
    Free(Name);
    Free(Type);
    Free(EdgeWeightType);
    Free(EdgeWeightFormat);
    Free(EdgeDataFormat);
    Free(NodeCoordType);
    Free(DisplayDataType);
    Free(Heap);
    Free(t);
    Free(T);
    Free(tSaved);
    Free(p);
    Free(q);
    Free(incl);
    Free(cycle);
    Free(G);
    FreePopulation();
}

/*      
   The FreeSegments function frees the segments.
 */

void FreeSegments()
{
    if (FirstSegment) {
        Segment *S = FirstSegment, *SPrev;
        do {
            SPrev = S->Pred;
            Free(S);
        }
        while ((S = SPrev) != FirstSegment);
        FirstSegment = 0;
    }
    if (FirstSSegment) {
        SSegment *SS = FirstSSegment, *SSPrev;
        do {
            SSPrev = SS->Pred;
            Free(SS);
        }
        while ((SS = SSPrev) != FirstSSegment);
        FirstSSegment = 0;
    }
}

/*      
 * The FreeCandidateSets function frees the candidate sets.
 */

void FreeCandidateSets()
{
    Node *N = FirstNode;
    if (!N)
        return;
    do {
        Free(N->CandidateSet);
        Free(N->BackboneCandidateSet);
    }
    while ((N = N->Suc) != FirstNode);
}
