#include <LKH.h>

/*
 * If a node is made "active", attempts are made to find an improving 
 * move with the node as anchor node, t1.
 *
 * The Active function makes a node active by inserting it into a 
 * queue of active nodes. FirstActive denotes the front node of 
 * the queue. LastActive denotes the rear. 
 *
 * The queue is implemented as a circular list in which the Next field 
 * of each Node references the successor node. 
 *
 * A node is member of the queue iff its Next != 0. The function has no 
 * effect if the node is already in the queue. 
 *
 * The function is called from the StoreTour function.  
 */
 int AscentCandidates;
 int BackboneTrials;
 int Backtracking;
 GainType BestCost;
 int* BestTour;
 GainType BetterCost;
 int* BetterTour;
 int CacheMask;
 int* CacheVal;
 int* CacheSig;
 int CandidateFiles;
 int* CostMatrix;
 int Dimension;
 int DimensionSaved;
 int EdgeFiles;
 double Excess;
 int ExtraCandidates;
 Node *FirstActive, *LastActive;
 Node* FirstNode;
 Segment* FirstSegment;
 SSegment* FirstSSegment;
 int Gain23Used;
 int GainCriterionUsed;
 double GridSize;
 int GroupSize;
 int SGroupSize;
 int Groups;
 int SGroups;
 unsigned Hash;
 Node** Heap;
 HashTable* HTable;
 int InitialPeriod;
 int InitialStepSize;
 double InitialTourFraction;
 char* LastLine;
 double LowerBound;
 int Kicks;
 int KickType;
 int M;
 int MaxBreadth;
 int MaxCandidates;
 int MaxMatrixDimension;
 int MaxSwaps;
 int MaxTrials;
 int MergeTourFiles;
 int MoveType;
 Node* NodeSet;
 int Norm;
 int NonsequentialMoveType;
 GainType Optimum;
 int PatchingA;
 int PatchingC;
 int Precision;
 int PredSucCostAvailable;
 int POPMUSIC_InitialTour;
 int POPMUSIC_MaxNeighbors;
 int POPMUSIC_SampleSize;
 int POPMUSIC_Solutions;
 int POPMUSIC_Trials;
 unsigned* Rand;
 int Recombination;
 int RestrictedSearch;
 short Reversed;
 int Run;
 int Runs;
 unsigned Seed;
 int StopAtOptimum;
 int Subgradient;
 int SubproblemSize;
 int SubsequentMoveType;
 int SubsequentPatching;
 SwapRecord* SwapStack;
 int Swaps;
 double TimeLimit;
 int TraceLevel;
 int Trial;
 char *ParameterFileName, *ProblemFileName, *PiFileName, *TourFileName, *OutputTourFileName,
    *InputTourFileName, **CandidateFileName, **EdgeFileName, *InitialTourFileName,
    *SubproblemTourFileName, **MergeTourFileName;
 char *Name, *Type, *EdgeWeightType, *EdgeWeightFormat, *EdgeDataFormat, *NodeCoordType, *DisplayDataType;
 int CandidateSetSymmetric, CandidateSetType, CoordType, DelaunayPartitioning, DelaunayPure,
    ExtraCandidateSetSymmetric, ExtraCandidateSetType, InitialTourAlgorithm, KarpPartitioning,
    KCenterPartitioning, KMeansPartitioning, MoorePartitioning, PatchingAExtended, PatchingARestricted,
    PatchingCExtended, PatchingCRestricted, ProblemType, RohePartitioning, SierpinskiPartitioning,
    SubproblemBorders, SubproblemsCompressed, WeightType, WeightFormat;

 FILE *ParameterFile, *ProblemFile, *PiFile, *InputTourFile, *TourFile, *InitialTourFile,
    *SubproblemTourFile, **MergeTourFile;
 CostFunction Distance, D, C, c;
 MoveFunction BestMove, BacktrackMove, BestSubsequentMove;
 MergeTourFunction MergeWithTour;





void Activate(Node * N)
{
    if (N->Next != 0)
        return;
    if (FirstActive == 0)
        FirstActive = LastActive = N;
    else
        LastActive = LastActive->Next = N;
    LastActive->Next = FirstActive;
}
