#ifndef _GENETIC_H
#define _GENETIC_H

/*
 * This header specifies the interface for the genetic algorithm part of LKH.
 */

typedef void (*CrossoverFunction)();

extern int MaxPopulationSize; /* The maximum size of the population */
extern int PopulationSize;    /* The current size of the population */

extern CrossoverFunction Crossover;

extern int** Population;  /* Array of individuals (solution tours) */
extern GainType* Fitness; /* The fitness (tour cost) of each individual */

extern void AddToPopulation(GainType Cost);
extern void ApplyCrossover(int i, int j);
extern void FreePopulation();
extern int HasFitness(GainType Cost);
extern int LinearSelection(int Size, double Bias);
extern GainType MergeTourWithIndividual(int i);
extern void PrintPopulation();
extern void ReplaceIndividualWithTour(int i, GainType Cost);
extern int ReplacementIndividual(GainType Cost);
 
extern void ERXT();

#endif
