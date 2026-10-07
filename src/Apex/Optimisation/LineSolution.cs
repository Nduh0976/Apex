using System.Collections.Immutable;

namespace Apex.Optimisation;

/// <summary>Lateral offsets for a racing line, and how the solver got there.</summary>
public sealed record LineSolution(ImmutableArray<double> Offsets, int Iterations, double Objective);