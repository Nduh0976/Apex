namespace Apex.Geometry;

/// <summary>
/// Reusable scratch memory for fitting a periodic cubic spline through
/// <see cref="Count"/> points. Allocate one per thread and reuse it: the
/// lap-time optimiser fits thousands of splines per second, and allocating
/// nine arrays each time would hand the garbage collector most of the work.
/// </summary>
public sealed class SplineWorkspace
{
    public SplineWorkspace(int count)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(count, 3);

        Count = count;
        H = new double[count];
        Sub = new double[count];
        Diag = new double[count];
        Sup = new double[count];
        Mx = new double[count];
        My = new double[count];
        Solver = new double[3 * count];
    }

    public int Count { get; }

    /// <summary> Chord length from knot i to knot i + 1 (wrapping).</summary>
    internal double[] H { get; }
    internal double[] Sub { get; }
    internal double[] Diag { get; }
    internal double[] Sup { get; }

    /// <summary> Second derivative of x(t) and y(t) at each knot.</summary>
    internal double[] Mx { get; }
    internal double[] My { get; }
    internal double[] Solver { get; }
}
