using System.Collections.Immutable;
using Apex.Geometry;
using Apex.Tracks;

namespace Apex.Optimisation;

/// <summary>
/// Minimum-curvature racing line. Minimises
///   J(a) = sum_i | p[i-1] - 2 p[i] + p[i+1] |^2, p[i] = c[i] + a[i] n[i]
/// subject to box bounds on a. J is a convex quadratic, so accelerated
/// projected gradient descent converges to the global minimum.
/// </summary>
public static class MinimumCurvature
{
    /// <param name="Reweightings">
    /// Extra passes that correct for uneven point spacing. 0 gives the plain
    /// equal-spacing objective.
    /// </param>
    public sealed record Options(
        double Margin = 1.0,
        int MaxIterations = 100_000,
        double Tolerance = 1e-5,
        int Reweightings = 2);

    public static LineSolution Solve(Track track, Options? options = null)
    {
        options ??= new Options();
        var n = track.Count;
        var weights = new double[n];
        Array.Fill(weights, 1.0);

        var solution = SolveWeighted(track, weights, new double[n], options);
        var totalIterations = solution.Iterations;
        var points = new Vec2[n];

        for (var pass = 0; pass < options.Reweightings; pass++)
        {
            track.PointsAt(solution.Offsets.AsSpan(), points);
            SpacingWeights(points, weights);
            solution = SolveWeighted(track, weights, solution.Offsets.AsSpan().ToArray(), options);
            totalIterations += solution.Iterations;
        }

        return solution with { Iterations = totalIterations };
    }

    /// <summary>
    /// w[i] = (mean spacing / local spacing)^3. Discrete curvature is
    /// |r[i]| / l[i]^2, and the integral of curvature squared weights each
    /// station by its length l[i], so the faithful objective is
    /// sum |r[i]|^2 / l[i]^3. Normalising by the mean keeps w near 1.
    /// </summary>
    public static void SpacingWeights(ReadOnlySpan<Vec2> points, Span<double> weights)
    {
        var n = points.Length;
        var mean = 0.0;

        for (var i = 0; i < n; i++)
        {
            mean += Vec2.Distance(points[i], points[Next(i, n)]);
        }

        mean /= n;

        for (var i = 0; i < n; i++)
        {
            var local = 0.5 * (Vec2.Distance(points[Prev(i, n)], points[i]) + Vec2.Distance(points[i], points[Next(i, n)]));
            var ratio = mean / local;

            weights[i] = ratio * ratio * ratio;
        }
    }

    private static LineSolution SolveWeighted(Track track, ReadOnlySpan<double> weights, double[] x, Options options)
    {
        var n = track.Count;
        var c = track.Centre.AsSpan();
        var normal = track.Normal.AsSpan();
        var bounds = new OffsetBounds(track, options.Margin);

        // Second difference of the centreline: the part of the residual that
        // does not depend on the offsets.
        var d = new Vec2[n];

        for (var i = 0; i < n; i++)
        {
            d[i] = c[Prev(i, n)] - 2.0 * c[i] + c[Next(i, n)];
        }

        // Each residual involves three offsets with weights 1, -2, 1 on unit
        // normals, so ||A|| <= 4 and the gradient's Lipschitz constant
        // L = 2 max(w) ||A||^2 <= 32 max(w). A step of 1/L is always safe.
        var maxWeight = 0.0;
        
        foreach (var w in weights)
        {
            maxWeight = Math.Max(maxWeight, w);
        }

        var step = 1.0 / (32.0 * maxWeight);

        var previous = x.AsSpan().ToArray(); // previous iterate
        var y = new double[n]; // extrapolated point
        var residual = new Vec2[n];
        var gradient = new double[n];
        var t = 1.0;

        var iteration = 0;

        for (; iteration < options.MaxIterations; iteration++)
        {
            // Nesterov extrapolation: y = x + momentum * (x - previous)
            var tNext = 0.5 * (1.0 + Math.Sqrt(1.0 + 4.0 * t * t));
            var momentum = (t - 1.0) / tNext;

            for (var i = 0; i < n; i++)
            {
                y[i] = x[i] + momentum * (x[i] - previous[i]);
            }

            Gradient(d, normal, weights, y, residual, gradient);

            //Gradient step from y, then project back onto the bounds.
            Array.Copy(x, previous, n);
            for (var i = 0; i < n; i++)
            {
                x[i] = y[i] - step * gradient[i];
            }

            bounds.Project(x);

            // Adaptive restart (O'Donoghue & Candes): if momemntum is carrying
            // us uphill, throw it away. This removes most of the oscillation
            // that plain Nesterov momentum shows, at almost no cost.
            var uphiill = 0.0;
            var stationarity = 0.0;

            for (var i = 0; i < n; i++)
            {
                uphiill += (y[i] - x[i]) * (x[i] - previous[i]);
                stationarity = Math.Max(stationarity, Math.Abs(y[i] - x[i]));
            }

            t = uphiill > 0 ? 1.0 : tNext;

            // Stop on the gradient mapping (y -x) / step: the projected
            // gradient, which is zero exactly at a constrained minimum. A
            // small step between iterates is NOT enough on it's own, because
            // this problem is ill-conditioned: smooth, track-wide shifts of
            // the line change J very little, so the iterates crawl long
            // before they arrive.
            if (stationarity / step < options.Tolerance)
            {
                break;
            }
        }

        return new LineSolution(ImmutableArray.Create(x), iteration, Objective(d, normal, weights, x));
    }
    
    /// <summary> Unweighted J(a) for any offsets. Useful for tests and reporting.</summary>
    public static double Objective(Track track, ReadOnlySpan<double> offsets)
    {
        var n = track.Count;
        var c = track.Centre.AsSpan();
        var d = new Vec2[n];

        for (var i = 0; i < n; i++)
        {
            d[i] = c[Prev(i, n)] - 2.0 * c[i] + c[Next(i, n)];
        }

        var weights = new double[n];
        Array.Fill(weights, 1.0);

        return Objective(d, track.Normal.AsSpan(), weights, offsets);
    }

    private static double Objective(ReadOnlySpan<Vec2> d, ReadOnlySpan<Vec2> normal, ReadOnlySpan<double> weights, ReadOnlySpan<double> a)
    {
        var n = d.Length;
        var sum = 0.0;

        for (var i = 0; i < n; i++)
        {
            sum += weights[i] * Residual(d, normal, a, i, n).LengthSquared; 
        }

        return sum;
    }

    /// <summary>
    /// dJ/da[j] = 2 n[j] . (w[j-1] r[j-1] - 2 w[j] r[j] + w[j+1] r[j+1]).
    /// The gradient is the same 1, -2, 1 stencil applied to the weighted
    /// residuals, because A-transpose has the same shape as A.
    /// </summary>
    private static void Gradient(
        ReadOnlySpan<Vec2> d,
        ReadOnlySpan<Vec2> normal,
        ReadOnlySpan<double> weights,
        ReadOnlySpan<double> a,
        Span<Vec2> residual,
        Span<double> gradient)
    {
        var n = d.Length;

        for (var i = 0; i < n; i++)
        {
            residual[i] = weights[i] * Residual(d, normal, a, i, n);
        }

        for (var j = 0; j < n; j++)
        {
            var stencil = residual[Prev(j, n)] - 2.0 * residual[j] + residual[Next(j, n)];
            gradient[j] = 2.0 * Vec2.Dot(normal[j], stencil);
        }
    }

    private static Vec2 Residual(ReadOnlySpan<Vec2> d, ReadOnlySpan<Vec2> normal, ReadOnlySpan<double> a, int i, int n)
    {
        int p = Prev(i, n), q = Next(i, n);

        return d[i] + a[p] * normal[p] - 2.0 * a[i] * normal[i] + a[q] * normal[q];
    }

    private static int Prev(int i, int n) => i == 0 ? n - 1 : i - 1;
    private static int Next(int i, int n) => i == n - 1 ?  0 : i + 1;
}
