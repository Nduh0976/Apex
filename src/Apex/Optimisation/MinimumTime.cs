using System.Collections.Immutable;
using Apex.Geometry;
using Apex.Simulation;
using Apex.Tracks;

namespace Apex.Optimisation;

/// <summary>
/// Minimum-lap-time line by projected gradient descent on the lap time
/// itself. The optimiser moves a smooth correction on top of a base line:
///     offsets = base + B b,
/// where B is a cubic B-Spline basis (see <see cref="BSplineBasis"/>). The
/// gradient with respect to each control value b[k] is estimated by central
/// differences, in parallel. The objective is non-convex, so this finds a
/// local improvement on the starting line, not a proof of optimality.
/// </summary>
public static class MinimumTime
{
    /// <param name="Passes">
    /// Outer passes. Each pass makes current line the new base, which
    /// re-centres the control boxes and frees stations that have left a bound.
    /// </param>
    /// <param name="IterationPerPass">Gradient steps per pass, at most</param>
    /// <param name="ControlSPacing">Stations between control values.</param>
    /// <param name="Pertubation">Finite-difference step h, metres.</param>
    /// <param name="InitialMove">Largest control move on the first step, metres.</param>
    /// <param name="MinimumMove">End a pass when no step this small improves the lap</param>
    public sealed record Options(
        double Margin = 1.0,
        int Passes = 4,
        int IterationPerPass = 25,
        int ControlSPacing = 4,
        double Pertubation = 0.05,
        double InitialMove = 0.5,
        double MinimumMove = 0.002);

    public static LineSolution Improve(
        Track track,
        LapSimulator simulator,
        ReadOnlySpan<double> start,
        Options? options = null,
        Action<int, double>? progress = null)
    {
        options ??= new Options();
        var n = track.Count;
        var basis = new BSplineBasis(n, options.ControlSPacing);
        var bounds = new OffsetBounds(track, options.Margin);
        var workspace = new LapWorkspace(n);

        var baseline = start.ToArray();
        bounds.Project(baseline);
        var offsets = new double[n];
        var controls = new double[basis.Count];
        var candidate = new double[basis.Count];
        var gradient = new double[basis.Count];

        var lapTime = Evaluate(controls);
        var totalIterations = 0;

        for (var pass = 0; pass < options.Passes; pass++)
        {
            // b = 0 reproduces the base line exactly, so a pass can never
            // start worse than the previous one ended.
            Array.Clear(controls);
            var (lower, upper) = basis.ControlBounds(bounds, baseline);
            var move = options.InitialMove;
            var stepsThisPass = 0;

            for (var iteration = 0; iteration < options.IterationPerPass; iteration++)
            {
                Gradient(track, simulator, basis, baseline, controls, options.Pertubation, gradient);

                var largest = 0.0;
                
                foreach(var g in gradient)
                {
                    largest = Math.Max(largest, Math.Abs(g));
                }

                if (largest == 0)
                {
                    break;
                }

                // Backtracking line search. The step is scaled so the control
                // with the steepest gradient moves exactly `move` metres, which
                // keeps step in physical units rather than s/m.
                var improved = false;
                while (move >= options.MinimumMove)
                {
                    var scale = move / largest;

                    for (var k = 0; k < candidate.Length; k++)
                    {
                        candidate[k] = Math.Clamp(controls[k] - scale * gradient[k], lower[k], upper[k]);
                    }

                    var candidateTime = Evaluate(candidate);
                    if (candidateTime < lapTime)
                    {
                        (controls, candidate) = (candidate, controls);
                        lapTime = candidateTime;
                        move *= 1.5; // be a little bolder next time
                        improved = true;
                        break;
                    }

                    move *= 0.5;
                }

                if (!improved)
                {
                    break;
                }

                stepsThisPass++;
                totalIterations++;

                // Not progress.?Invoke(++totalIterations, ...): with a null
                // delegate, ?. skips evaluating the arguments too, so the
                // increment would silently vanish.
                progress?.Invoke(totalIterations, lapTime);
            }

            // Fold this pass's correction into the base line.
            Offsets(controls);
            Array.Copy(offsets, baseline, n);

            if (stepsThisPass == 0)
            {
                break; // nothing left to gain
            }
        }

        return new LineSolution(ImmutableArray.Create(baseline), totalIterations, lapTime);

        void Offsets(double[] c)
        {
            basis.Evaluate(c, offsets);
            for (var i = 0; i < n; i++)
            {
                offsets[i] += baseline[i];
            }
        }

        double Evaluate(double[] c)
        {
            Offsets(c);
            track.PointsAt(offsets, workspace.Points);

            return simulator.LapTime(workspace);
        }
    }

    /// <summary>
    /// dT/db[k] ~ (T{b + h e_k) - T(b - h e_k)) / 2h for every control k.
    /// Nudging one control value moves only the few stations it touches, so
    /// each evaluation patches those points and restores them afterwards.
    /// Controls are independent of each other: embarrassingly parallel.
    /// Pass <paramref name="parallelOptions"/> to cap the thread count (the
    /// benchmarks use it measure scaling).
    /// </summary>
    public static void Gradient(
        Track track,
        LapSimulator simulator,
        BSplineBasis basis,
        double[] baseline,
        double[] controls,
        double h,
        double[] gradient,
        ParallelOptions? parallelOptions = null)
    {
        var n = track.Count;
        var offsets = new double[n];
        basis.Evaluate(controls, offsets);

        for (var i = 0; i < n; i++)
        {
            offsets[i] += baseline[i];
        }

        var basePoints = new Vec2[n];
        track.PointsAt(offsets, basePoints);

        var normal = track.Normal;

        Parallel.For(
            0,
            basis.Count,
            parallelOptions ?? new ParallelOptions(),
            // localInit runs once per worker, not once per index: each worker
            // gets a private workspace holding its own copy of the line.
            localInit: () =>
            {
                var ws = new LapWorkspace(n);
                basePoints.CopyTo(ws.Points);
                return ws;
            },
            body: (k, _, ws) =>
            {
                var stations = basis.Stations(k);
                var weights = basis.Weights(k);

                for (var j = 0; j < stations.Length; j++)
                {
                    ws.Points[stations[j]] = basePoints[stations[j]] + (h * weights[j]) * normal[stations[j]];
                }

                var plus = simulator.LapTime(ws);

                for (var j = 0; j < stations.Length; j++)
                {
                    ws.Points[stations[j]] = basePoints[stations[j]] - (h * weights[j]) * normal[stations[j]];
                }

                var minus = simulator.LapTime(ws);

                foreach (var i in stations)
                {
                    ws.Points[i] = basePoints[i]; // restore before the next control
                }

                gradient[k] = (plus - minus) / (2.0 * h);
                return ws;
            },
            localFinally: _ => { });
    }
}
