using System.Collections.Immutable;
using Apex.Geometry;
using Apex.Vehicles;

namespace Apex.Simulation;

/// <summary>
/// Quasi-steady-state lap simulation of point mass on a friction ellipse.
/// Given a closed racing line, finds the fastest speed profile the car can
/// follow and the resulting lap time.
/// </summary>
public sealed class LapSimulator(Vehicle vehicle)
{
    private const double MinimumSpeed = 1.0; // m/s, keeps P / v finite

    public Vehicle Vehicle { get; } = vehicle ?? throw new ArgumentNullException(nameof(vehicle));

    /// <summary>
    /// Convenience overload: allocates a workspace and returns the full profile.
    /// Fine for one-off use; the optimisers call <see cref="LapTime"/> instead.
    /// </summary>
    public LapResult Simulate(ReadOnlySpan<Vec2> points)
    {
        var ws = new LapWorkspace(points.Length);
        points.CopyTo(ws.Points);
        var time = LapTime(ws);

        var n = points.Length;
        var speed = new double[n];

        for (var i = 0; i < n; i++)
        {
            speed[i] = Math.Min(ws.Forward[i], ws.Backward[i]);
        }

        return new LapResult(
            time,
            ImmutableArray.Create(speed),
            ImmutableArray.Create(ws.Curvature),
            ImmutableArray.Create(ws.SegmentLength));
    }

    /// <summary>
    /// Lap time for the line in <c>ws.Points</c> Allocation-free: everything
    /// it touches lives in the workspace
    /// </summary>
    public double LapTime(LapWorkspace ws)
    {
        var n = ws.Count;
        var points = ws.Points.AsSpan();
        var ds = ws.SegmentLength.AsSpan();
        var kappa = ws.Curvature.AsSpan();
        var ceiling = ws.Ceiling.AsSpan();
        var forward = ws.Forward.AsSpan();
        var backward = ws.Backward.AsSpan();
        var grip = Vehicle.Grip;
        var top = Vehicle.TopSpeed;

        // 1. Geometry: segment lengths and curvature
        for (var i = 0; i < n; i++)
        {
            ds[i] = Vec2.Distance(points[i], points[i == n - 1 ? 0 : i + 1]);
        }

        SplineMath.CurvatureAtKnots(points, kappa, ws.Spline);

        // 2. Speed ceiling from lateral grip; remember the tightest point.
        var start = 0;

        for (var i = 0; i < n; i++)
        {
            ceiling[i] = grip.CorneringSpeed(Math.Abs(kappa[i]), top);
            
            if (ceiling[i] < ceiling[start])
            {
                start = i;
            }
        }

        // 3. Forward pass: accelerate out of every corner/
        //    The tightest point is the one place we know the speed for sure.

        forward[start] = ceiling[start];
        for (var step = 0; step < n; step++)
        {
            var i = (start + step) % n;
            var next = i == n - 1 ? 0 : i + 1;
            var v = forward[i];
            var a = DriveAcceleration(v, kappa[i]);
            var reachable = Math.Sqrt(Math.Max(0.0, v * v + 2.0 * a * ds[i]));

            if (next != start)
            {
                forward[next] = Math.Min(ceiling[next], reachable);
            }
        }

        // 4. Backward pass: walk the lap in reverse, asking how fast the car
        //    could be at i and stll brake down to the speed at i + 1.
        backward[start] = ceiling[start];
        for (var step = 0; step < n; step++)
        {
            var i = ((start - step) % n + n) % n;
            var previous = i == 0 ? n - 1 : i - 1;
            var v = backward[i];
            var a = BrakeDeceleration(v, kappa[i]);
            var reachable = Math.Sqrt(v * v + 2.0 * a * ds[previous]);

            if (previous != start)
            {
                backward[previous] = Math.Min(ceiling[previous], reachable);
            }
        }

        // 5. The car does whichever limit bites first. Intergrate time.
        var time = 0.0;

        for (var i = 0; i < n; i++)
        {
            var next = i == n - 1 ? 0 : i + 1;
            var v0 = Math.Min(forward[i], backward[i]);
            var v1 = Math.Min(forward[next], backward[next]);
            time += 2.0 * ds[i] / (v0 + v1);
        }

        return time;
    }

    /// <summary>Net forward acceleration available at speed v on curvature k.</summary>
    private double DriveAcceleration(double v, double kappa)
    {
        var tyre = RemainingLongitudinal(v, kappa);
        var power = Vehicle.Power / (Vehicle.Mass * Math.Max(v, MinimumSpeed));

        return Math.Min(tyre, power) - Vehicle.DragDeceleration(v);
    }

    /// <summary>Total deceleration available when braking: tyres plus drag.</summary>
    private double BrakeDeceleration(double v, double kappa)
    {
        return RemainingLongitudinal(v, kappa) + Vehicle.DragDeceleration(v);
    }

    ///<summary>
    /// The friction ellipse: whatever lateral grip cornering uses, the
    /// longitudinal limit shrinks to match.
    ///</summary>
    private double RemainingLongitudinal(double v, double kappa)
    {
        var grip = Vehicle.Grip;
        var used = v * v * Math.Abs(kappa) / grip.LateralLimit(v);
        var remaining = used >= 1.0 ? 0.0 : Math.Sqrt(1.0 - used * used);

        return grip.LongitudinalLimit(v) * remaining;
    }
}
