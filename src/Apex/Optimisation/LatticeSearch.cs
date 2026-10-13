using System.Collections.Immutable;
using Apex.Geometry;
using Apex.Tracks;
using Apex.Vehicles;

namespace Apex.Optimisation;

/// <summary>
/// Dynamic programming over a lattice: at every station the car may sit in
/// one of M lanes (fixed lateral offsets, evenly spaced), and we find the
/// cheapest closed path through the lattice. The cost if a station is the
/// time to drive its segment at the cornering-limited speed, which needs the
/// curvature there, ehich needs three consecutive points. So the DP state is a PAIR of
/// lateral positions (previous, current), and each step costs O(M^3).
/// 
/// The answer is globally optimal for that cost on that lattice. The cost
/// ignores acceleration and braking entirely, so this is the exact answer to
/// an approximate question. That trade is the point of the milestone.
/// </summary>
public static class LatticeSearch
{
    /// <param name="LaneSpacing">Metres between neighbouring lanes.</param>
    /// <param name="Stride">
    /// Run the DP on every Stride-th station, then smooth back to every
    /// station with a cubic B-Sline. A 1 m lane change spread over 20 m is a
    /// gentle curve; the same change over 5 m is a kink.
    /// </param>
    public sealed record Options(double Margin = 1.0, double LaneSpacing = 1.0, int Stride = 4);

    public static LineSolution Solve(Track track, Vehicle vehicle, Options? options = null)
    {
        options ??= new Options();
        var basis = new BSplineBasis(track.Count, options.Stride);
        var coarse = Subsample(track, basis);
        var solution = SolveLattice(coarse, vehicle, options);

        // The coarse lane choices become B-spline control values. Each
        // control sits on its own coarse station, so the smoothed line
        // follows the DP's choices without its corners.
        var offsets = new double[track.Count];
        basis.Evaluate(solution.Offsets.AsSpan(), offsets);
        new OffsetBounds(track, options.Margin).Project(offsets);

        return solution with { Offsets = ImmutableArray.Create(offsets) };
    }

    /// <summary> The stations at the B-Spline control points, as a track of their own.</summary>
    private static Track Subsample(Track track, BSplineBasis basis)
    {
        var picks = Enumerable.Range(0, basis.Count).Select(basis.CentreStation).ToArray();

        return new Track(
            track.Name,
            [.. picks.Select(i => track.Centre[i])],
            [.. picks.Select(i => track.WidthRight[i])],
            [.. picks.Select(i => track.WidthLeft[i])]);
    }

    private static LineSolution SolveLattice(Track track, Vehicle vehicle, Options options)
    {
        var n = track.Count;
        var bounds = new OffsetBounds(track, options.Margin);
        var lanes = Lanes(bounds, options.LaneSpacing);
        var m = lanes.Length;

        if (m > 255)
        {
            throw new ArgumentOutOfRangeException(nameof(options), "Too many lanes; increase LaneSpacing.");
        }

        var cost = StationCosts(track, vehicle, bounds, lanes);

        // Try every closing pair (j0, j1) for stations 0 and 1. Each run is
        // independent, so they go in parallel and each writes only it's own
        // slots in `totals`: no locks, no shared mutable state.
        var totals = new double[m * m];
        Parallel.For(
            0,
            m * m,
            localInit: () => new Buffers(n, m),
            body: (pair, _, buffers) =>
            {
                totals[pair] = Run(pair / m, pair % m, n, m, cost, buffers, path: null);
                return buffers;
            },
            localFinally: _ => { });

        var best = 0;
        for (var pair = 1; pair < totals.Length; pair++)
        {
            if (totals[pair] < totals[best])
            {
                best = pair;
            }
        }

        // Re-run the winner once more, this time recording the path.
        var indices = new int[n];
        Run(best / m, best % m, n, m, cost, new Buffers(n, m), indices);

        var offsets = new double[n];
        for (var i = 0; i < n; i++)
        {
            offsets[i] = lanes[indices[i]];
        }

        return new LineSolution(ImmutableArray.Create(offsets), Iterations: m * m, Objective: totals[best]);
    }

    /// <summary>
    /// Lanes at fixed offsets, the same at every station, spanning the widest
    /// part of the track. A lane is only usable at statios where it lies
    /// inside the bounds. Fixed offsets matter: if lanes were spread between
    /// each station's own edges, "stay in lane 3" would wiggle every time the
    /// track width changed, and the DP would pay for wiggles it never chose.
    /// </summary>
    private static double[] Lanes(OffsetBounds bounds, double spacing)
    {
        var low = bounds.Lower.Min();
        var high = bounds.Upper.Max();
        var count = (int)Math.Floor((high - low) / spacing) + 1;
        var lanes = new double[count];

        for (var j = 0; j < count; j++)
        {
            lanes[j] = low + j * spacing;
        }

        return lanes;
    }

    /// <summary>
    /// cost[((i * m + a) * m + b) * m + c] = time for segment i -> i+1 when
    /// station i-1 is in lane a, i in lane b, and i+1 in lane c; infinity if
    /// any of those lanes is off the track. Precomputed once, so every DP run
    /// is pure table lookups and additions.
    /// </summary>
    private static double[] StationCosts(Track track, Vehicle vehicle, OffsetBounds bounds, double[] lanes)
    {
        var n = track.Count;
        var m = lanes.Length;
        var cost = new double[n * m * m * m];
        var centre = track.Centre;
        var normal = track.Normal;
        var grip = vehicle.Grip;

        Parallel.For(0, n, i =>
        {
            var prev = i == 0 ? n - 1 : i - 1;
            var next = i == n - 1 ? 0 : i + 1;

            for (var a = 0; a < m; a++)
            {
                var pa = centre[prev] + lanes[a] * normal[prev];

                for (var b = 0; b < m; b++)
                {
                    var pb = centre[i] + lanes[b] * normal[i];

                    for (var c = 0; c < m; c++)
                    {
                        var index = ((i * m + a) * m + b) * m + c;

                        if (!Inside(prev, a) || !Inside(i, b) || !Inside(next, c))
                        {
                            cost[index] = double.PositiveInfinity;
                            continue;
                        }

                        var pc = centre[next] + lanes[c] * normal[next];
                        var kappa = MengerCurvature(pa, pb, pc);
                        var speed = grip.CorneringSpeed(Math.Abs(kappa), vehicle.TopSpeed);

                        cost[index] = Vec2.Distance(pb, pc) / speed;
                    }
                }
            }
        });

        return cost;

        bool Inside(int station, int lane) =>
            lanes[lane] >= bounds.Lower[station] - 1e-9 && lanes[lane] <= bounds.Upper[station] + 1e-9;
    }

    /// <summary>
    /// The DP's cost function evaluated on any line, at every station: the
    /// time to drive it if speed only ever depended on local curvature.
    /// Comparing this with the real lap time shows how far the proxy is from
    /// the truth for a given line.
    /// </summary>
    public static double ProxyTime(Track track, Vehicle vehicle, ReadOnlySpan<double> offsets)
    {
        var n = track.Count;
        var points = new Vec2[n];
        track.PointsAt(offsets, points);

        var total = 0.0;
        for (var i = 0; i < n; i++)
        {
            var a = points[i == 0 ? n - 1 : i - 1];
            var b = points[i];
            var c = points[i == n - 1 ? 0 : i + 1];
            var speed = vehicle.Grip.CorneringSpeed(Math.Abs(MengerCurvature(a, b, c)), vehicle.TopSpeed);

            total += Vec2.Distance(b, c) / speed;
        }

        return total;
    }

    /// <summary>
    /// Curvature of the circle through three points: 2 sin(angle) / chord,
    /// written as 2 * cross / (|ab| |bc| |ca|). Signed, potitive turning left.
    /// </summary>
    public static double MengerCurvature(Vec2 a, Vec2 b, Vec2 c)
    {
        var ab = b - a;
        var bc = c - b;
        var denominator = ab.Length * bc.Length * (c - a).Length;

        return denominator > 0 ? 2.0 * Vec2.Cross(ab, bc) / denominator : 0.0;
    }

    private sealed class Buffers(int n, int m)
    {
        public double[] Current { get; set; } = new double[m * m];
        public double[] Next { get; set; } = new double[m * m];

        /// <summary>back[(i * m + b) * m + c] = best a for state (b, c) at station i</summary>
        public byte[] Back { get; } = new byte[n * m * m];
    }

    /// <summary>
    /// One DP pass with stations 0 and 1 fixed at (j0, j1). State after
    /// processing station i is (position at i, position at i+1).
    /// Returns the total lap cost; fills <paramref name="path"/> if given.
    /// </summary>
    private static double Run(int j0, int j1, int n, int m, double[] cost, Buffers buffers, int[]? path)
    {
        var current = buffers.Current;
        Array.Fill(current, double.PositiveInfinity);
        current[j0 * m + j1] = 0.0;

        // Stations 1 .. n-2: extend every reachable (a, b) by every c.
        for (var i = 1; i <= n - 2; i++)
        {
            var next = buffers.Next;
            Array.Fill(next, double.PositiveInfinity);
            var stationBase = i * m * m * m;

            for (var a = 0; a < m; a++)
            {
                for (var b = 0; b < m; b++)
                {
                    var soFar = current[a * m + b];

                    if (double.IsPositiveInfinity(soFar))
                    {
                        continue;
                    }

                    var row = stationBase + (a * m + b) * m;
                    for (var c = 0; c < m; c++)
                    {
                        var total = soFar + cost[row + c];
                        if (total < next[b * m + c])
                        {
                            next[b * m + c] = total;
                            buffers.Back[(i * m + b) * m + c] = (byte)a;
                        }
                    }
                }
            }

            (buffers.Current, buffers.Next) = (next, current);
            current = buffers.Current;
        }

        // Close the loop: station n-1 looks ahead to j0, station 0 to j1.
        var best = double.PositiveInfinity;
        int bestA = 0, bestB = 0;

        for (var a = 0; a < m; a++)
        {
            for (var b = 0; b < m; b++)
            {
                var soFar = current[a * m + b];
                if (double.IsPositiveInfinity(soFar))
                {
                    continue;
                }

                var total = soFar
                    + cost[(((n - 1) * m + a) * m + b) * m + j0]
                    + cost[((0 * m + b) * m + j0) * m + j1];

                if (total < best)
                {
                    best = total;
                    bestA = a;
                    bestB = b;
                }
            }
        }

        if (path is not null)
        {
            path[0] = j0;
            path[1] = j1;
            path[n - 2] = bestA;
            path[n - 1] = bestB;

            for (var i = n - 2; i >= 2; i--)
            {
                path[i - 1] = buffers.Back[(i * m + path[i]) * m + path[i + 1]];
            }
        }

        return best;
    }
}
