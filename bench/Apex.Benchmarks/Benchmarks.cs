using Apex.Geometry;
using Apex.Optimisation;
using Apex.Simulation;
using Apex.Tracks;
using Apex.Vehicles;
using BenchmarkDotNet.Attributes;

namespace Apex.Benchmarks;

internal static class Shared
{
    // Monza resampled to 5 m; 1,158 stations.
    public static readonly Track Monza = TrackLoader.LoadCsv(FindTrack()).Resample(5.0);

    // BenchmarkDotNet runs each benchmark in a generated child process deep
    // inside bin/, so a path relative to the working directory is fragile.
    // Walk up from the executable until tracks.Monza.csv appears.
    private static string FindTrack()
    {
        for (var dir = new DirectoryInfo(AppContext.BaseDirectory); dir is not null; dir = dir.Parent)
        {
            var candidate = Path.Combine(dir.FullName, "tracks", "Monza.csv");
            if (File.Exists(candidate))
            {
                return candidate;
            }
        }

        throw new FileNotFoundException($"tracks/Monza.csv not found above {AppContext.BaseDirectory}");
    }
}

/// <summary>Allocating convenience method vs reused workspace.</summary>
[MemoryDiagnoser]
public class LapSimulationBenchmarks
{
    private LapSimulator _simulator = null!;
    private Vec2[] _points = null!;
    private LapWorkspace _workspace = null!;

    [GlobalSetup]
    public void Setup()
    {
        _simulator = new LapSimulator(Vehicle.TrackCar());
        _points = Shared.Monza.Centre.AsSpan().ToArray();
        _workspace = new LapWorkspace(_points.Length);
        _points.CopyTo(_workspace.Points);
    }

    [Benchmark(Baseline = true)]
    public double Simulate_allocating() => _simulator.Simulate(_points).LapTime;

    [Benchmark]
    public double LapTime_reused_wprkspace() => _simulator.LapTime(_workspace);
}

/// <summary>Two cyclic solves vs one paired solve sharing the elimination.</summary>
public class TridiagonalBenchmarks
{
    private double[] _sub = null!, _diag = null!, _sup = null!, _x = null!, _y = null!;
    private double[] _a = null!, _b = null!, _scratch = null!;

    [GlobalSetup]
    public void Setup()
    {
        const int n = 1158;
        var random = new Random(42);
        _sub = new double[n]; _diag = new double[n]; _sup = new double[n];
        _x = new double[n]; _y = new double[n]; _a = new double[n]; _b = new double[n];
        _scratch = new double[3 * n];

        for (var i = 0; i < n; i++)
        {
            _sub[i] = 1 + random.NextDouble();
            _sup[i] = 1 + random.NextDouble();
            _diag[i] = 2 * (_sub[i] + _sup[i]);
            _x[i] = random.NextDouble();
            _y[i] = random.NextDouble();
        }
    }

    // Both methods copy fresh right-hand sides first: solving in place
    // repeatedly would feed each run the previous run's output.
    [Benchmark(Baseline = true)]
    public double Two_singles_solves()
    {
        _x.CopyTo(_a); _y.CopyTo(_b);
        Tridiagonal.SolveCyclic(_sub, _diag, _sup, _a, _b, _scratch);
        return _a[0] + _b[0];
    }

    [Benchmark]
    public double One_paired_solve()
    {
        _x.CopyTo(_a); _y.CopyTo(_b);
        Tridiagonal.SolveCyclic(_sub, _diag, _sup, _a, _b, _scratch);
        return _a[0] + _b[0];
    }
}

/// <summary>Scalar vs Vector&lt;double&gt; element-wise loops in the QP solver.</summary>
public class MinumumCurvatureBenchmarks
{
    [Benchmark(Baseline = true)]
    public double Scalar() => MinimumCurvature.Solve(Shared.Monza, new(Vectorised: false)).Objective;

    [Benchmark]
    public double Vectorised() => MinimumCurvature.Solve(Shared.Monza, new(Vectorised: true)).Objective;
}

/// <summary>How the lap-time gradient scales with threads.</summary>
public class GradientBenchmarks
{
    private LapSimulator _simulator = null!;
    private BSplineBasis _basis = null!;
    private double[] _baseline = null!, _controls = null!, _gradient = null!;

    [Params(1, 2, 4, 8)]
    public int Threads { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        var monza = Shared.Monza;
        _simulator = new LapSimulator(Vehicle.TrackCar());
        _basis = new BSplineBasis(monza.Count, 4);
        _baseline = MinimumCurvature.Solve(monza).Offsets.AsSpan().ToArray();
        _controls = new double[_basis.Count];
        _gradient = new double[_basis.Count];
    }

    [Benchmark]
    public double Gradient()
    {
        MinimumTime.Gradient(Shared.Monza, _simulator, _basis, _baseline, _controls, 0.05, _gradient,
            new ParallelOptions { MaxDegreeOfParallelism = Threads });

        return _gradient[0];
    }
}