// Usage: Apex.Cli [track.csv] [--grip "<formula in v>"] [--lattace]
// With no track selected, run Monza from the repo's tracks folder

using System.Diagnostics;
using System.Globalization;
using Apex.MathEngine;
using Apex.Optimisation;
using Apex.Output;
using Apex.Simulation;
using Apex.Tracks;
using Apex.Vehicles;

string trackPath;

if (args.Length > 0 && !args[0].StartsWith("--", StringComparison.Ordinal))
{
    trackPath = args[0];
}
else
{
    var root = FindRepoRoot();

    if (root is null)
    {
        Console.Error.WriteLine("usage: Apex.Cli [track.csv] [--grip \"<formula in v>\"] [--lattace]");
        Console.Error.WriteLine("No track given, and no tracks/Monza.csv found above" + AppContext.BaseDirectory);

        return 1;
    }

    // An IDE starts the app in bin/Debug/net10.0. Moving to the repo root
    // makes the default run behave exactly like `dotnet run` from the root:
    // the track resolves from there and out/ is written there.
    Directory.SetCurrentDirectory(root);
    trackPath = Path.Combine("tracks", "Monza.csv");
    Console.WriteLine($"No track given; using {Path.GetFullPath(trackPath)}");
}

var track = TrackLoader.LoadCsv(trackPath).Resample(spacing: 5.0);
var vehicle = Vehicle.TrackCar();

var gripIndex = Array.IndexOf(args, "--grip");
if (gripIndex >= 0 && gripIndex + 1 < args.Length)
{
    var constants = new Dictionary<string, double>
    {
        ["mu"] = 1.3,
        ["g"] = 9.81,
        ["rho"] = 1.2,
        ["cla"] = 2.5,
        ["m"] = vehicle.Mass
    };

    vehicle = vehicle with { Grip = ExpressionGrip.FromFormula(args[gripIndex + 1], constants, longitudinalRatio: 1.2 / 1.3) };
    Console.WriteLine($"Grip: {args[gripIndex + 1]}");
}

var simulator = new LapSimulator(vehicle);
Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"Track: {track.Name}   length {track.Length:N0} m stations {track.Count}   spacing {track.Length / track.Count:F1} m"));
Console.WriteLine();
Console.WriteLine($"{"Line",-18}{"Lap time",10}{"Max |k| (1/m)",16}{"Solve (ms)",12}");

var stopwatch = Stopwatch.StartNew();
Report("Centreline", new double[track.Count]);

stopwatch.Restart();
var minimumCurvature = MinimumCurvature.Solve(track);
Report("Min curvature", minimumCurvature.Offsets.AsSpan());

stopwatch.Restart();
var minimumTime = MinimumTime.Improve(track, simulator, minimumCurvature.Offsets.AsSpan());
var best = Report("Min time", minimumTime.Offsets.AsSpan());

if (args.Contains("--lattice"))
{
    stopwatch.Restart();
    var lattice = LatticeSearch.Solve(track, vehicle);
    Report("Lattice (DP)", lattice.Offsets.AsSpan());
}

Directory.CreateDirectory("out");
var svgPath = Path.Combine("out", $"{track.Name}.svg");
File.WriteAllText(svgPath, SvgWriter.Render(track, minimumTime.Offsets.AsSpan(), best, $"{track.Name}, minimum-time line"));
Console.WriteLine();
Console.WriteLine($"Wrote {svgPath}");

return 0;

LapResult Report(string name, ReadOnlySpan<double> offsets)
{
    var elapsed = stopwatch.ElapsedMilliseconds;
    var points = new Apex.Geometry.Vec2[track.Count];
    track.PointsAt(offsets, points);
    var lap = simulator.Simulate(points);
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{name,-18}{SvgWriter.FormatLapTime(lap.LapTime),10}{lap.MaxAbsCurvature,16:F4}{elapsed,12}"));

    return lap;
}

// Walk up from the executable until a folder containing tracks/Monza.csv appears.
static string? FindRepoRoot()
{
    for (var dir = new DirectoryInfo(AppContext.BaseDirectory); dir is not null; dir = dir.Parent)
    {
        if (File.Exists(Path.Combine(dir.FullName, "tracks", "Monza.csv")))
        {
            return dir.FullName;
        }
    }

    return null;
}
