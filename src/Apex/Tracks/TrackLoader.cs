using System.Collections.Immutable;
using System.Globalization;
using Apex.Geometry;

namespace Apex.Tracks;

/// <summary>
/// Reads the TUM racetrack-database CSV format:
///     # x_m,y_m,w_tr_right_m,w_tr_left_m
/// Widths are measured from the centreline to the track edge, right and left
/// relative to the direction of travel.
/// </summary>
public static class TrackLoader
{
    public static Track LoadCsv(string path, string? name = null)
    {
        var centre = ImmutableArray.CreateBuilder<Vec2>();
        var right = ImmutableArray.CreateBuilder<double>();
        var left = ImmutableArray.CreateBuilder<double>();

        var lineNumber = 0;
        foreach (var raw in File.ReadLines(path))
        {
            lineNumber++;
            var line = raw.Trim();

            if (line.Length == 0 || line.StartsWith('#'))
            {
                continue;
            }

            var parts = line.Split(',');
            if (parts.Length < 4)
            {
                throw new FormatException($"{path}:{lineNumber}: expected 4 columns, found {parts.Length}.");
            }

            // Invariant culture: the file format fixes '.' as the decimal
            // separator, whatever the machine's locale says.

            centre.Add(new Vec2(Parse(parts[0]), Parse(parts[1])));
            right.Add(Parse(parts[2]));
            left.Add(Parse(parts[3]));

            double Parse(string s) =>
                double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var value)
                    ? value
                    : throw new FormatException($"{path}:{lineNumber}: '{s}' is not a number.");

            // Many published tracks repeat the first point at the end to close the loop
            // Our model is implicitly closed, so a duplicate would be a zero-length segment

            if (centre.Count > 1 && Vec2.Distance(centre[0], centre[^1]) < 1e-6)
            {
                centre.RemoveAt(centre.Count - 1);
                right.RemoveAt(right.Count - 1);
                left.RemoveAt(left.Count - 1);
            }
        }

        return new Track(
            name ?? Path.GetFileNameWithoutExtension(path),
            centre.ToImmutable(),
            right.ToImmutable(),
            left.ToImmutable());
    }
}
