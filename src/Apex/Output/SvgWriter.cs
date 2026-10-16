using System.Globalization;
using System.Text;
using Apex.Geometry;
using Apex.Simulation;
using Apex.Tracks;

namespace Apex.Output;

/// <summary>
/// Writes a track map with the racing line coloured by speed: blue where the
/// car is slowest, red wherre it is fastest.
/// </summary>
public static class SvgWriter
{
    private const double CanvasWidth = 1000;
    private const double Padding = 20;

    public static string Render(Track track, ReadOnlySpan<double> offsets, LapResult lap, string title)
    {
        var n = track.Count;
        var line = new Vec2[n];
        var left = new Vec2[n];
        var right = new Vec2[n];
        track.PointsAt(offsets, line);

        for (var i = 0; i < n; i++)
        {
            left[i] = track.Centre[i] + track.WidthLeft[i] * track.Normal[i];
            right[i] = track.Centre[i] - track.WidthRight[i] * track.Normal[i];
        }

        double minX = double.MaxValue, minY = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue;

        foreach (var p in left.Concat(right))
        {
            minX = Math.Min(minX, p.X); maxX = Math.Max(maxX, p.X);
            minY = Math.Min(minY, p.Y); maxY = Math.Max(maxY, p.Y);
        }

        var scale = (CanvasWidth - 2 * Padding) / (maxX - minX);
        var height = (maxY - minY) * scale + 2 * Padding + 40;

        // SVG's y axis points down; the track's point up. Flip it here, once.
        string X(Vec2 p) => F(Padding + (p.X - minX) * scale);
        string Y(Vec2 p) => F(Padding + 40 +(maxY - p.Y) * scale);

        var slowest = lap.Speed.Min();
        var fastest = lap.Speed.Max();

        var svg = new StringBuilder();
        svg.AppendLine($"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {F(CanvasWidth)} {F(height)}" font-family="sans-serif">""");
        svg.AppendLine($"""<rect width="100%" height="100%" fill="white"/>""");
        svg.AppendLine($"""<text x="{F(Padding)}" y="26" font-size="18">{Escape(title)}: {FormatLapTime(lap.LapTime)} ({F(slowest * 3.6)}-{F(fastest * 3.6)} km/h)</text>""");

        foreach (var edge in new[] { left, right })
        {
            var points = string.Join(" ", edge.Append(edge[0]).Select(p => $"{X(p)}, {Y(p)}"));
            svg.AppendLine($"""<polyline points="{points}" fill="none" stroke="#999" stroke-width="1"/>""");
        }

        for (var i = 0; i < n; i++)
        {
            var next = i == n - 1 ? 0 : i + 1;
            var t = (lap.Speed[i] - slowest) / Math.Max(fastest - slowest, 1e-9);
            var colour = SpeedColour(t);
            svg.AppendLine($"""<line x1="{X(line[i])}" y1="{Y(line[i])}" x2="{X(line[next])}" y2="{Y(line[next])}" stroke="{colour}" stroke-width="3" stroke-linecap="round"/>""");
        }

        svg.AppendLine("</svg>");
        return svg.ToString();
    }

    /// <summary>
    /// Hue from 240 degrees (blue, t = 0) to 0 degrees (red, t = 1) at fixed
    /// saturation and lightness, as #rrggbb. Hex rather than hsl() because
    /// not every SVG renderer supports CSS colour functions.
    /// </summary>
    private static string SpeedColour(double t)
    {
        var hue = 240.0 * (1.0 - Math.Clamp(t, 0.0, 1.0));
        const double saturation = 0.9, lightness = 0.45;
        var chroma = (1 - Math.Abs(2 * lightness - 1)) * saturation;
        var x = chroma * (1 - Math.Abs(hue / 60.0 % 2 - 1));

        var (r, g, b) = (hue / 60.0) switch
        {
            < 1 => (chroma, x, 0.0),
            < 2 => (x, chroma, 0.0),
            < 3 => (0.0, chroma, x),
            < 4 => (0.0, x, chroma),
            _ => (x, 0.0, chroma),
        };

        var m = lightness - chroma / 2;

        return $"#{Byte(r + m):x2}{Byte(g + m):x2}{Byte(b + m):x2}";

        static int Byte(double v) => (int)Math.Round(Math.Clamp(v, 0, 1) * 255);
    }

    public static string FormatLapTime(double seconds) =>
        TimeSpan.FromSeconds(seconds).ToString(@"m\:ss\.fff", CultureInfo.InvariantCulture);

    private static string F(double value) => value.ToString("0.#", CultureInfo.InvariantCulture);

    private static string Escape(string text) =>
        text.Replace("&", "&amp;").Replace("<", "&lt;").Replace(">", "&gt;");
}
