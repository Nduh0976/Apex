using System.Collections.Immutable;
using Apex.Geometry;

namespace Apex.Tracks;

/// <summary>
/// Synthetic tracks with known answers. These are what the tests run on:
/// a real circuit tells you whether the output looks plausible, a circle
/// tells you whether it is right
/// </summary>
public static class TrackFactory
{
    /// <summary>An anticlockwise circle of the given centreline radius</summary>
    public static Track Circle(double radius, double halfWidth, int count)
    {
        var centre = ImmutableArray.CreateBuilder<Vec2>(count);

        for (var i = 0; i < count; i++)
        {
            var angle = 2.0 * Math.PI * i / count;
            centre.Add(new Vec2(radius * Math.Cos(angle), radius * Math.Sin(angle)));
        }
        
        var widths = ImmutableArray.CreateRange(Enumerable.Repeat(halfWidth, count));
        return new Track($"Circle R{radius}", centre.MoveToImmutable(), widths, widths);
    }

    /// <summary>
    /// A stadium: two straights joined by two semicircles, anticlockwise.
    /// Stations are spaced approximately <paramref name="spacing"/> metres apart.
    /// </summary>
    public static Track Stadium(double straightLength, double radius, double halfWidth, double spacing)
    {
        var points = new List<Vec2>();
        var straightSteps = Math.Max(1, (int)Math.Round(straightLength / spacing));
        var arcSteps = Math.Max(2, (int)Math.Round(Math.PI * radius / spacing));
        var half = straightLength / 2;

        // Bottom straight, left to right, at y = -radius.
        for (var i = 0; i < straightSteps; i++)
        {
            points.Add(new Vec2(-half + straightLength * i / straightSteps, -radius));
        }

        // Right semicircle, from bottom to top.
        for (var i = 0; i < arcSteps; i++)
        {
            var angle = -Math.PI / 2 + Math.PI * i / arcSteps;
            points.Add(new Vec2(half + radius * Math.Cos(angle), radius * Math.Sin(angle)));
        }

        // Top straight, right to left.
        for (var i = 0; i < straightSteps; i++)
        {
            points.Add(new Vec2(half - straightLength * i / straightSteps, radius));
        }

        // Left semicircle, from top to bottom.
        for (var i = 0; i < arcSteps; i++)
        {
            var angle = Math.PI / 2 + Math.PI * i / arcSteps;
            points.Add(new Vec2(-half + radius * Math.Cos(angle), radius * Math.Sin(angle)));
        }

        var widths = ImmutableArray.CreateRange(Enumerable.Repeat(halfWidth, points.Count));
        return new Track("Stadium", [.. points], widths, widths);
    }
}
