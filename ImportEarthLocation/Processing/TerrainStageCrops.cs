using System;
using System.Collections.Generic;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

// Small height crops at fixed sites after each source-grid terrain stage (report 121 §17),
// so the first stage that loses a ridge, terrace or shore can be found. Diagnostics only.
//
// The source grid is the elevation tiles' own: Web Mercator pixels at the tile zoom, row
// zero north. Each record names its stage and site and gives the crop's global pixel
// origin and zoom, so it can be laid over the later output-grid maps (offline design input,
// final design, native ground) with GeoMath. Heights are centimetres, rounded; a sample
// that is not finite is written as null. Sites are the map centre and four points a
// quarter of the source footprint north, east, south and west of it; each crop is
// CropSize samples square (about 1.8 km at zoom 13), clipped to the grid.
public sealed class TerrainStageCrops
{
    public const int CropSize = 128;

    private readonly ImportLog? _log;
    private readonly int _width, _height, _originX, _originY, _zoom;
    private readonly (string Name, int X, int Y)[] _sites;
    private readonly List<object> _records = new();

    public TerrainStageCrops(ImportLog? log, int width, int height, int originX, int originY, int zoom)
    {
        _log = log?.DiagnosticsEnabled == true ? log : null;
        _width = width;
        _height = height;
        _originX = originX;
        _originY = originY;
        _zoom = zoom;
        var centreX = width / 2;
        var centreY = height / 2;

        _sites =
            new[]
            {
                ("centre", centreX, centreY),
                ("north", centreX, centreY - height / 4),
                ("east", centreX + width / 4, centreY),
                ("south", centreX, centreY + height / 4),
                ("west", centreX - width / 4, centreY)
            };
    }

    public void Capture(string stage, float[] grid)
    {
        if (_log == null || grid.Length != _width * _height)
            return;

        foreach (var (name, x, y) in _sites)
        {
            var left = Math.Max(0, Math.Min(_width - CropSize, x - CropSize / 2));
            var top = Math.Max(0, Math.Min(_height - CropSize, y - CropSize / 2));
            var columns = Math.Min(CropSize, _width);
            var rows = Math.Min(CropSize, _height);
            var heights = new int?[columns * rows];

            for (var row = 0; row < rows; row++)
                for (var column = 0; column < columns; column++)
                {
                    var value = grid[(top + row) * _width + left + column];
                    heights[row * columns + column] =
                        float.IsNaN(value) || float.IsInfinity(value) ? null : (int)Math.Round(value * 100);
                }

            _records.Add(
                new
                {
                    stage,
                    site = name,
                    zoom = _zoom,
                    globalPixelX = _originX + left,
                    globalPixelY = _originY + top,
                    columns,
                    rows,
                    rowZero = "north",
                    heightsCentimetres = heights
                });
        }
    }

    // Writes every stage captured so far as one gzip ndjson artifact.
    public void Write()
    {
        if (_log != null && _records.Count > 0)
            _log.DiagnosticCompressedRecords("terrain-stage-crops.ndjson.gz", _records);
    }
}
