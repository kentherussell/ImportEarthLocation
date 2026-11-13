using System.IO.Compression;
using Unity.Mathematics;

// Plan-view PNG: terrain hillshade (optional), OSM centrelines in thin white/grey,
// finished sections as class-coloured bands at native width with their nodes.
public static class Plot
{
    public static void Run(string map, string baseDir, double lat, double lon, double half, string output, string opts)
    {
        var capture = new Cap(map);
        var data = capture.LoadOsm();
        var otherPoint = capture.Local(lat, lon);
        var sections =
            opts.Contains("input")
                ? Compare.Input(capture)
                : opts.Contains("gamefinal") ? Compare.Final(capture) : Hw.Load(Path.Combine(baseDir, map));

        var size = 1400;
        var img = new byte[size * size * 3];
        double sc = size / (2 * half);

        (int, int) ToPixel(double x, double z)
            => ((int)((x - otherPoint.x + half) * sc), (int)((half - (z - otherPoint.z)) * sc));

        void Set(int x, int y, byte r, byte g, byte b, double a = 1)
        {
            if (x < 0 || y < 0 || x >= size || y >= size)
                return;

            var k = (y * size + x) * 3;
            img[k] = (byte)(img[k] * (1 - a) + r * a);
            img[k + 1] = (byte)(img[k + 1] * (1 - a) + g * a);
            img[k + 2] = (byte)(img[k + 2] * (1 - a) + b * a);
        }

        // Background hillshade from the design terrain; water tint where below the sea surface.
        var terr =
            capture.Heights(
                opts.Contains("base")
                    ? capture.InputValue("terrainFile")
                    : capture.Has("native-observed-heightmap.u16")
                        ? "native-observed-heightmap.u16"
                        : "final-design-heightmap.u16");

        var seaS = (float?)null;

        if (opts.Contains("sea"))
            seaS = float.Parse(opts.Split("sea")[1].Split(',')[0]);

        for (var y = 0; y < size; y++)
            for (var x = 0; x < size; x++)
            {
                var wx = otherPoint.x - half + x / sc;
                var wz = otherPoint.z + half - y / sc;
                var e = 2.0 / sc + 4;
                var h = capture.HeightAt(terr, wx, wz);
                var dx = (capture.HeightAt(terr, wx + e, wz) - capture.HeightAt(terr, wx - e, wz)) / (2 * e);
                var dz = (capture.HeightAt(terr, wx, wz + e) - capture.HeightAt(terr, wx, wz - e)) / (2 * e);
                var n = math.normalize(new double3(-dx, 1, -dz));
                var s = Math.Clamp(math.dot(n, math.normalize(new double3(-1, 1.2, 1))) * .9, 0, 1);
                var v = (byte)(80 + s * 140);

                if (seaS is float sea && h < sea)
                    Set(x, y, (byte)(v * .35), (byte)(v * .55), (byte)Math.Min(255, v * 1.05));
                else
                    Set(x, y, v, v, v);
            }

        void Line(double ax, double az, double bx, double bz, byte r, byte g, byte b, double wpx, double a = 1)
        {
            var (x0, y0) = ToPixel(ax, az);
            var (x1, y1) = ToPixel(bx, bz);
            var steps = Math.Max(1, (int)Math.Ceiling(Math.Sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)) * 2));
            var radius = Math.Max(.5, wpx / 2);

            for (var i = 0; i <= steps; i++)
            {
                var px = x0 + (x1 - x0) * i / (double)steps;
                var py = y0 + (y1 - y0) * i / (double)steps;
                var ri = (int)Math.Ceiling(radius);

                for (var dy = -ri; dy <= ri; dy++)
                    for (var dx = -ri; dx <= ri; dx++)
                        if (dx * dx + dy * dy <= radius * radius + .25)
                            Mark((int)(px + dx), (int)(py + dy), r, g, b, a);
            }
        }

        var stamp = new int[size * size];
        var pass = 1;

        void Mark(int x, int y, byte r, byte g, byte b, double a)
        {
            if (x < 0 || y < 0 || x >= size || y >= size)
                return;

            if (stamp[y * size + x] == pass)
                return;

            stamp[y * size + x] = pass;
            Set(x, y, r, g, b, a);
        }

        bool Near(double x, double z)
            => Math.Abs(x - otherPoint.x) < half * 1.2 && Math.Abs(z - otherPoint.z) < half * 1.2;

        (byte, byte, byte) ClassColor(string classification) => Hw.Group(classification) switch
        {
            "motorway" => ((byte)230, (byte)60, (byte)60),
            "motorway_link" => ((byte)240, (byte)130, (byte)60),
            "trunk" => ((byte)225, (byte)110, (byte)190),
            "trunk_link" => ((byte)230, (byte)160, (byte)200),
            "primary" => ((byte)240, (byte)200, (byte)60),
            "secondary" => ((byte)120, (byte)200, (byte)80),
            "tertiary" => ((byte)90, (byte)190, (byte)210),
            _ => ((byte)200, (byte)200, (byte)230)
        };

        var show =
            sections
                .Where(section => Near(section.P0.x, section.P0.z) || Near(section.P3.x, section.P3.z))
                .Where(
                    section => !opts.Contains("hwonly")
                        || Hw.Group(section.Cls) is "motorway" or "motorway_link" or "trunk" or "trunk_link")
                .ToList();

        // Bands first, each section its own pass so translucency shows overlaps.
        foreach (var section in show.OrderBy(otherSection => otherSection.Layer))
        {
            pass++;
            var (r, g, b) = section.Kind == "Rail" ? ((byte)60, (byte)60, (byte)60) : ClassColor(section.Cls);
            var n = Math.Max(4, (int)(section.ArcLength() * sc / 4));
            var point = section.At(0);
            var a = section.Tunnel ? .25 : section.Bridge ? .75 : .55;

            for (var k = 1; k <= n; k++)
            {
                var secondPoint = section.At(k / (double)n);
                Line(point.x, point.z, secondPoint.x, secondPoint.z, r, g, b, Math.Max(1.5, section.Width * sc), a);
                point = secondPoint;
            }
        }

        // OSM polylines
        pass++;

        foreach (var way in data.Ways.Values)
        {
            if (!way.Tags.TryGetValue("highway", out var hw))
                continue;

            if (opts.Contains("hwonly") && hw is not ("motorway" or "motorway_link" or "trunk" or "trunk_link"))
                continue;

            var points =
                way.NodeIds
                    .Where(data.Nodes.ContainsKey)
                    .Select(id => capture.Local(data.Nodes[id].Latitude, data.Nodes[id].Longitude))
                    .ToArray();

            for (var j = 1; j < points.Length; j++)
                if (Near(points[j].x, points[j].z))
                    Line(points[j - 1].x, points[j - 1].z, points[j].x, points[j].z, 250, 250, 250, 1, 1);
        }

        // Section centre and nodes
        pass++;

        foreach (var section in show)
        {
            var n = Math.Max(4, (int)(section.ArcLength() * sc / 4));
            var point = section.At(0);

            for (var k = 1; k <= n; k++)
            {
                var secondPoint = section.At(k / (double)n);
                Line(point.x, point.z, secondPoint.x, secondPoint.z, 20, 20, 20, 1, 1);
                point = secondPoint;
            }
        }

        pass++;

        foreach (var section in show)
            foreach (var e in new[] { section.P0, section.P3 })
            {
                var (x, y) = ToPixel(e.x, e.z);

                for (var dy = -3; dy <= 3; dy++)
                    for (var dx = -3; dx <= 3; dx++)
                        if (dx * dx + dy * dy <= 9 && dx * dx + dy * dy >= 4)
                            Set(x + dx, y + dy, 0, 255, 255);
            }

        WritePng(output, size, size, img);
        Console.WriteLine($"plot {output}: {show.Count} sections; half {half} m; scale {sc:F3} px/m");
    }

    public static void WritePng(string path, int w, int h, byte[] rgb)
    {
        using var stream = File.Create(path);
        stream.Write(new byte[] { 137, 80, 78, 71, 13, 10, 26, 10 });

        void Chunk(string type, byte[] d)
        {
            var t = System.Text.Encoding.ASCII.GetBytes(type);
            var length = BitConverter.GetBytes(d.Length);
            Array.Reverse(length);
            stream.Write(length);
            stream.Write(t);
            stream.Write(d);
            var crc = Crc(t, d);
            var callback = BitConverter.GetBytes(crc);
            Array.Reverse(callback);
            stream.Write(callback);
        }

        var ihdr = new byte[13];
        var wb = BitConverter.GetBytes(w);
        Array.Reverse(wb);
        var hb = BitConverter.GetBytes(h);
        Array.Reverse(hb);
        wb.CopyTo(ihdr, 0);
        hb.CopyTo(ihdr, 4);
        ihdr[8] = 8;
        ihdr[9] = 2;
        Chunk("IHDR", ihdr);
        using var ms = new MemoryStream();

        using (var otherStream = new ZLibStream(ms, CompressionLevel.Fastest, true))
        {
            for (var y = 0; y < h; y++)
            {
                otherStream.WriteByte(0);
                otherStream.Write(rgb, y * w * 3, w * 3);
            }
        }

        Chunk("IDAT", ms.ToArray());
        Chunk("IEND", Array.Empty<byte>());
    }

    static uint[]? table;

    static uint Crc(byte[] a, byte[] b)
    {
        if (table == null)
        {
            table = new uint[256];

            for (uint n = 0; n < 256; n++)
            {
                var c = n;

                for (var k = 0; k < 8; k++)
                    c = (c & 1) != 0 ? 0xedb88320 ^ (c >> 1) : c >> 1;

                table[n] = c;
            }
        }

        uint r = 0xffffffff;

        foreach (var x in a)
            r = table[(r ^ x) & 255] ^ (r >> 8);

        foreach (var x in b)
            r = table[(r ^ x) & 255] ^ (r >> 8);

        return r ^ 0xffffffff;
    }
}
