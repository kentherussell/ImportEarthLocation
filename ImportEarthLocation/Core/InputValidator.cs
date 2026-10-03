using System;
using System.Collections.Generic;
using ImportEarthLocation.Geo;

namespace ImportEarthLocation.Core;

public static class InputValidator
{
    public static IReadOnlyList<string> Validate(ImportRequest request, bool mapExtDetected)
    {
        var errors = new List<string>();
        if (double.IsNaN(request.Latitude) || double.IsInfinity(request.Latitude) || request.Latitude is < -85d or > 85d)
            errors.Add("Latitude must be between -85 and 85 degrees for this MVP.");
        if (double.IsNaN(request.Longitude) || double.IsInfinity(request.Longitude) || request.Longitude is < -180d or > 180d)
            errors.Add("Longitude must be between -180 and 180 degrees.");
        if (request.MapSize.RequiresMapExt && !mapExtDetected)
            errors.Add("The selected size requires MapExt, but MapExt was not detected.");

        var bounds = GeoMath.BoundsFromCenter(request.Latitude, request.Longitude, request.MapSize.WidthMeters);
        if (bounds.CrossesDateline)
            errors.Add("Locations whose map bounds cross the international date line are not supported in the MVP.");
        return errors;
    }
}
