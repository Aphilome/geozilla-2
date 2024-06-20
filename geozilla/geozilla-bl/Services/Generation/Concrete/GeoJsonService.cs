using geozilla_bl.Services.Generation.Abstract;
using System.Runtime.InteropServices;

namespace geozilla_bl.Services.Generation.Concrete;

public class GeoJsonService: IGeoJsonService
{
    public async Task<string> Generate(string path, float latitude, float longitude)
    {
        IntPtr geoJsonPtr = GeozillaCoreDll.GenerateGeoJson(path, latitude, longitude);
        string? geoJson = Marshal.PtrToStringAnsi(geoJsonPtr);
        GeozillaCoreDll.FreeBuffer(geoJsonPtr);
        return await Task.FromResult(geoJson ?? "{}");
    }
}
