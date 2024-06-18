using System.Text.Json.Serialization;

namespace geozilla_api.Endpoints.GeneratorGeoJson.Requests;

public class GenerateGeoJsonRequest
{
    [JsonPropertyName("latitudeNW")]
    public float LatitudeNW { get; set; }

    [JsonPropertyName("longitudeNW")]
    public float LongitudeNW { get; set; }

    [JsonPropertyName("latitudeSE")]
    public float LatitudeSE { get; set; }

    [JsonPropertyName("longitudeSE")]
    public float LongitudeSE { get; set; }

    [JsonPropertyName("file")]
    public required IFormFile File { get; set; }
}
