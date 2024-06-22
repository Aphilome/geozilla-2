using System.Text.Json.Serialization;

namespace geozilla_api.Endpoints.GeneratorGeoJson.Requests;

public class GenerateGeoJsonRequest
{
    [JsonPropertyName("file")]
    public required IFormFile File { get; set; }
}
