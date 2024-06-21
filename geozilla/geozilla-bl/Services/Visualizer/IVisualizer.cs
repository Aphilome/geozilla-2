namespace geozilla_bl.Services.Visualizer;

public interface IVisualizer
{
    Task<byte[]> ConvertToGltf(string b3dmFilePath);
}
