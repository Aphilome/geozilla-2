using B3dmCore;

namespace geozilla_bl.Services.Visualizer;

public class Visualizer: IVisualizer
{
    public async Task<byte[]> ConvertToGltf(string b3dmFilePath)
    {
        using var inputStream = File.OpenRead(b3dmFilePath);
        var b3dm = B3dmReader.ReadB3dm(inputStream);
        using var outputStream = new MemoryStream(b3dm.GlbData);

        return outputStream.ToArray();
    }
}
