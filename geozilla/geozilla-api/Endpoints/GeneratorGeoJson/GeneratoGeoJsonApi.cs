﻿using B3dmCore;
using geozilla_api.Endpoints.GeneratorGeoJson.Requests;
using geozilla_bl.Services.Generation.Abstract;
using Microsoft.AspNetCore.Mvc;

namespace geozilla_api.Endpoints.GeneratorGeoJson;

public static class GeneratoGeoJsonApi
{
    private static string TempUploadedFileDirectory = "Temp";

    public static IEndpointRouteBuilder AddGeneratoGeoJsonApi(this IEndpointRouteBuilder builder)
    {
        var info = new DirectoryInfo(TempUploadedFileDirectory);
        if (!info.Exists)
            info.Create();

        builder.MapPost("generate/geo-json", GenerateGeoJson)
            .WithName("GenerateGeoJson").WithOpenApi()
            .DisableAntiforgery();

        builder.MapGet("tile", GetTile)
            .WithName("GetTile").WithOpenApi()
            .CacheOutput( i => i.NoCache());

        return builder;
    }

    private static async Task<string> GenerateGeoJson([FromForm] GenerateGeoJsonRequest request, IGeoJsonService service)
    {
        var path = Path.Combine(TempUploadedFileDirectory, request.File.FileName);
        using var outputFileStream = new FileStream(path, FileMode.Create);

        var inputStream = request.File.OpenReadStream();
        inputStream.CopyTo(outputFileStream);
        inputStream.Seek(0, SeekOrigin.Begin);


        var destPath = Path.Combine(TempUploadedFileDirectory, "tile.glb");
        var b3dm = B3dmReader.ReadB3dm(inputStream);
        var stream = new MemoryStream(b3dm.GlbData);
        File.WriteAllBytes(destPath, stream.ToArray());

        var result = await service.Generate(Path.GetFullPath(path));
        //result = TestData.GeoJson;
        return result;
    }

    private static async Task<IResult> GetTile()
    {
        var path = Path.Combine(TempUploadedFileDirectory, "tile.glb");
        var fullPath = Path.GetFullPath(path);
        if (!File.Exists(fullPath))
            return Results.Ok();
        var mimeType = "model/gltf-binary";
        return Results.File(fullPath, contentType: mimeType);
    }
}

#region Test data

[Obsolete]
internal static class TestData
{
    public static string GeoJson =
        """
        {
          "type": "FeatureCollection",
          "features": [
            {
              "type": "Feature",
              "properties": {
                "zoneType": "grass"
              },
              "geometry": {
                "coordinates": [
                  [
                    [
                      47.20459054875374,
                      56.11833941183451
                    ],
                    [
                      47.22662253379352,
                      56.10579488510362
                    ],
                    [
                      47.23242274354888,
                      56.10556994636116
                    ],
                    [
                      47.21115515919604,
                      56.11850299198113
                    ],
                    [
                      47.20459054875374,
                      56.11833941183451
                    ]
                  ]
                ],
                "type": "Polygon"
              },
              "id": 1
            },
            {
              "type": "Feature",
              "properties": {},
              "geometry": {
                "coordinates": [
                  [
                    [
                      47.27415804422614,
                      56.124659500011205
                    ],
                    [
                      47.29740956421716,
                      56.102884766912666
                    ],
                    [
                      47.30223153713774,
                      56.10412865980092
                    ],
                    [
                      47.277643346629645,
                      56.1253210496549
                    ],
                    [
                      47.27415804422614,
                      56.124659500011205
                    ]
                  ]
                ],
                "type": "Polygon"
              }
            },
            {
              "type": "Feature",
              "properties": {},
              "geometry": {
                "coordinates": [
                  [
                    [
                      47.27408806397847,
                      56.10309732982137
                    ],
                    [
                      47.27869167657974,
                      56.103054995777114
                    ],
                    [
                      47.30950041261386,
                      56.12461871520185
                    ],
                    [
                      47.30585509466786,
                      56.126107780535875
                    ],
                    [
                      47.27408806397847,
                      56.10309732982137
                    ]
                  ]
                ],
                "type": "Polygon"
              }
            },
            {
              "type": "Feature",
              "properties": {
                "zoneType": "grass"
              },
              "geometry": {
                "type": "Polygon",
                "coordinates": [
                  [
                    [
                      47.25161532536433,
                      56.156024163758495
                    ],
                    [
                      47.244233358895904,
                      56.15582196939922
                    ],
                    [
                      47.23692271590054,
                      56.15521733991222
                    ],
                    [
                      47.229754024050926,
                      56.15421611701072
                    ],
                    [
                      47.22279652646557,
                      56.152827973545655
                    ],
                    [
                      47.21611740696697,
                      56.15106631917249
                    ],
                    [
                      47.20978113615529,
                      56.14894816962383
                    ],
                    [
                      47.2038488448625,
                      56.146493980908176
                    ],
                    [
                      47.198377731238864,
                      56.14372745010655
                    ],
                    [
                      47.19342050734325,
                      56.14067528477052
                    ],
                    [
                      47.18902489066348,
                      56.13736694323483
                    ],
                    [
                      47.18523314549465,
                      56.133834348441496
                    ],
                    [
                      47.182081678553594,
                      56.13011157812668
                    ],
                    [
                      47.179600692619765,
                      56.12623453444585
                    ],
                    [
                      47.17781390137147,
                      56.12224059630288
                    ],
                    [
                      47.176738307941484,
                      56.11816825780475
                    ],
                    [
                      47.17638404905563,
                      56.114056756383576
                    ],
                    [
                      47.17675430595026,
                      56.109945694211405
                    ],
                    [
                      47.177845282597104,
                      56.10587465658032
                    ],
                    [
                      47.179646251104174,
                      56.10188283093106
                    ],
                    [
                      47.18213966351605,
                      56.098008630189106
                    ],
                    [
                      47.18530132861236,
                      56.09428932400624
                    ],
                    [
                      47.18910065170373,
                      56.09076068141373
                    ],
                    [
                      47.19350093485656,
                      56.087456628267304
                    ],
                    [
                      47.19845973444362,
                      56.084408922708825
                    ],
                    [
                      47.2039292724215,
                      56.0816468516864
                    ],
                    [
                      47.209856897279984,
                      56.079196951364686
                    ],
                    [
                      47.21618559019499,
                      56.07708275402385
                    ],
                    [
                      47.22285451154743,
                      56.0753245637909
                    ],
                    [
                      47.229799582645654,
                      56.073939263272955
                    ],
                    [
                      47.23695409721062,
                      56.07294015287219
                    ],
                    [
                      47.24424935695037,
                      56.07233682425689
                    ],
                    [
                      47.25161532536433,
                      56.07213506914826
                    ],
                    [
                      47.258981293778284,
                      56.07233682425689
                    ],
                    [
                      47.26627655351803,
                      56.07294015287219
                    ],
                    [
                      47.273431068083,
                      56.073939263272955
                    ],
                    [
                      47.28037613918122,
                      56.0753245637909
                    ],
                    [
                      47.287045060533664,
                      56.07708275402385
                    ],
                    [
                      47.29337375344866,
                      56.079196951364686
                    ],
                    [
                      47.29930137830714,
                      56.0816468516864
                    ],
                    [
                      47.30477091628503,
                      56.084408922708825
                    ],
                    [
                      47.309729715872095,
                      56.087456628267304
                    ],
                    [
                      47.31412999902492,
                      56.09076068141373
                    ],
                    [
                      47.31792932211629,
                      56.09428932400624
                    ],
                    [
                      47.3210909872126,
                      56.098008630189106
                    ],
                    [
                      47.32358439962448,
                      56.10188283093106
                    ],
                    [
                      47.32538536813154,
                      56.10587465658032
                    ],
                    [
                      47.32647634477839,
                      56.109945694211405
                    ],
                    [
                      47.326846601673026,
                      56.114056756383576
                    ],
                    [
                      47.32649234278717,
                      56.11816825780475
                    ],
                    [
                      47.32541674935718,
                      56.12224059630288
                    ],
                    [
                      47.32362995810889,
                      56.12623453444585
                    ],
                    [
                      47.32114897217505,
                      56.13011157812668
                    ],
                    [
                      47.31799750523399,
                      56.133834348441496
                    ],
                    [
                      47.31420576006517,
                      56.13736694323483
                    ],
                    [
                      47.309810143385405,
                      56.14067528477052
                    ],
                    [
                      47.304852919489775,
                      56.14372745010655
                    ],
                    [
                      47.299381805866155,
                      56.146493980908176
                    ],
                    [
                      47.29344951457335,
                      56.14894816962383
                    ],
                    [
                      47.28711324376168,
                      56.15106631917249
                    ],
                    [
                      47.28043412426308,
                      56.152827973545655
                    ],
                    [
                      47.27347662667771,
                      56.15421611701072
                    ],
                    [
                      47.2663079348281,
                      56.15521733991222
                    ],
                    [
                      47.25899729183274,
                      56.15582196939922
                    ],
                    [
                      47.25161532536433,
                      56.156024163758495
                    ]
                  ]
                ]
              },
              "id": 8
            }
          ]
        }
        """;
}

#endregion
