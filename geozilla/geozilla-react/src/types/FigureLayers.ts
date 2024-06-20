import L from "leaflet";

interface FigureLayers {
    grassLayer: L.FeatureGroup;
    roadLayer: L.FeatureGroup;
    sidewalkLayer: L.FeatureGroup;
    buildingLayer: L.FeatureGroup;
    obstaclesLayer: L.FeatureGroup;
    defaultLayer: L.FeatureGroup;
}

export default FigureLayers;