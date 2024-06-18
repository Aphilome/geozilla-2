import React from "react";
import L from "leaflet";

interface Layers {
    grassLayerRef: React.MutableRefObject<L.FeatureGroup<any>>;
    roadLayerRef: React.MutableRefObject<L.FeatureGroup<any>>;
    sidewalkLayerRef: React.MutableRefObject<L.FeatureGroup<any>>;
    buildingLayerRef: React.MutableRefObject<L.FeatureGroup<any>>;
    obstaclesLayerRef: React.MutableRefObject<L.FeatureGroup<any>>;
    defaultLayerRef: React.MutableRefObject<L.FeatureGroup<any>>;
}

export default Layers;