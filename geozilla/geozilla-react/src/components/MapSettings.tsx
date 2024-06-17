import {useMap} from "react-leaflet";
import React, {useEffect} from "react";
import L from "leaflet";

interface MapSettingsProps {
    mapRef:  React.MutableRefObject<L.Map | null>;
    layerControlRef:  React.MutableRefObject<L.Control.Layers | null>;

    grassLayerRef:  React.MutableRefObject<L.FeatureGroup<any>>;
    // obstacleLayerRef
    roadLayerRef:  React.MutableRefObject<L.FeatureGroup<any>>;
    sidewalkLayerRef:  React.MutableRefObject<L.FeatureGroup<any>>;
    buildingLayerRef:  React.MutableRefObject<L.FeatureGroup<any>>;
    defaultLayerRef:  React.MutableRefObject<L.FeatureGroup<any>>;
}

const MapSettings: React.FC<MapSettingsProps> = ({ mapRef , layerControlRef, grassLayerRef, roadLayerRef, sidewalkLayerRef, buildingLayerRef, defaultLayerRef}) => {
    const map = useMap();
    useEffect(() => {
        if (!mapRef.current) {
            mapRef.current = map;

            map.pm.addControls({
                position: 'topleft',
                drawCircleMarker: false,
                rotateMode: false,
                drawText: false,
            });

            if (layerControlRef.current === null) {
                layerControlRef.current = L.control.layers({}, {
                    'Grass': grassLayerRef.current,
                    'Road': roadLayerRef.current,
                    'Sidewalk': sidewalkLayerRef.current,
                    'Building': buildingLayerRef.current,
                    'Default': defaultLayerRef.current


                }, { collapsed: false }).addTo(map);

                map.addLayer(grassLayerRef.current);
                map.addLayer(roadLayerRef.current);
                map.addLayer(sidewalkLayerRef.current);
                map.addLayer(buildingLayerRef.current);
                map.addLayer(defaultLayerRef.current);
            }
        }
    }, [map, grassLayerRef, roadLayerRef, sidewalkLayerRef, buildingLayerRef, defaultLayerRef, layerControlRef, mapRef]);

    return null;
}

export default MapSettings;