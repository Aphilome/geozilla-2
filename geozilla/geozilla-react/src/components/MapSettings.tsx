import {useMap} from "react-leaflet";
import React, {useEffect} from "react";
import L from "leaflet";
import Layers from "../types/Layers";

interface MapSettingsProps {
    layerControlRef:  React.MutableRefObject<L.Control.Layers | null>;
    layersRef:  React.MutableRefObject<Layers>;
}

const MapSettings: React.FC<MapSettingsProps> = ({ layerControlRef, layersRef}) => {
    const map = useMap();

    useEffect(() => {
            map.pm.addControls({
                position: 'topleft',
                drawCircleMarker: false,
                rotateMode: false,
                drawText: false,
            });

            if (layerControlRef.current === null) {
                layerControlRef.current = L.control.layers({}, {
                    'Grass': layersRef.current.grassLayerRef.current,
                    'Road': layersRef.current.roadLayerRef.current,
                    'Sidewalk': layersRef.current.sidewalkLayerRef.current,
                    'Building': layersRef.current.buildingLayerRef.current,
                    'Default': layersRef.current.defaultLayerRef.current


                }, { collapsed: false }).addTo(map);

                map.addLayer(layersRef.current.grassLayerRef.current);
                map.addLayer(layersRef.current.roadLayerRef.current);
                map.addLayer(layersRef.current.sidewalkLayerRef.current);
                map.addLayer(layersRef.current.buildingLayerRef.current);
                map.addLayer(layersRef.current.defaultLayerRef.current);
            }
        // }
    }, [map, layersRef, layerControlRef]);

    return null;
}

export default MapSettings;