import {useMap} from "react-leaflet";
import React, {useEffect} from "react";
import L, {Map} from "leaflet";
import Layers from "../types/Layers";

interface MapSettingsProps {
    layerControlRef:  React.MutableRefObject<L.Control.Layers | null>;
    layersRef:  React.MutableRefObject<Layers>;
    mapRef: React.MutableRefObject<Map | undefined>;
}

const MapSettings: React.FC<MapSettingsProps> = ({ layerControlRef, layersRef, mapRef}) => {
    console.log('MapSettings component');
    const map = useMap();

    useEffect(() => {
        if (!mapRef.current){
            mapRef.current = map;
        }
        mapRef.current.pm.addControls({
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

            mapRef.current.addLayer(layersRef.current.grassLayerRef.current);
            mapRef.current.addLayer(layersRef.current.roadLayerRef.current);
            mapRef.current.addLayer(layersRef.current.sidewalkLayerRef.current);
            mapRef.current.addLayer(layersRef.current.buildingLayerRef.current);
            mapRef.current.addLayer(layersRef.current.defaultLayerRef.current);
        }
    }, [mapRef.current, layersRef, layerControlRef]);

    return null;
}

export default MapSettings;