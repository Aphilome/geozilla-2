import {useMap} from "react-leaflet";
import React, {useEffect} from "react";
import L, {Map} from "leaflet";
import FigureLayers from "../types/FigureLayers";

interface MapSettingsProps {
    layerControlRef:  React.MutableRefObject<L.Control.Layers | null>;
    layersRef:  React.MutableRefObject<FigureLayers>;
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
            drawMarker: false
        });

        if (layerControlRef.current === null) {
            layerControlRef.current = L.control.layers({}, {
                'Grass': layersRef.current.grassLayer,
                'Road': layersRef.current.roadLayer,
                'Sidewalk': layersRef.current.sidewalkLayer,
                'Building': layersRef.current.buildingLayer,
                'Obstacles': layersRef.current.obstaclesLayer,
                'Default': layersRef.current.defaultLayer
            }, { collapsed: false }).addTo(map);

            mapRef.current.addLayer(layersRef.current.grassLayer);
            mapRef.current.addLayer(layersRef.current.roadLayer);
            mapRef.current.addLayer(layersRef.current.sidewalkLayer);
            mapRef.current.addLayer(layersRef.current.buildingLayer);
            mapRef.current.addLayer(layersRef.current.obstaclesLayer);
            mapRef.current.addLayer(layersRef.current.defaultLayer);
        }
    }, [mapRef.current, layersRef, layerControlRef]);

    return null;
}

export default MapSettings;