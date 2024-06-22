import React, {useEffect, useRef} from "react";
import {LeafletEvent, Map} from "leaflet";
import {FeatureCollection} from "geojson";
import FigureLayers from "../types/FigureLayers";
import {onCreateHandler} from "../utils/MapEventHandlers";

interface MapEditorProps {
    layersRef: React.MutableRefObject<FigureLayers>;
    geoJsonViewRef: React.MutableRefObject<FeatureCollection>;
    setGeoJsonView: (geoJson: FeatureCollection) => void;
    activeLayer: string;
    nextFeatureIdRef: React.MutableRefObject<number>;
    mapRef: React.MutableRefObject<Map | undefined>;
}

const FigureCreator: React.FC<MapEditorProps> = ({ geoJsonViewRef, setGeoJsonView, layersRef, activeLayer, nextFeatureIdRef, mapRef }) => {
    console.log('MapEditor component');

    const activeLayerRef = useRef<string>(activeLayer);
    const map = mapRef.current!;

    useEffect(() => {
        activeLayerRef.current = activeLayer;
    }, [activeLayer])

    useEffect(() => {
        map.on('pm:create', (e: LeafletEvent) => {
            onCreateHandler(e, activeLayerRef, layersRef, nextFeatureIdRef, geoJsonViewRef, setGeoJsonView);
        });

        return () => {
            map.off('pm:create');
        };
    })

    return null;
};

export default FigureCreator;