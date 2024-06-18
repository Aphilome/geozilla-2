import React, {useEffect, useRef} from "react";
import L, {Map} from "leaflet";
import {FeatureCollection} from "geojson";
import {colorizeFigure, getFigureLayer} from "../utils/layerUtils";
import Layers from "../types/Layers";

interface MapEditorProps {
    layersRef: React.MutableRefObject<Layers>;
    geoJsonView: FeatureCollection;
    setGeoJsonView: (geoJson: FeatureCollection) => void;
    activeLayer: string;
    nextFeatureIdRef: React.MutableRefObject<number>;
    mapRef: React.MutableRefObject<Map | undefined>;
}

const MapEditor: React.FC<MapEditorProps> = ({ geoJsonView, setGeoJsonView, layersRef, activeLayer, nextFeatureIdRef, mapRef }) => {
    console.log('MapEditor component');

    const activeLayerRef = useRef<string>(activeLayer);

    useEffect(() => {
        activeLayerRef.current = activeLayer;
    }, [activeLayer])


    useEffect(() => {
        const map = mapRef.current!;

        map.on('pm:create', (e) => {
            console.log('Created shape:', e);
            const figure = e.layer;

            if (figure instanceof L.Polygon) {
                const polygon = figure as L.Polygon;
                polygon.setStyle(colorizeFigure(activeLayerRef.current));

                const targetLayer = getFigureLayer(activeLayerRef.current, layersRef);
                targetLayer.addLayer(polygon);

                const geoJson = polygon.toGeoJSON();
                geoJson.properties.zoneType = activeLayerRef.current;
                geoJson.properties.featureId = nextFeatureIdRef.current;

                nextFeatureIdRef.current++;
                setGeoJsonView({ type: geoJsonView.type, features: [...geoJsonView.features, geoJson]})
            }
        });

        map.on('pm:edit', (e) => {
            //updateGeoJson();
            console.log('Edited shape:', e);
        });

        map.on('pm:remove', (e) => {
            //updateGeoJson();
            console.log('Removed shape:', e);
        });

        return () => {
            map.off('pm:create');
            map.off('pm:edit');
            map.off('pm:remove');
        };
    }, []);

    return null;
};

export default MapEditor;