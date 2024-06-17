import React, {useEffect} from "react";
import L from "leaflet";
import {FeatureCollection} from "geojson";
import {useMap} from "react-leaflet";
import * as geojson from "geojson";

interface MapEditorProps {
    grassLayerRef: React.MutableRefObject<L.FeatureGroup<any>>;
    roadLayerRef: React.MutableRefObject<L.FeatureGroup<any>>;
    sideWalkLayerRef: React.MutableRefObject<L.FeatureGroup<any>>;
    buildingLayerRef: React.MutableRefObject<L.FeatureGroup<any>>;
    defaultLayerRef: React.MutableRefObject<L.FeatureGroup<any>>;
    geoJson: FeatureCollection | null;
    setGeoJson: (geoJson: FeatureCollection) => void;
    activeLayer: string;
}

const MapEditor: React.FC<MapEditorProps> = ({ setGeoJson, grassLayerRef, roadLayerRef, sideWalkLayerRef, buildingLayerRef, defaultLayerRef, activeLayer }) => {
    const map = useMap();

    useEffect(() => {
        if (!map) return;

        map.pm.addControls({
            position: 'topleft',
            drawCircle: false,
            drawMarker: false,
            drawCircleMarker: false,
        });

        const updateGeoJson = () => {
            //debugger;
            const allLayers = map.pm.getGeomanLayers();

            const updatedGeoJson: FeatureCollection = {
                type: 'FeatureCollection',
                features: allLayers.map((layer: L.Layer) => {
                    if ('toGeoJSON' in layer) {
                        return (layer as any).toGeoJSON() as geojson.Feature;
                    }
                    return null;
                }).filter((feature): feature is geojson.Feature => feature !== null)
            };
            setGeoJson(updatedGeoJson);
        };

        const addLayerToActiveLayer = (layer: L.Layer) => {
            console.log('addLayerToActiveLayer - ' + activeLayer);
            switch (activeLayer) {
                case 'grass':
                    grassLayerRef.current.addLayer(layer);
                    break;
                case 'road':
                    roadLayerRef.current.addLayer(layer);
                    break;
                case 'sidewalk':
                    sideWalkLayerRef.current.addLayer(layer);
                    break;
                case 'building':
                    buildingLayerRef.current.addLayer(layer);
                    break;
                default:
                    defaultLayerRef.current.addLayer(layer);
            }
        };

        map.on('pm:create', (e) => {
            const layer = e.layer;
            const feature = (layer as any).toGeoJSON() as geojson.Feature;
            feature.properties!.zoneType = activeLayer;

            addLayerToActiveLayer(layer);

            updateGeoJson();
            console.log('Created shape:', e);
        });

        map.on('pm:edit', (e) => {
            updateGeoJson();
            console.log('Edited shape:', e);
        });

        map.on('pm:remove', (e) => {
            updateGeoJson();
            console.log('Removed shape:', e);
        });

        return () => {
            map.off('pm:create');
            map.off('pm:edit');
            map.off('pm:remove');
        };
    }, [map, setGeoJson, activeLayer, defaultLayerRef, buildingLayerRef, grassLayerRef, roadLayerRef, sideWalkLayerRef]);

    return null;
};

export default MapEditor;