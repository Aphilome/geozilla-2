import React, {useEffect} from "react";
import L from "leaflet";
import {FeatureCollection} from "geojson";
import {useMap} from "react-leaflet";
import {colorizeFigure, getFigureLayer} from "../utils/layerUtils";
import Layers from "../types/Layers";

interface MapEditorProps {
    layersRef: React.MutableRefObject<Layers>;
    geoJsonView: FeatureCollection;
    setGeoJsonView: (geoJson: FeatureCollection) => void;
    activeLayer: string;
    nextFeatureId: React.MutableRefObject<number>;
   // setNextFeatureId: (nextFeatureId: number) => void;
}

const MapEditor: React.FC<MapEditorProps> = ({ geoJsonView, setGeoJsonView, layersRef, activeLayer, nextFeatureId }) => {
    console.log('MapEditor component');

    const map = useMap();

    useEffect(() => {
        console.log(`change MapEditor geoJsonView: ${geoJsonView.features.length}`);
    }, [geoJsonView]);


    useEffect(() => {
        console.log(`change MapEditor nextFeatureId: ${nextFeatureId.current}`);
    }, [nextFeatureId.current])

    useEffect(() => {
        console.log(`change MapEditor activeLayer: ${activeLayer}`);

    }, [activeLayer])


    useEffect(() => {
        if (!map) return;


        map.on('pm:create', (e) => {
            console.log('Created shape:', e);
            const figure = e.layer;

            if (figure instanceof L.Polygon) {
                console.log('activeLayer = ' + activeLayer);
                const polygon = figure as L.Polygon;
                polygon.setStyle(colorizeFigure(activeLayer));

                const targetLayer = getFigureLayer(activeLayer, layersRef);
                targetLayer.addLayer(polygon);

                const geoJson = polygon.toGeoJSON();
                geoJson.properties.zoneType = activeLayer;
                geoJson.properties.featureId = nextFeatureId.current;

                console.log('nextFeatureId = ' + nextFeatureId.current);
                console.log('old geoJsonView.features = ' + geoJsonView.features.length);

                nextFeatureId.current++;
                //setNextFeatureId(nextFeatureId + 1);
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
    }, [map, setGeoJsonView, activeLayer, layersRef]);

    return null;
};

export default MapEditor;