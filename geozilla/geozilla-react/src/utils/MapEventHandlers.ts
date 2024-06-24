import {colorizeFigure, getFigureLayer} from "./LayerUtils";
import {LeafletEvent} from "leaflet";
import React from "react";
import FigureLayers from "../types/FigureLayers";
import {FeatureCollection} from "geojson";
import "./MapEventHandlers.css";
import FeatureCollectionExt from "../types/FeatureCollectionExt";

const onCreateHandler = (e: LeafletEvent,
                         activeLayerRef: React.MutableRefObject<string>,
                         layersRef: React.MutableRefObject<FigureLayers>,
                         nextFeatureIdRef: React.MutableRefObject<number>,
                         geoJsonViewRef: React.MutableRefObject<FeatureCollectionExt>,
                         setGeoJsonView: (geoJsonView: FeatureCollectionExt) => void) => {
    const layer = e.layer;
    if (!('toGeoJSON' in layer && 'setStyle' in layer)) return;

    const figure = layer as any;
    figure.setStyle(colorizeFigure(activeLayerRef.current));

    const targetLayer = getFigureLayer(activeLayerRef.current, layersRef);
    targetLayer.addLayer(figure);

    const geoJson = figure.toGeoJSON();
    geoJson.properties.zoneType = activeLayerRef.current;
    geoJson.properties.featureId = nextFeatureIdRef.current;
    nextFeatureIdRef.current++;
    setGeoJsonView({ type: geoJsonViewRef.current.type, name: geoJsonViewRef.current.name, features: [...geoJsonViewRef.current.features, geoJson]})

    figure.on('pm:edit', (ie: LeafletEvent) => {
        onEditHandler(ie, geoJson.properties.featureId, geoJsonViewRef, setGeoJsonView);
    });

    figure.on('pm:remove', (ie: LeafletEvent) => {
        onRemoveHandler(ie, geoJson.properties.featureId, geoJsonViewRef, setGeoJsonView);
    });

    // e.layer.bindTooltip("<div class='geoHightTooltip1'>" + "hi " + "</div>",
    //     {
    //         direction: 'right',
    //         permanent: false,
    //         sticky: true,
    //         offset: [10, 0],
    //         opacity: 0.75,
    //         className: 'leaflet-tooltip-own'
    //     });
}

const onEditHandler = (e: LeafletEvent,
                       featureId: number,
                       geoJsonViewRef: React.MutableRefObject<FeatureCollectionExt>,
                       setGeoJsonView: (geoJsonView: FeatureCollectionExt) => void) => {
    const layer = e.layer;
    if (!('toGeoJSON' in layer)) return;

    const figure = layer.toGeoJSON();
    let old = geoJsonViewRef.current.features.find(f => f.properties!.featureId === featureId);
    old!.geometry = figure.geometry;

    setGeoJsonView({...geoJsonViewRef.current, features: geoJsonViewRef.current.features});
}

const onRemoveHandler = (e: LeafletEvent,
                         featureId: number,
                         geoJsonViewRef: React.MutableRefObject<FeatureCollectionExt>,
                         setGeoJsonView: (geoJsonView: FeatureCollectionExt) => void) => {
    const otherFeatures = geoJsonViewRef.current.features.filter(f => f.properties!.featureId !== featureId);

    setGeoJsonView({...geoJsonViewRef.current, features: otherFeatures});
}

export {onCreateHandler, onEditHandler, onRemoveHandler};