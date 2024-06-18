import L, {PathOptions} from "leaflet";
import React from "react";
import Layers from "../types/Layers";

const colorizeFigure = (zoneType: string): PathOptions => {
    switch (zoneType) {
        case 'grass':
            return {
                color: '#76c893',
                weight: 2,
                fillColor: '#76c893',
                fillOpacity: 0.5,
            };
        case 'road':
            return {
                color: '#a1a1a1',
                weight: 2,
                fillColor: '#a1a1a1',
                fillOpacity: 0.5,
            };
        case 'sidewalk':
            return {
                color: '#d1d1d1',
                weight: 2,
                fillColor: '#d1d1d1',
                fillOpacity: 0.5,
            };
        case 'building':
            return {
                color: '#ff8c00',
                weight: 2,
                fillColor: '#ff8c00',
                fillOpacity: 0.5,
            };
        default:
            return {
                color: '#ee5858',
                weight: 2,
                fillColor: '#a11ae0',
                fillOpacity: 0.5,
            };
    }
};

const getFigureLayer = (zoneType: string, layersRef: React.MutableRefObject<Layers>) => {
    switch (zoneType) {
        case 'grass':
            return layersRef.current.grassLayerRef.current;
        case 'road':
            return layersRef.current.roadLayerRef.current;
        case 'sidewalk':
            return layersRef.current.sidewalkLayerRef.current;
        case 'building':
            return layersRef.current.buildingLayerRef.current;
        default:
            return layersRef.current.defaultLayerRef.current;
    }
};

export {colorizeFigure, getFigureLayer};
