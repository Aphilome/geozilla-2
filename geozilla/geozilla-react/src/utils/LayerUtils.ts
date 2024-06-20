import {PathOptions} from "leaflet";
import React from "react";
import FigureLayers from "../types/FigureLayers";

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
        case 'obstacles':
            return {
                color: '#863b5d',
                weight: 2,
                fillColor: '#863b5d',
                fillOpacity: 0.5,
            }
        default:
            return {
                color: '#ee5858',
                weight: 2,
                fillColor: '#a11ae0',
                fillOpacity: 0.5,
            };
    }
};

const getFigureLayer = (zoneType: string, layersRef: React.MutableRefObject<FigureLayers>) => {
    switch (zoneType) {
        case 'grass':
            return layersRef.current.grassLayer;
        case 'road':
            return layersRef.current.roadLayer;
        case 'sidewalk':
            return layersRef.current.sidewalkLayer;
        case 'building':
            return layersRef.current.buildingLayer;
        case 'obstacles':
            return layersRef.current.obstaclesLayer;
        default:
            return layersRef.current.defaultLayer;
    }
};

export {colorizeFigure, getFigureLayer};
