import React, { useEffect, useRef, useState } from 'react';
import {MapContainer, TileLayer} from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L, {LeafletEvent, Map} from 'leaflet';
import { Feature, FeatureCollection } from 'geojson';
import "@geoman-io/leaflet-geoman-free";
import "@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css";
import {Container, Grid} from "@mui/material";
import "./MapViewer.css";
import FigureCreator from "./FigureCreator";
import MapSettings from "./MapSettings";
import {colorizeFigure, getFigureLayer} from "../utils/LayerUtils";
import FigureLayers from "../types/FigureLayers";
import JsonView from "@uiw/react-json-view";
import {onEditHandler, onRemoveHandler} from "../utils/MapEventHandlers";
import Visualizer from "./Visualizer";
import { Tile } from './Tile';
import ActiveLayerSelector from "./ActiveLayerSelector";

interface MapViewerProps {
    zoom: number;
    geoJson: FeatureCollection;
}

const MapViewer: React.FC<MapViewerProps> = ({zoom, geoJson}) => {
    console.log('MapViewer component')

    const layersRef = useRef<FigureLayers>({
        grassLayer: new L.FeatureGroup(),
        roadLayer: new L.FeatureGroup(),
        sidewalkLayer: new L.FeatureGroup(),
        buildingLayer: new L.FeatureGroup(),
        obstaclesLayer: new L.FeatureGroup(),
        defaultLayer: new L.FeatureGroup()
    })

    const [activeLayer, setActiveLayer] = useState<string>('default');
    const [geoJsonView, setGeoJsonView] = useState<FeatureCollection>({type: "FeatureCollection", features: []})
    const [mapInitialized, setMapInitialized] = useState(false);

    const layerControlRef = useRef<L.Control.Layers | null>(null);
    const geoJsonViewRef = useRef<FeatureCollection>(geoJsonView);
    const nextFeatureId = useRef(1);
    const mapRef = useRef<Map>();

    useEffect(() => {
        const features = geoJson.features;
        let featuresView: Feature[] = [];
        features.forEach((feature) => {
            const zoneType = feature.properties!.zoneType;
            const layer = L.geoJSON(feature, {
                style: colorizeFigure(zoneType)
            });

            const targetLayer = getFigureLayer(zoneType, layersRef);
            targetLayer.addLayer(layer);

            feature.properties!.featureId = nextFeatureId.current++;
            layer.on('pm:edit', (e: LeafletEvent) =>
                onEditHandler(e, feature.properties!.featureId, geoJsonViewRef, setGeoJsonView));
            layer.on('pm:remove', (e: LeafletEvent) =>
                onRemoveHandler(e, feature.properties!.featureId, geoJsonViewRef, setGeoJsonView));

            layer.bindTooltip("<div class='geoHightTooltip'>" + "hi " + "</div>",
            {
                direction: 'right',
                permanent: false,
                sticky: true,
                offset: [10, 0],
                opacity: 0.75,
                className: 'leaflet-tooltip-own'
            });
            featuresView.push(feature);
        });
        setGeoJsonView({ type: geoJsonView.type, features: featuresView});
    }, []);

    useEffect(() => {
        geoJsonViewRef.current = geoJsonView;
    }, [geoJsonView]);

    useEffect(() => {
        if (!mapInitialized) return;

        let bounds: L.LatLngBounds | null = null;
        mapRef.current?.eachLayer((layer) => {
            if (bounds || !('getBounds' in layer)) return;
            bounds = (layer as any).getBounds() as L.LatLngBounds;
        });
        if (!bounds) return;

        mapRef.current?.flyToBounds(bounds);
    }, [mapInitialized])

    // @ts-ignore
    return (
        <Container>
            <Grid container spacing={2}>
                <Grid item xs={7} style={{ height: "800px", width: '1600px' }}>
                    <MapContainer center={[0, 0]} zoom={zoom} style={{ height: "100%", width: '100%' }} >
                        <TileLayer
                            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                            attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                        />
                        <MapSettings layerControlRef={layerControlRef} layersRef={layersRef} mapRef={mapRef} setMapInitialized={setMapInitialized}/>
                        {mapRef.current && <FigureCreator geoJsonViewRef={geoJsonViewRef} setGeoJsonView={setGeoJsonView} layersRef={layersRef} activeLayer={activeLayer} nextFeatureIdRef={nextFeatureId} mapRef={mapRef} />}
                    </MapContainer>
                    <ActiveLayerSelector activeLayer={activeLayer} setActiveLayer={setActiveLayer}/>
                    <Visualizer model={<Tile/>}/>
                </Grid>
                <Grid item xs={3} style={{ height: "800px" }}>
                    <div style={{ height: '100%', overflow: 'auto', textAlign:'start' }}>
                        <JsonView value={geoJsonView} />
                    </div>
                </Grid>
            </Grid>
        </Container>
    );
}
export default MapViewer;
