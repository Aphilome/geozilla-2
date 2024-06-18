import React, { useEffect, useRef, useState } from 'react';
import {MapContainer, TileLayer} from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L, {Map} from 'leaflet';
import { Feature, FeatureCollection } from 'geojson';
import "@geoman-io/leaflet-geoman-free";
import "@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css";
import {Box, Container, Grid, Button} from "@mui/material";
import "./MapViewer.css";
import MapEditor from "./MapEditor";
import MapSettings from "./MapSettings";
import {colorizeFigure, getFigureLayer} from "../utils/layerUtils";
import FigureLayers from "../types/FigureLayers";
import JsonView from "@uiw/react-json-view";


interface MapViewerProps {
    center: [number, number];
    zoom: number;
    geoJson: FeatureCollection;
}

const MapViewer: React.FC<MapViewerProps> = ({center, zoom, geoJson}) => {
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

    const layerControlRef = useRef<L.Control.Layers | null>(null);
    const geoJsonViewRef = useRef<FeatureCollection>(geoJsonView);
    const nextFeatureId = useRef(1);
    const mapRef = useRef<Map>();

    useEffect(() => {
        console.log('geoJson set! ' + geoJson.features.length);

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
            featuresView.push(feature);
        });
        setGeoJsonView({ type: geoJsonView.type, features: featuresView});
    }, []);

    useEffect(() => {
        geoJsonViewRef.current = geoJsonView;
    }, [geoJsonView]);

    return (
        <Container>
            <Box marginTop={4} height="500px">
                <Grid container spacing={2}>
                    <Grid item xs={8} style={{ height: "500px" }}>
                        <MapContainer center={center} zoom={zoom} style={{ height: '100%', width: '100%' }} >
                            <TileLayer
                                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                                attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                            />
                            <MapSettings layerControlRef={layerControlRef} layersRef={layersRef} mapRef={mapRef}/>
                            {mapRef.current && <MapEditor geoJsonViewRef={geoJsonViewRef} setGeoJsonView={setGeoJsonView} layersRef={layersRef} activeLayer={activeLayer} nextFeatureIdRef={nextFeatureId} mapRef={mapRef} />}

                        </MapContainer>
                        <div className="layer-buttons">
                            <Button variant={activeLayer === 'default' ? 'contained' : 'outlined'} color="primary" onClick={() => setActiveLayer('default')}>Default</Button>
                            <Button variant={activeLayer === 'grass' ? 'contained' : 'outlined'} color="primary" onClick={() => setActiveLayer('grass')}>Grass</Button>
                            <Button variant={activeLayer === 'road' ? 'contained' : 'outlined'} color="primary" onClick={() => setActiveLayer('road')}>Road</Button>
                            <Button variant={activeLayer === 'sidewalk' ? 'contained' : 'outlined'} color="primary" onClick={() => setActiveLayer('sidewalk')}>Sidewalk</Button>
                            <Button variant={activeLayer === 'building' ? 'contained' : 'outlined'} color="primary" onClick={() => setActiveLayer('building')}>Building</Button>
                        </div>
                    </Grid>
                    <Grid item xs={4} style={{ height: "500px" }}>
                        <div style={{ height: '100%', overflow: 'auto', textAlign:'start' }}>
                            <JsonView value={geoJsonView} />
                        </div>
                    </Grid>
                </Grid>
            </Box>
        </Container>
    );
}
export default MapViewer;
