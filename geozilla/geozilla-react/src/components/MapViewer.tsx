import React, { useEffect, useRef, useState } from 'react';
import { MapContainer, TileLayer } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L, { PathOptions } from 'leaflet';
import { FeatureCollection } from 'geojson';
import "@geoman-io/leaflet-geoman-free";
import "@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css";
import { Box, Container, Grid, Button } from "@mui/material";
import "./MapViewer.css";
import MapEditor from "./MapEditor";
import MapSettings from "./MapSettings";


interface MapViewerProps {
    center: [number, number];
    zoom: number;
    geoJson: FeatureCollection | null;
    setGeoJson: (geoJson: FeatureCollection) => void;
}

const MapViewer: React.FC<MapViewerProps> = ({ center, zoom, geoJson, setGeoJson }) => {
    const grassLayerRef = useRef<L.FeatureGroup>(new L.FeatureGroup());
    const roadLayerRef = useRef<L.FeatureGroup>(new L.FeatureGroup());
    const sidewalkLayerRef = useRef<L.FeatureGroup>(new L.FeatureGroup());
    const buildingLayerRef = useRef<L.FeatureGroup>(new L.FeatureGroup());
    const defaultLayerRef = useRef<L.FeatureGroup>(new L.FeatureGroup());

    const layerControlRef = useRef<L.Control.Layers | null>(null);
    const mapRef = useRef<L.Map | null>(null);
    const [activeLayer, setActiveLayer] = useState<string>('default');

    useEffect(() => {
        delete (L as any).Icon.Default.prototype._getIconUrl;

        L.Icon.Default.mergeOptions({
            iconRetinaUrl: require('leaflet/dist/images/marker-icon-2x.png').default,
            iconUrl: require('leaflet/dist/images/marker-icon.png').default,
            shadowUrl: require('leaflet/dist/images/marker-shadow.png').default,
        });
    }, []);

    useEffect(() => {
        if (!geoJson) return;

        grassLayerRef.current.clearLayers();
        roadLayerRef.current.clearLayers();
        sidewalkLayerRef.current.clearLayers();
        buildingLayerRef.current.clearLayers();
        defaultLayerRef.current.clearLayers();

        const features = geoJson.features;

        features.forEach((feature) => {
            const zoneType = feature.properties!.zoneType;

            const layer = L.geoJSON(feature, {
                style: colorize(zoneType)
            });

            const targetLayer = getLayer(zoneType);
            targetLayer.addLayer(layer);
        });
    }, [geoJson]);

    const getLayer = (zoneType: string) => {
        switch (zoneType) {
            case 'grass':
                return grassLayerRef.current;
            case 'road':
                return roadLayerRef.current;
            case 'sidewalk':
                return sidewalkLayerRef.current;
            case 'building':
                return buildingLayerRef.current;
            default:
                return defaultLayerRef.current;
        }
    };


    const colorize = (zoneType: string): PathOptions => {
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


    const handleLayerButtonClick = (layerName: string) => {
        setActiveLayer(layerName);
        console.log(`Highlighted layer: ${layerName}`);
    };

    return (
        <Container>
            <Box marginTop={4} height="500px">
                <Grid container spacing={2}>
                    <Grid item xs={8} style={{ height: "500px" }}>
                        <MapContainer center={center} zoom={zoom} style={{ height: '100%', width: '100%' }}>
                            <TileLayer
                                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                                attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                            />
                            <MapEditor geoJson={geoJson} setGeoJson={setGeoJson} grassLayerRef={grassLayerRef} roadLayerRef={roadLayerRef} sideWalkLayerRef={sidewalkLayerRef} buildingLayerRef={buildingLayerRef} defaultLayerRef={defaultLayerRef} activeLayer={activeLayer} />
                            <MapSettings mapRef={mapRef} layerControlRef={layerControlRef} grassLayerRef={grassLayerRef} roadLayerRef={roadLayerRef} sidewalkLayerRef={sidewalkLayerRef} buildingLayerRef={buildingLayerRef} defaultLayerRef={defaultLayerRef}/>

                        </MapContainer>
                        <div className="layer-buttons">
                            {activeLayer}
                            <Button variant={activeLayer === 'default' ? 'contained' : 'outlined'} color="primary" onClick={() => handleLayerButtonClick('default')}>Default</Button>
                            <Button variant={activeLayer === 'grass' ? 'contained' : 'outlined'} color="primary" onClick={() => handleLayerButtonClick('grass')}>Grass</Button>
                            <Button variant={activeLayer === 'road' ? 'contained' : 'outlined'} color="primary" onClick={() => handleLayerButtonClick('road')}>Road</Button>
                            <Button variant={activeLayer === 'sidewalk' ? 'contained' : 'outlined'} color="primary" onClick={() => handleLayerButtonClick('sidewalk')}>Sidewalk</Button>
                            <Button variant={activeLayer === 'building' ? 'contained' : 'outlined'} color="primary" onClick={() => handleLayerButtonClick('building')}>Building</Button>
                        </div>
                    </Grid>
                    <Grid item xs={4} style={{ height: "500px" }}>
                        <div style={{ height: '100%', overflow: 'auto' }}>
                            {JSON.stringify(geoJson)}
                        </div>
                    </Grid>
                </Grid>
            </Box>
        </Container>
    );
}
export default MapViewer;
