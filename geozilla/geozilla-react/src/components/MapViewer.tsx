import React, { useEffect, useRef, useState } from 'react';
import {MapContainer, TileLayer} from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L, {Map} from 'leaflet';
import { Feature, FeatureCollection, GeoJsonProperties, Geometry} from 'geojson';
import "@geoman-io/leaflet-geoman-free";
import "@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css";
import {Box, Container, Grid, Button} from "@mui/material";
import "./MapViewer.css";
import MapEditor from "./MapEditor";
import MapSettings from "./MapSettings";
import {colorizeFigure, getFigureLayer} from "../utils/layerUtils";
import Layers from "../types/Layers";


interface MapViewerProps {
    center: [number, number];
    zoom: number;
    geoJson: FeatureCollection;
}

const MapViewer: React.FC<MapViewerProps> = ({center, zoom, geoJson}) => {
    console.log('MapViewer component')

    const layersRef = useRef<Layers>({
        grassLayerRef: useRef<L.FeatureGroup>(new L.FeatureGroup()),
        roadLayerRef: useRef<L.FeatureGroup>(new L.FeatureGroup()),
        sidewalkLayerRef: useRef<L.FeatureGroup>(new L.FeatureGroup()),
        buildingLayerRef: useRef<L.FeatureGroup>(new L.FeatureGroup()),
        obstaclesLayerRef: useRef<L.FeatureGroup>(new L.FeatureGroup()),
        defaultLayerRef: useRef<L.FeatureGroup>(new L.FeatureGroup()),
    })

    const layerControlRef = useRef<L.Control.Layers | null>(null);
    //const map = useMap();
    const [activeLayer, setActiveLayer] = useState<string>('default');

    const [geoJsonView, setGeoJsonView] = useState<FeatureCollection>({type: "FeatureCollection", features: []})
    const geoJsonViewRef = useRef<FeatureCollection>(geoJsonView);
    //const [nextFeatureId, setNextFeatureId] = useState(1);
    const nextFeatureId = useRef(1);
    const mapRef = useRef<Map>();

    useEffect(() => {
        //if (!geoJson) return;

        console.log('geoJson set! ' + geoJson.features.length);

        const features = geoJson.features;
        //let id = nextFeatureId;
        let featuresView: Feature<Geometry, GeoJsonProperties>[] = [];
        features.forEach((feature) => {
            console.log('add new feature')

            const zoneType = feature.properties!.zoneType;
            const layer = L.geoJSON(feature, {
                style: colorizeFigure(zoneType)
            });

            const targetLayer = getFigureLayer(zoneType, layersRef);
            targetLayer.addLayer(layer);

            feature.properties!.featureId = nextFeatureId.current++;
            featuresView.push(feature);
        });
        //setNextFeatureId(id);
        setGeoJsonView({ type: geoJsonView.type, features: featuresView});
    }, []);

    useEffect(() => {
        console.log(`change geoJsonView: ${geoJsonView.features.length}`);
        geoJsonViewRef.current = geoJsonView;
    }, [geoJsonView]);


    useEffect(() => {
        console.log(`change nextFeatureId: ${nextFeatureId.current}`);
    }, [nextFeatureId.current])

    useEffect(() => {
        console.log(`change activeLayer: ${activeLayer}`);

    }, [activeLayer])

    const handleLayerButtonClick = (layerName: string) => {
        setActiveLayer(layerName);
        console.log(`Highlighted layer: ${layerName}`);
    };

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
                            <Button variant={activeLayer === 'default' ? 'contained' : 'outlined'} color="primary" onClick={() => handleLayerButtonClick('default')}>Default</Button>
                            <Button variant={activeLayer === 'grass' ? 'contained' : 'outlined'} color="primary" onClick={() => handleLayerButtonClick('grass')}>Grass</Button>
                            <Button variant={activeLayer === 'road' ? 'contained' : 'outlined'} color="primary" onClick={() => handleLayerButtonClick('road')}>Road</Button>
                            <Button variant={activeLayer === 'sidewalk' ? 'contained' : 'outlined'} color="primary" onClick={() => handleLayerButtonClick('sidewalk')}>Sidewalk</Button>
                            <Button variant={activeLayer === 'building' ? 'contained' : 'outlined'} color="primary" onClick={() => handleLayerButtonClick('building')}>Building</Button>
                        </div>
                    </Grid>
                    <Grid item xs={4} style={{ height: "500px" }}>
                        <div style={{ height: '100%', overflow: 'auto' }}>
                            {JSON.stringify(geoJsonView)}
                        </div>
                    </Grid>
                </Grid>
            </Box>
        </Container>
    );
}
export default MapViewer;
