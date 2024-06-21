import React, { useState } from 'react';
import './App.css';
import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import MapViewer from './components/MapViewer';
import DataSender from "./components/DataSender";
import {FeatureCollection} from "geojson";
import Visualizer from "./components/Visualizer";

const App = () => {
    console.log("App component");
    const [geoJson, setGeoJson] = useState<FeatureCollection | null>(null);

    return (
        <div className="App">
            <Visualizer/>
            {!geoJson && <DataSender setGeoJson={setGeoJson}/>}
            {geoJson && <MapViewer geoJson={geoJson} center={[56.1322200, 47.2519400]} zoom={10} />}
        </div>
    );
}

export default App;
