import React, {useState} from 'react';
import './App.css';
import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import MapViewer from './components/MapViewer';
import DataSender from "./components/DataSender";
import {FeatureCollection} from "geojson";
import Visualizer from "./components/Visualizer";
import {Elf} from "./components/Elf";

const App = () => {
    console.log("App component");
    const [geoJson, setGeoJson] = useState<FeatureCollection | null>(null);

    return (
        <div className="App">
            {!geoJson
                ?
                <div>
                    <Visualizer model={<Elf/>}/>
                    <DataSender setGeoJson={setGeoJson}/>
                </div>
                :
                <div>
                    <MapViewer geoJson={geoJson} zoom={3} />
                </div>
            }
        </div>
    );
}

export default App;
