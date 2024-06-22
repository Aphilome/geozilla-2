import React from 'react';
import {Button} from "@mui/material";

interface ActiveLayerSelectorProps {
    activeLayer: string;
    setActiveLayer: (layer: string) => void;
}

const ActiveLayerSelector: React.FC<ActiveLayerSelectorProps> = ({activeLayer, setActiveLayer}) => {
    return (
        <div className="layer-buttons">
            <Button variant={activeLayer === 'default' ? 'contained' : 'outlined'} color="primary" onClick={() => setActiveLayer('default')}>Default</Button>
            <Button variant={activeLayer === 'grass' ? 'contained' : 'outlined'} color="primary" onClick={() => setActiveLayer('grass')}>Grass</Button>
            <Button variant={activeLayer === 'road' ? 'contained' : 'outlined'} color="primary" onClick={() => setActiveLayer('road')}>Road</Button>
            <Button variant={activeLayer === 'sidewalk' ? 'contained' : 'outlined'} color="primary" onClick={() => setActiveLayer('sidewalk')}>Sidewalk</Button>
            <Button variant={activeLayer === 'building' ? 'contained' : 'outlined'} color="primary" onClick={() => setActiveLayer('building')}>Building</Button>
        </div>
    );
};

export default ActiveLayerSelector;