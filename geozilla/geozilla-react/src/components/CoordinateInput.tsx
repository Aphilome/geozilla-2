import React, {useState} from 'react';
import {Box, Grid, TextField} from '@mui/material';
import {LatLngString} from "../types/LatLng";

interface CoordinateInputProps {
    setSelectedCoords: React.Dispatch<React.SetStateAction<LatLngString | null>>;
}

const CoordinateInput : React.FC<CoordinateInputProps> = ({setSelectedCoords}) => {
    const [lat, setLat] = useState('');
    const [lng, setLng] = useState('');

    const onChange = (prop: string, newValue: string) => {
        if (prop === 'lat') {
            setLat(newValue);
            setSelectedCoords({lat: newValue, lng: lng});
        }
        if (prop === 'lng') {
            setLng(newValue);
            setSelectedCoords({lat: lat, lng: newValue});
        }
    }
    
    return (
        <div>
            <Grid container style={{justifyContent: 'center'}}>
                <Grid item xs={6}>
                    <TextField
                        label="Широта"
                        value={lat}
                        onChange={(e) => onChange('lat', e.target.value)}
                    />
                </Grid>
                <Grid item xs={6}>
                    <TextField
                        label="Долгота"
                        value={lng}
                        onChange={(e) => onChange('lng', e.target.value)}
                    />
                </Grid>
            </Grid>
        </div>
    );
};

export default CoordinateInput;
