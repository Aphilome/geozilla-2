import React, {useState} from 'react';
import {Box, Container, TextField, Typography} from '@mui/material';
import {LatLngString} from "../types/LatLng";

interface CoordinateInputProps {
    setSelectedCoordsNW: (coords: LatLngString | null) => void;
    setSelectedCoordsSE: (coords: LatLngString | null) => void;
}

const CoordinateInput : React.FC<CoordinateInputProps> = ({setSelectedCoordsNW, setSelectedCoordsSE}) => {
    const [latNW, setLatNW] = useState('');
    const [lngNW, setLngNW] = useState('');
    const [latSE, setLatSE] = useState('');
    const [lngSE, setLngSE] = useState('');

    const onChangeNW = (prop: keyof LatLngString, newValue: string) => {
        if (prop === 'lat') {
            setLatNW(newValue);
            setSelectedCoordsNW({lat: newValue, lng: lngNW});
        }
        else if (prop === 'lng') {
            setLngNW(newValue);
            setSelectedCoordsNW({lat: latNW, lng: newValue});
        }
    }

    const onChangeSE = (prop: keyof LatLngString, newValue: string) => {
        if (prop === 'lat') {
            setLatSE(newValue);
            setSelectedCoordsSE({lat: newValue, lng: lngSE});
        }
        else if (prop === 'lng') {
            setLngSE(newValue);
            setSelectedCoordsSE({lat: latSE, lng: newValue});
        }
    }
    
    return (
        <Container>
            <Box sx={{'& > :not(style)': {m: 1}}}>
                <Typography variant="button" display="block" gutterBottom>
                    Северо-западная точка
                </Typography>
                <TextField
                    label="Широта"
                    value={latNW}
                    onChange={(e) => onChangeNW('lat', e.target.value)}
                />
                <TextField
                    label="Долгота"
                    value={lngNW}
                    onChange={(e) => onChangeNW('lng', e.target.value)}
                />
            </Box>
            <Box sx={{'& > :not(style)': {m: 1}}}>
                <Typography variant="button" display="block" gutterBottom>
                    Юго-восточная точка
                </Typography>
                <TextField
                    margin={'dense'}
                    label="Широта"
                    value={latSE}
                    onChange={(e) => onChangeSE('lat', e.target.value)}
                />
                <TextField
                    margin={'dense'}
                    label="Долгота"
                    value={lngSE}
                    onChange={(e) => onChangeSE('lng', e.target.value)}
                />
            </Box>
        </Container>
    );
};

export default CoordinateInput;
