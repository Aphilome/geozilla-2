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
    const [hgtNW, setHgtNW] = useState('');
    const [latSE, setLatSE] = useState('');
    const [lngSE, setLngSE] = useState('');
    const [hgtSE, setHgtSE] = useState('');

    const onChangeNW = (prop: keyof LatLngString, newValue: string) => {
        switch (prop) {
            case 'lat':
                setLatNW(newValue);
                setSelectedCoordsNW({lat: newValue, lng: lngNW, hgt: hgtNW});
                break;
            case 'lng':
                setLngNW(newValue);
                setSelectedCoordsNW({lat: latNW, lng: newValue, hgt: hgtNW});
                break;
            case 'hgt':
                setHgtNW(newValue);
                setSelectedCoordsNW({lat: latNW, lng: lngNW, hgt: newValue});
                break;
        }
    }

    const onChangeSE = (prop: keyof LatLngString, newValue: string) => {
        switch (prop){
            case 'lat':
                setLatSE(newValue);
                setSelectedCoordsSE({lat: newValue, lng: lngSE, hgt: hgtSE});
                break;
            case 'lng':
                setLngSE(newValue);
                setSelectedCoordsSE({lat: latSE, lng: newValue, hgt: hgtSE});
                break;
            case 'hgt':
                setHgtSE(newValue);
                setSelectedCoordsSE({lat: latSE, lng: lngSE, hgt: newValue});
                break;
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
                <TextField
                    label="Высота"
                    value={hgtNW}
                    onChange={(e) => onChangeNW('hgt', e.target.value)}
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
                <TextField
                    label="Высота"
                    value={hgtSE}
                    onChange={(e) => onChangeSE('hgt', e.target.value)}
                />
            </Box>
        </Container>
    );
};

export default CoordinateInput;
