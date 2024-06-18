import React, {useState} from 'react';
import {Box, Button, Container, Snackbar} from "@mui/material";
import FileUploader from "./FileUploader";
import CoordinateInput from "./CoordinateInput";
import {LatLngString} from "../types/LatLng";
import {GenerateGeoJsonApi} from "../api/GenerateGeoJsonApi";
import {FeatureCollection } from "geojson";

interface DataSenderProps {
    setGeoJson: (geoJson: FeatureCollection) => void;
}

const DataSender: React.FC<DataSenderProps> = ({setGeoJson}) => {
    const [uploadedFile, setUploadedFile] = useState<File | null>(null);
    const [selectedCoordsNW, setSelectedCoordsNW] = useState<LatLngString | null>(null);
    const [selectedCoordsSE, setSelectedCoordsSE] = useState<LatLngString | null>(null);
    const [snackbarOpen, setSnackbarOpen] = useState(false);
    const [error, setError] = useState('');

    const handleSubmit = () => {
        if (!selectedCoordsNW || !selectedCoordsSE || !uploadedFile)
            return;

        GenerateGeoJsonApi.sendData(selectedCoordsNW, selectedCoordsSE, uploadedFile)
            .then(async response => {
                if (response) {
                    setSnackbarOpen(true);
                    setSelectedCoordsSE({lat: '', lng: ''});
                    setSelectedCoordsNW({lat: '', lng: ''});
                    setUploadedFile(null);
                    setError('');
                    setGeoJson(JSON.parse(await response.data.text()));
                } else {
                    throw new Error('Не удалось отправить координаты');
                }
            })
            .catch(error => {
                console.error(error);
                setError('Произошла ошибка при отправке координат');
            });
    };

    return (
        <Container>

            <Box marginTop={4}>
                <CoordinateInput setSelectedCoordsNW={setSelectedCoordsNW} setSelectedCoordsSE={setSelectedCoordsSE}/>
            </Box>

            {error && <p style={{ color: 'red' }}>{error}</p>}
            <Snackbar
                open={snackbarOpen}
                onClose={() => setSnackbarOpen(false)}
                message="Данные успешно отправлены"
            />

            { !uploadedFile && <Box margin={1}>Выберите файл</Box> }
            <Box marginTop={1} marginBottom={1}>
                <FileUploader setUploadedFile={setUploadedFile} />
            </Box>
            <Button variant="contained" onClick={handleSubmit}>
                Отправить данные
            </Button>
        </Container>
    );
};

export default DataSender;