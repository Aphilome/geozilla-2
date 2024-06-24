import React, {useState} from 'react';
import {Button, Container, Snackbar} from "@mui/material";
import FileUploader from "./FileUploader";
import {GenerateGeoJsonApi} from "../api/GenerateGeoJsonApi";
import FeatureCollectionExt from "../types/FeatureCollectionExt";

interface DataSenderProps {
    setGeoJson: (geoJson: FeatureCollectionExt) => void;
}

const DataSender: React.FC<DataSenderProps> = ({setGeoJson}) => {
    const [uploadedFile, setUploadedFile] = useState<File | null>(null);
    const [snackbarOpen, setSnackbarOpen] = useState(false);
    const [error, setError] = useState('');

    const handleSubmit = () => {
        if (!uploadedFile)
            return;

        GenerateGeoJsonApi.sendData(uploadedFile)
            .then(async response => {
                if (response) {
                    setSnackbarOpen(true);
                    setUploadedFile(null);
                    setError('');
                    setGeoJson(JSON.parse(await response.data.text()));
                } else {
                    throw new Error('Не удалось отправить координаты');
                }
            })
            .catch(error => {
                console.error(error);
                setError('Произошла ошибка при отправке файла');
            });
    };

    return (
        <Container>
            {error && <p style={{ color: 'red' }}>{error}</p>}
            <Snackbar
                open={snackbarOpen}
                onClose={() => setSnackbarOpen(false)}
                message="Данные успешно отправлены"
            />

            { !uploadedFile &&
                <FileUploader uploadedFile={uploadedFile} setUploadedFile={setUploadedFile} />
            }

            { uploadedFile &&
                <Button variant="contained" onClick={handleSubmit} style={{ height: '50px', width: '250px'}}>
                    Отправить данные
                </Button>
            }
        </Container>
    );
};

export default DataSender;