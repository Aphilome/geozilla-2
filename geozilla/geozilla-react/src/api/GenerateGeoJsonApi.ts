import {BaseApi} from "./BaseApi";
import axios from "axios";

export class GenerateGeoJsonApi extends BaseApi {
    private static _generateGeoJsonUri = `${this.baseUri}/generate/geo-json`;


    public static async sendData(file: File) {
        const formData = new FormData();
        formData.append("file", file);

        return await axios.post(
            this._generateGeoJsonUri,
            formData,
            {
                headers: {
                    "Content-Type": "multipart/form-data"
                },
                responseType: "blob"
            }
        );
    }
}