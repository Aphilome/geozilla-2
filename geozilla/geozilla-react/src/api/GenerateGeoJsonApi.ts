import {BaseApi} from "./BaseApi";
import {LatLngString} from "../types/LatLng";
import axios from "axios";

export class GenerateGeoJsonApi extends BaseApi {
    private static _generateGeoJsonUri = `${this.baseUri}/generate/geo-json`;


    public static async sendData(coordsNW: LatLngString, coordsSE: LatLngString, file: File) {
        const formData = new FormData();
        formData.append("file", file);
        formData.append("latitudeNW", coordsNW.lat ?? "");
        formData.append("longitudeNW", coordsNW.lng ?? "");
        formData.append("heightNW", coordsNW.hgt ?? "");
        formData.append("latitudeSE", coordsSE.lat ?? "");
        formData.append("longitudeSE", coordsSE.lng ?? "");
        formData.append("heightSE", coordsSE.hgt ?? "");

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