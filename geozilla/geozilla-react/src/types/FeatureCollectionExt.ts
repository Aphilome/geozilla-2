import {FeatureCollection} from "geojson";

interface FeatureCollectionExt extends FeatureCollection {
    name: string;
}

export default FeatureCollectionExt;