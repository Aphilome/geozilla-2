import React from 'react';
import {Canvas} from "@react-three/fiber";
import {OrbitControls} from "@react-three/drei";

interface VisualizerProps {
    model: JSX.Element;
}

const Visualizer: React.FC<VisualizerProps> = ({model}) => {
    return (
        <Canvas
            camera={{
                fov: 90,
                position: [30, 30, 50],
                zoom: 1
            }}
        >
            <ambientLight intensity={1} />
            <directionalLight position={[1, 1, 1]} intensity={0.8} />
            <OrbitControls />
            {model}
        </Canvas>
    );
};

export default Visualizer;