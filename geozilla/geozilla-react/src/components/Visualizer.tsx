import React from 'react';
import {Canvas} from "@react-three/fiber";
import {Elf} from "./Elf";
import {OrbitControls} from "@react-three/drei";

const Visualizer = () => {
    return (
        <div>
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
                <Elf/>
            </Canvas>
        </div>
    );
};

export default Visualizer;