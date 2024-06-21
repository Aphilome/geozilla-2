import React from 'react';
import { Canvas } from "@react-three/fiber";
import {Elf} from "./Elf";

const Visualizer = () => {
    return (
        <div>
            <Canvas
                camera={{
                    fov: 90,
                    position: [0, 0, 3],
                }}
            >
                <ambientLight intensity={0.1} />
                <directionalLight position={[1, 1, 1]} intensity={0.8} />
                {/* ! */}
                <Elf />
            </Canvas>
        </div>
    );
};

export default Visualizer;