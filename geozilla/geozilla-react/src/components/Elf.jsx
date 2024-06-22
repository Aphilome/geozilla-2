import React, { useRef } from 'react'
import { useGLTF } from '@react-three/drei'
import { useFrame } from "@react-three/fiber";

export function Elf(props) {
  const { nodes, materials } = useGLTF('./elf.glb')
  const meshRef = useRef(null);
  useFrame(() => (meshRef.current.rotation.z += 0.003));

  return (
    <group {...props} dispose={null}>
      <group ref={meshRef} rotation={[-Math.PI / 2, 0, 0]}>
        <mesh  geometry={nodes.Object_2.geometry} material={materials.Rig2lambert23SG} />
        <mesh  geometry={nodes.Object_3.geometry} material={materials.Rig2lambert23SG} />
        <mesh  geometry={nodes.Object_4.geometry} material={materials.lambert22SG} />
        <mesh  geometry={nodes.Object_5.geometry} material={materials.lambert22SG} />
        <mesh geometry={nodes.Object_6.geometry} material={materials.lambert22SG} />
        <mesh geometry={nodes.Object_7.geometry} material={materials.lambert25SG} />
        <mesh geometry={nodes.Object_8.geometry} material={materials.pasted__lambert2SG} />
      </group>
    </group>
  )
}

useGLTF.preload('./elf.glb')
