import React, { useRef } from 'react'
import { useGLTF } from '@react-three/drei'

export function Elf(props) {
  const { nodes, materials } = useGLTF('/elf.gltf')
  return (
    <group {...props} dispose={null}>
      <group rotation={[-Math.PI / 2, 0, 0]}>
        <mesh geometry={nodes.Object_2.geometry} material={materials.Rig2lambert23SG} />
        <mesh geometry={nodes.Object_3.geometry} material={materials.Rig2lambert23SG} />
        <mesh geometry={nodes.Object_4.geometry} material={materials.lambert22SG} />
        <mesh geometry={nodes.Object_5.geometry} material={materials.lambert22SG} />
        <mesh geometry={nodes.Object_6.geometry} material={materials.lambert22SG} />
        <mesh geometry={nodes.Object_7.geometry} material={materials.lambert25SG} />
        <mesh geometry={nodes.Object_8.geometry} material={materials.pasted__lambert2SG} />
      </group>
    </group>
  )
}

useGLTF.preload('/elf.gltf')
