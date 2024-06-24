import React, {useEffect, useRef} from 'react'
import { useGLTF } from '@react-three/drei'

export function Tile(props) {
  const { nodes, materials } = useGLTF('http://localhost:5045/tile');
  const meshRef = useRef(null);

  useEffect(() => {
    if (!nodes) return;
    caches.keys()
        .then(keys =>
          keys.forEach(key =>
              caches.delete(key)
                  .then(() => {})
                  .catch(reason => console.log('delete(key) error', reason))
          )
        )
        .catch(reason => console.log('caches.keys error', reason));
  }, [nodes])

  return (
    <group {...props} dispose={null}>
      <group ref={meshRef} rotation={[0, 0, -Math.PI / 2]}>
        <mesh  geometry={nodes.mesh_0.geometry} material={materials[""]} />
      </group>
    </group>
  )
}
