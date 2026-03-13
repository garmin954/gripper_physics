import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
// DRACOLoader
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader.js';


const loader = new GLTFLoader();
const dracoLoader = new DRACOLoader();
dracoLoader.setDecoderPath('https://www.gstatic.com/draco/v1/decoders/');
loader.setDRACOLoader(dracoLoader);

export interface GripperParts {
    left: {
        splint: THREE.Object3D | undefined;
        support: THREE.Object3D | undefined;
        tie: THREE.Object3D | undefined;
    };
    right: {
        splint: THREE.Object3D | undefined;
        support: THREE.Object3D | undefined;
        tie: THREE.Object3D | undefined;
    };
    base: THREE.Object3D | undefined;
}

export interface GripperModel {
    scene: THREE.Group;
    parts: GripperParts;
}

export async function loadGripperModel(
    url: string, 
    onProgress?: (percent: number) => void
): Promise<GripperModel | null> {
    return new Promise((resolve, reject) => {
        loader.load(
            url,
            (gltf) => {
                const scene = gltf.scene;
                const root = scene.getObjectByName("gripper_g2");
                if (!root) {
                    resolve(null);
                    return;
                }

                const parts = {
                    left: {
                        splint: root.getObjectByName("splint_l"),
                        support: root.getObjectByName("support_l"),
                        tie: root.getObjectByName("tie_l")
                    },
                    right: {
                        splint: root.getObjectByName("splint_r"),
                        support: root.getObjectByName("support_r"),
                        tie: root.getObjectByName("tie_r")
                    },
                    base: root.getObjectByName("case")
                };

                resolve({ scene, parts });
            },
            (xhr) => {
                if (xhr.lengthComputable && onProgress) {
                    const percent = Math.min(Math.round((xhr.loaded / xhr.total) * 100), 100);
                    onProgress(percent);
                }
            },
            (error) => {
                console.error("加载 GLB 失败:", error);
                reject(error);
            }
        );
    });
}
