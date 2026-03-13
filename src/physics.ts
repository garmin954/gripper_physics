import * as RAPIER from '@dimforge/rapier3d-compat';
import type { GripperParts } from './loader';
import * as THREE from 'three';

const GRIPPER_GROUP = 0x0001FFFE;
const YAXIS = { x: 0, y: 1, z: 0 };

function getLocalVertices(obj: THREE.Object3D): { vertices: Float32Array, indices: Uint32Array } | null {
    const vertices: number[] = [];
    const indices: number[] = [];
    let vertexOffset = 0;

    obj.updateMatrixWorld(true);

    // 获取该物体在世界空间的位置、旋转和缩放
    const position = new THREE.Vector3();
    const quaternion = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    obj.matrixWorld.decompose(position, quaternion, scale);

    // 构建一个只有位置和旋转、没有缩放的矩阵，求它的逆矩阵
    // 这样变换后的顶点将“顺带”把所有缩放直接烤（bake）进坐标数据中
    const unscaledMatrix = new THREE.Matrix4().compose(position, quaternion, new THREE.Vector3(1, 1, 1));
    const inverseUnscaledMatrix = unscaledMatrix.invert();

    obj.traverse((child) => {
        if ((child as THREE.Mesh).isMesh) {
            const mesh = child as THREE.Mesh;
            const geometry = mesh.geometry;
            if (!geometry || !geometry.attributes.position) return;
            const positionAttribute = geometry.attributes.position;
            const matrix = mesh.matrixWorld;

            for (let i = 0; i < positionAttribute.count; i++) {
                const vec = new THREE.Vector3();
                vec.fromBufferAttribute(positionAttribute as THREE.BufferAttribute, i);
                vec.applyMatrix4(matrix); // 转换到绝对世界坐标 (此时已经带有所有层级的缩放)
                vec.applyMatrix4(inverseUnscaledMatrix); // 转换回该零件的无缩放局部坐标系
                vertices.push(vec.x, vec.y, vec.z);
            }

            // 获取三角形索引
            if (geometry.index) {
                const indexArray = geometry.index.array;
                for (let i = 0; i < indexArray.length; i++) {
                    indices.push(indexArray[i] + vertexOffset);
                }
            } else {
                // 对于非索引几何体，按顺序每 3 个顶点组成一个三角形
                for (let i = 0; i < positionAttribute.count; i++) {
                    indices.push(i + vertexOffset);
                }
            }

            vertexOffset += positionAttribute.count;
        }
    });

    return vertices.length > 0 && indices.length > 0
        ? { vertices: new Float32Array(vertices), indices: new Uint32Array(indices) }
        : null;
}

export class PhysicsWorld {
    public world: RAPIER.World;
    public partRigidBodies: Map<THREE.Object3D, RAPIER.RigidBody> = new Map();
    public initialRotations: Map<THREE.Object3D, THREE.Quaternion> = new Map();
    public initialTranslations: Map<THREE.Object3D, THREE.Vector3> = new Map();
    public baseRigidBodies: { rb: RAPIER.RigidBody, initialPos: THREE.Vector3 }[] = [];

    constructor() {
        // 重力设为很小或者 0，以便观察纯机械逻辑
        const gravity = { x: 0.0, y: -2.0, z: 0.0 };
        this.world = new RAPIER.World(gravity);

        // 设置地板
        const groundDesc = RAPIER.ColliderDesc.cuboid(10, 0.1, 10).setCollisionGroups(GRIPPER_GROUP);
        this.world.createCollider(groundDesc, this.world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, -0.1, 0)));
    }

    public addTestCube(cube: THREE.Object3D, halfExtents: THREE.Vector3 = new THREE.Vector3(0.1, 0.1, 0.1)) {
        cube.updateWorldMatrix(true, false);
        const position = new THREE.Vector3();
        const quaternion = new THREE.Quaternion();
        const scale = new THREE.Vector3();
        cube.matrixWorld.decompose(position, quaternion, scale);

        const rb = this.world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic()
                .setTranslation(position.x, position.y, position.z)
                .setRotation({ x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w })
                .setLinearDamping(0.2)
                .setAngularDamping(0.3)
        );

        const colliderDesc = RAPIER.ColliderDesc.cuboid(
            halfExtents.x * scale.x,
            halfExtents.y * scale.y,
            halfExtents.z * scale.z
        )
            .setFriction(0.8)
            .setRestitution(0.05);

        this.world.createCollider(colliderDesc, rb);
        this.partRigidBodies.set(cube, rb);
    }

    public addGripperParts(parts: GripperParts) {
        // 遍历所有的部件并创建对应的 RigidBody 与 Collider
        const allParts = [
            parts.base,
            parts.left.splint, parts.left.support, parts.left.tie,
            parts.right.splint, parts.right.support, parts.right.tie
        ];

        allParts.forEach((part) => {
            if (!part) return;

            // 获取部件的世界坐标和旋转
            // 当场景有整体旋转时，我们需要将该旋转结果也合并到初始刚体的状态中
            part.updateWorldMatrix(true, false);
            const position = new THREE.Vector3();
            const quaternion = new THREE.Quaternion();
            const scale = new THREE.Vector3();
            part.matrixWorld.decompose(position, quaternion, scale);
            console.log('position==>', part.name, position);


            const rbDesc = RAPIER.RigidBodyDesc.kinematicPositionBased()
                .setTranslation(position.x, position.y, position.z)
                .setRotation({ x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w });

            const rb = this.world.createRigidBody(rbDesc);
            this.partRigidBodies.set(part, rb);
            this.initialRotations.set(part, quaternion.clone());
            this.initialTranslations.set(part, position.clone());

            // 提取顶点并创建 Trimesh
            const meshData = getLocalVertices(part);
            if (meshData) {
                const { vertices, indices } = meshData;
                // 因为顶点是在相对对象坐标系下提取的，所以这里可以直接用作为 collider 的形状
                const colliderDesc = RAPIER.ColliderDesc.trimesh(vertices, indices)
                    .setCollisionGroups(GRIPPER_GROUP); // 设为不互相碰撞的组，避免关节直接散架

                if (colliderDesc) {
                    this.world.createCollider(colliderDesc, rb);
                } else {
                    console.warn(`无法为 ${part.name} 创建 Trimesh 碰撞体`);
                }
            }
        });


        this.connectGripperParts(parts, 'left')
        this.connectGripperParts(parts, 'right')

    }

    public connectGripperParts(parts: GripperParts, side: "left" | "right") {
        const { splint, support, tie } = parts[side];

        if (!splint || !support || !tie) {
            console.error(`无法获取 ${side} 侧的机械爪部件`);
            return;
        }

        const rbSplint = this.partRigidBodies.get(splint);
        const rbSupport = this.partRigidBodies.get(support);
        const rbTie = this.partRigidBodies.get(tie);

        if (!rbSplint || !rbSupport || !rbTie) {
            console.error(`无法获取 ${side} 侧对应的刚体`);
            return;
        }

        const side_prefix = side === "left" ? "l" : "r";
        const supportDot = support.getObjectByName(`support_dot_${side_prefix}`);
        const tieDot = tie.getObjectByName(`tie_dot_${side_prefix}`);
        const splintDot = splint.getObjectByName(`splint_dot_${side_prefix}`);

        if (!supportDot || !tieDot || !splintDot) {
            console.error(`无法获取 ${side} 侧的连接点 support_dot/tie_dot/splint_dot`);
            return;
        }

        // ==========================
        // 1. 设置相关的刚体物理属性
        // ==========================
        rbSupport.setBodyType(RAPIER.RigidBodyType.Dynamic, false);
        rbSupport.setAdditionalMass(1.0, true);

        rbSplint.setBodyType(RAPIER.RigidBodyType.Dynamic, false);
        rbSplint.setAdditionalMass(1.0, true);

        // ==========================
        // 2. 将连接点的世界坐标转换到对应刚体的安全局部坐标空间中
        // ==========================
        // 获取连接点的绝对世界坐标
        const supportCenterWorld = support.getWorldPosition(new THREE.Vector3());
        const supportDotWorld = supportDot.getWorldPosition(new THREE.Vector3());
        const tieDotWorld = tieDot.getWorldPosition(new THREE.Vector3());

        // 使用 worldToLocal 精确计算各零配件刚体的局部锚点偏移 (包含了旋转的修正)
        // (1) Support 与 世界固定铰链的连接锚点
        const supportAnchorFixed = support.worldToLocal(supportCenterWorld.clone());
        // (2) Support 与 Splint 的连接锚点
        const supportAnchorSplint = support.worldToLocal(supportDotWorld.clone());
        const splintAnchorSupport = splint.worldToLocal(supportDotWorld.clone());
        // (3) Tie 与 Splint 的连接锚点
        const tieAnchorSplint = tie.worldToLocal(tieDotWorld.clone());
        const splintAnchorTie = splint.worldToLocal(tieDotWorld.clone());


        console.log('supportAnchorFixed', supportAnchorFixed);
        console.log('supportAnchorSplint', supportAnchorSplint);
        console.log('splintAnchorSupport', splintAnchorSupport);
        console.log('tieAnchorSplint', tieAnchorSplint);
        console.log('splintAnchorTie', splintAnchorTie);

        // ==========================
        // 3. 建立物理关节连接
        // ==========================

        // a. 将 Support 固定在底盘/空间中
        // 【极其重要】外层场景旋转后，我们创建这个模拟固定在空中的隐形零件必须也要具有跟 support 一致的初始旋转！
        // 否则物理引擎启动的第一帧，它会发现一个向上一个向前，两根不同的销钉轴被强制合一，从而产生摧毁性的扭力把整个零件扯碎！
        const supportQuatWorld = support.getWorldQuaternion(new THREE.Quaternion());
        const rbSupportFixed = this.world.createRigidBody(
            RAPIER.RigidBodyDesc.kinematicPositionBased() // 因为必须允许玩家移动，将其从 fixed 改为 kinematicPositionBased
                .setTranslation(supportCenterWorld.x, supportCenterWorld.y, supportCenterWorld.z)
                .setRotation({ x: supportQuatWorld.x, y: supportQuatWorld.y, z: supportQuatWorld.z, w: supportQuatWorld.w })
        );
        this.baseRigidBodies.push({ rb: rbSupportFixed, initialPos: supportCenterWorld.clone() });
        this.world.createImpulseJoint(
            RAPIER.JointData.revolute(supportAnchorFixed, { x: 0, y: 0, z: 0 }, YAXIS),
            rbSupport,
            rbSupportFixed,
            true
        );

        // b. 连接 Splint 和 Support
        this.world.createImpulseJoint(
            RAPIER.JointData.revolute(splintAnchorSupport, supportAnchorSplint, YAXIS),
            rbSplint,
            rbSupport,
            true
        );

        // c. 连接 Tie 和 Splint
        this.world.createImpulseJoint(
            RAPIER.JointData.revolute(tieAnchorSplint, splintAnchorTie, YAXIS),
            rbTie,
            rbSplint,
            true
        );
    }

    public setGripperTranslation(offset: THREE.Vector3) {
        // 同步所有负责充当环境地基的锚点刚体
        this.baseRigidBodies.forEach(({ rb, initialPos }) => {
            const pos = initialPos.clone().add(offset);
            rb.setNextKinematicTranslation(pos);
        });
    }

    public setPartKinematicState(part: THREE.Object3D, eulerOffset: THREE.Euler, translationOffset: THREE.Vector3) {
        const rb = this.partRigidBodies.get(part);
        const initQuat = this.initialRotations.get(part);
        const initPos = this.initialTranslations.get(part);

        if (!rb || !initQuat || !initPos) return;

        // 构建偏移旋转
        const offsetQuat = new THREE.Quaternion().setFromEuler(eulerOffset);
        // 在初始旋转的基础上进行局部旋转叠加
        const finalQuat = initQuat.clone().multiply(offsetQuat);
        rb.setNextKinematicRotation(finalQuat);

        // 如果这个刚体并非纯物理掉落，而是由运动学驱动（如底座主动件和tie传动轴）
        // 那我们必须一并根据坐标系偏移修改它的全局位置，防止位移错位
        if (rb.bodyType() === RAPIER.RigidBodyType.KinematicPositionBased) {
            const finalPos = initPos.clone().add(translationOffset);
            rb.setNextKinematicTranslation(finalPos);
        }
    }

    public step() {
        this.world.step();
    }
}

export async function initPhysics() {
    await RAPIER.init();
    return new PhysicsWorld();
}
