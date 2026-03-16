import * as RAPIER from '@dimforge/rapier3d-compat';
import type { GripperParts } from './loader';
import * as THREE from 'three';
import { degToRad } from 'three/src/math/MathUtils.js';

// 碰撞组定义：(membership << 16) | filter
// 夹爪零件：属于 Group 1 (0x0002)，只与 Group 0 (0x0001) 发生碰撞，不与自己(Group 1)碰撞
const GRIPPER_GROUP = (0x0002 << 16) | 0x0001;
// 环境物体：属于 Group 0 (0x0001)，与所有组发生碰撞
const ENVIRONMENT_GROUP = (0x0001 << 16) | 0xFFFF;

const YAXIS = { x: 0, y: 1, z: 0 };

// 将世界坐标转换为局部坐标
function worldPointToLocal(point: THREE.Vector3, origin: THREE.Vector3, rotation: THREE.Quaternion): THREE.Vector3 {
    const inverseRotation = rotation.clone().invert();
    return point.clone().sub(origin).applyQuaternion(inverseRotation);
}

function getLocalVertices(obj: THREE.Object3D): { vertices: Float32Array, indices: Uint32Array } | null {
    const vertices: number[] = [];
    const indices: number[] = [];
    let vertexOffset = 0;
    const tempVec = new THREE.Vector3(); // 复用向量

    obj.updateMatrixWorld(true);

    const position = new THREE.Vector3();
    const quaternion = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    obj.matrixWorld.decompose(position, quaternion, scale);

    const unscaledMatrix = new THREE.Matrix4().compose(position, quaternion, new THREE.Vector3(1, 1, 1));
    const inverseUnscaledMatrix = unscaledMatrix.invert();

    obj.traverse((child) => {
        if ((child as THREE.Mesh).isMesh) {
            const mesh = child as THREE.Mesh;
            const geometry = mesh.geometry;
            if (!geometry?.attributes.position) return;

            const positionAttribute = geometry.attributes.position;
            const matrix = mesh.matrixWorld;

            for (let i = 0; i < positionAttribute.count; i++) {
                tempVec.fromBufferAttribute(positionAttribute as THREE.BufferAttribute, i);
                tempVec.applyMatrix4(matrix);
                tempVec.applyMatrix4(inverseUnscaledMatrix);
                vertices.push(tempVec.x, tempVec.y, tempVec.z);
            }

            if (geometry.index) {
                const indexArray = geometry.index.array;
                for (let i = 0; i < indexArray.length; i++) {
                    indices.push(indexArray[i] + vertexOffset);
                }
            } else {
                // 修正：按三角形（每3个顶点）生成索引
                for (let i = 0; i < positionAttribute.count; i += 3) {
                    indices.push(i + vertexOffset, i + 1 + vertexOffset, i + 2 + vertexOffset);
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
    public gripperDynamicBodies: Map<number, RAPIER.RigidBody> = new Map();
    // 专门存储 tie 的电机关节
    public tieJoints: Map<"left" | "right", RAPIER.RevoluteImpulseJoint> = new Map();
    // 夹爪底座的刚体
    public gripperBaseBody: RAPIER.RigidBody | null = null;
    // 夹爪底座的初始位置
    public gripperBaseInitialPos: THREE.Vector3 | null = null;
    // 夹爪底座的初始旋转
    public gripperBaseInitialRot: THREE.Quaternion | null = null;

    constructor() {
        // 重力设为很小或者 0，以便观察纯机械逻辑
        const gravity = { x: 0.0, y: -9.8, z: 0.0 };
        this.world = new RAPIER.World(gravity);

        // 增大求解器迭代次数，让关节约束更精确、更"硬"
        // 将迭代次数提升至 128，极大增强关节在运动时的抗变形能力
        this.world.numSolverIterations = 64;
        // 增加内部投影高斯-赛德尔（PGS）迭代次数，增强关节精度
        this.world.numInternalPgsIterations = 10;

        // 设置地板，分入 ENVIRONMENT_GROUP
        const groundDesc = RAPIER.ColliderDesc.cuboid(10, 0.1, 10).setCollisionGroups(ENVIRONMENT_GROUP);
        this.world.createCollider(groundDesc, this.world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, -0.1, 0)));
    }

    // 注册夹爪的动态刚体
    private registerGripperDynamicBody(rb: RAPIER.RigidBody) {
        this.gripperDynamicBodies.set(rb.handle, rb);
    }

    // 添加测试方块
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
                .setLinearDamping(1.0) // 线性阻尼
                .setAngularDamping(1.0) // 角阻尼
        );

        const colliderDesc = RAPIER.ColliderDesc.cuboid(
            halfExtents.x * scale.x,
            halfExtents.y * scale.y,
            halfExtents.z * scale.z
        )
            .setFriction(2.0)  // 极大增强摩擦力
            .setRestitution(0.0) // 零弹性
            .setCollisionGroups(ENVIRONMENT_GROUP); // 分入 ENVIRONMENT_GROUP

        this.world.createCollider(colliderDesc, rb);
        this.partRigidBodies.set(cube, rb);

        // 开启 CCD，防止由于力大导致的穿模
        rb.enableCcd(true);
    }

    // 添加夹爪部件
    public addGripperParts(parts: GripperParts) {
        const baseReferencePart =
            parts.base
            ?? parts.left.support
            ?? parts.right.support
            ?? parts.left.tie
            ?? parts.right.tie
            ?? parts.left.splint
            ?? parts.right.splint;

        if (baseReferencePart) {
            baseReferencePart.updateWorldMatrix(true, false);
            const basePosition = new THREE.Vector3();
            const baseQuaternion = new THREE.Quaternion();
            const baseScale = new THREE.Vector3();
            baseReferencePart.matrixWorld.decompose(basePosition, baseQuaternion, baseScale);

            this.gripperBaseBody = this.world.createRigidBody(
                RAPIER.RigidBodyDesc.kinematicPositionBased()
                    .setTranslation(basePosition.x, basePosition.y, basePosition.z)
                    .setRotation({ x: baseQuaternion.x, y: baseQuaternion.y, z: baseQuaternion.z, w: baseQuaternion.w })
            );
            this.gripperBaseInitialPos = basePosition.clone();
            this.gripperBaseInitialRot = baseQuaternion.clone();
        }

        // 遍历所有的部件并创建对应的 RigidBody 与 Collider
        const allParts = [
            parts.base,
            parts.left.splint, parts.left.support, parts.left.tie,
            parts.right.splint, parts.right.support, parts.right.tie
        ];

        allParts.forEach((part) => {
            if (!part) return;

            // 获取部件的世界坐标和旋转
            part.updateWorldMatrix(true, false);
            const position = new THREE.Vector3();
            const quaternion = new THREE.Quaternion();
            const scale = new THREE.Vector3();
            part.matrixWorld.decompose(position, quaternion, scale);

            // 初始全部设置为 kinematic，但在 connectGripperParts 中我们会根据需要切换到 Dynamic
            const rbDesc = RAPIER.RigidBodyDesc.kinematicPositionBased()
                .setTranslation(position.x, position.y, position.z)
                .setRotation({ x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w });

            const rb = this.world.createRigidBody(rbDesc);
            this.partRigidBodies.set(part, rb);
            this.initialRotations.set(part, quaternion.clone());
            this.initialTranslations.set(part, position.clone());

            // 给所有零件开启 CCD
            rb.enableCcd(true);

            const meshData = getLocalVertices(part);
            if (meshData) {
                const { vertices, indices } = meshData;
                const colliderDesc = RAPIER.ColliderDesc.trimesh(vertices, indices);

                if (colliderDesc) {
                    colliderDesc.setCollisionGroups(GRIPPER_GROUP);
                    this.world.createCollider(colliderDesc, rb);
                }
            }
        });


        this.connectGripperParts(parts, 'left')
        this.connectGripperParts(parts, 'right')
    }

    // 连接夹爪部件
    public connectGripperParts(parts: GripperParts, side: "left" | "right") {
        const { splint, support, tie } = parts[side];
        if (!splint || !support || !tie) return;

        const rbSplint = this.partRigidBodies.get(splint)!;
        const rbSupport = this.partRigidBodies.get(support)!;
        const rbTie = this.partRigidBodies.get(tie)!;

        // 设置零件为 Dynamic，使其接受物理碰撞阻力
        rbSplint.setBodyType(RAPIER.RigidBodyType.Dynamic, false);
        rbSupport.setBodyType(RAPIER.RigidBodyType.Dynamic, false);
        rbTie.setBodyType(RAPIER.RigidBodyType.Dynamic, false);
        this.registerGripperDynamicBody(rbSplint);
        this.registerGripperDynamicBody(rbSupport);
        this.registerGripperDynamicBody(rbTie);

        // 降低运动件惯性，减少整体平移时把闭合链条拉开的趋势。
        rbTie.setAdditionalMass(0.25, true);
        rbSplint.setAdditionalMass(0.25, true);
        rbSupport.setAdditionalMass(0.25, true);
        rbTie.setAdditionalSolverIterations(32);
        rbSplint.setAdditionalSolverIterations(32);
        rbSupport.setAdditionalSolverIterations(32);

        rbSplint.setLinearDamping(0.5);
        rbSupport.setLinearDamping(0.5);
        rbTie.setLinearDamping(0.5);
        rbSplint.setAngularDamping(2.0);
        rbSupport.setAngularDamping(2.0);
        rbTie.setAngularDamping(2.0);


        const side_prefix = side === "left" ? "l" : "r";
        const supportDot = support.getObjectByName(`support_dot_${side_prefix}`)!;
        const tieDot = tie.getObjectByName(`tie_dot_${side_prefix}`)!;

        // 获取连接点世界坐标
        const supportCenterWorld = support.getWorldPosition(new THREE.Vector3());
        const supportDotWorld = supportDot.getWorldPosition(new THREE.Vector3());
        const tieDotWorld = tieDot.getWorldPosition(new THREE.Vector3());
        const tieCenterWorld = tie.getWorldPosition(new THREE.Vector3());
        const baseBody = this.gripperBaseBody;
        const basePos = this.gripperBaseInitialPos;
        const baseRot = this.gripperBaseInitialRot;

        if (!baseBody || !basePos || !baseRot) return;

        // 将世界坐标转换为局部坐标
        const supportAnchorFixed = support.worldToLocal(supportCenterWorld.clone());
        const supportAnchorSplint = support.worldToLocal(supportDotWorld.clone());
        const splintAnchorSupport = splint.worldToLocal(supportDotWorld.clone());
        const tieAnchorSplint = tie.worldToLocal(tieDotWorld.clone());
        const splintAnchorTie = splint.worldToLocal(tieDotWorld.clone());
        const tieAnchorLocal = tie.worldToLocal(tieCenterWorld.clone());
        const supportAnchorBase = worldPointToLocal(supportCenterWorld, basePos, baseRot);
        const tieAnchorBase = worldPointToLocal(tieCenterWorld, basePos, baseRot);

        // 1. Support 固定点挂到底座上，整个夹爪平移时共享同一个运动学参考系。
        const jointSupportFixed = this.world.createImpulseJoint(
            RAPIER.JointData.revolute(
                supportAnchorFixed,
                { x: supportAnchorBase.x, y: supportAnchorBase.y, z: supportAnchorBase.z },
                YAXIS
            ),
            rbSupport,
            baseBody,
            true
        ) as RAPIER.RevoluteImpulseJoint;

        jointSupportFixed.setLimits(side === "left" ? degToRad(-30) : 0, side === "left" ? 0 : degToRad(30))
        jointSupportFixed.setContactsEnabled(false);

        // 2. Splint 和 Support 连接
        this.world.createImpulseJoint(
            RAPIER.JointData.revolute(splintAnchorSupport, supportAnchorSplint, YAXIS),
            rbSplint,
            rbSupport,
            true
        );

        // 3. Tie 和 Splint 连接
        const jointTieSplint = this.world.createImpulseJoint(
            RAPIER.JointData.revolute(tieAnchorSplint, splintAnchorTie, YAXIS),
            rbTie,
            rbSplint,
            true
        ) as RAPIER.RevoluteImpulseJoint;
        jointTieSplint.setLimits(side === "left" ? degToRad(-30) : 0, side === "left" ? 0 : degToRad(30))


        const jointTieToAnchor = this.world.createImpulseJoint(
            RAPIER.JointData.revolute(
                tieAnchorLocal,
                { x: tieAnchorBase.x, y: tieAnchorBase.y, z: tieAnchorBase.z },
                YAXIS
            ),
            rbTie,
            baseBody,
            true
        ) as RAPIER.RevoluteImpulseJoint;
        jointTieToAnchor.setLimits(side === "left" ? degToRad(-30) : 0, side === "left" ? 0 : degToRad(30))


        // 这里就是主驱动电机！
        jointTieToAnchor.configureMotorModel(RAPIER.MotorModel.AccelerationBased);
        jointTieToAnchor.configureMotorPosition(0, 20000.0, 5000.0);

        // 禁用关节连接体之间的碰撞计算，减少内部压力导致的变形
        jointTieToAnchor.setContactsEnabled(false);
        this.tieJoints.set(side, jointTieToAnchor);
    }

    // 设置夹爪的平移
    public setGripperTranslation(offset: THREE.Vector3, linearVelocity?: THREE.Vector3) {
        if (!this.gripperBaseBody || !this.gripperBaseInitialPos) return;

        const pos = this.gripperBaseInitialPos.clone().add(offset);
        this.gripperBaseBody.setNextKinematicTranslation(pos);

        if (this.gripperBaseInitialRot) {
            this.gripperBaseBody.setNextKinematicRotation(this.gripperBaseInitialRot);
        }

        if (linearVelocity) {
            const velocity = { x: linearVelocity.x, y: linearVelocity.y, z: linearVelocity.z };
            this.gripperDynamicBodies.forEach((rb) => {
                rb.setLinvel(velocity, true);
            });
        }
    }

    public setPartKinematicState(part: THREE.Object3D, eulerOffset: THREE.Euler, translationOffset: THREE.Vector3) {
        const rb = this.partRigidBodies.get(part);
        const initQuat = this.initialRotations.get(part);
        const initPos = this.initialTranslations.get(part);
        if (!rb || !initQuat || !initPos) return;

        // 注意：现在 tie 是 Dynamic 的，不应该再通过 setNextKinematicRotation 处理它的旋转。
        // 我们只在这里处理 base/case 等依然保持 kinematic 属性的零件的位置同步。
        if (rb.bodyType() === RAPIER.RigidBodyType.KinematicPositionBased) {
            const finalPos = initPos.clone().add(translationOffset);
            rb.setNextKinematicTranslation(finalPos);

            // 只有非 Tie 零件（比如 base）才同步旋转偏量，Tie 的旋转现在交给电机驱动锚点处理
            const offsetQuat = new THREE.Quaternion().setFromEuler(eulerOffset);
            const finalQuat = initQuat.clone().multiply(offsetQuat);
            rb.setNextKinematicRotation(finalQuat);
        }
    }

    /**
     * 更新 Tie 驱动电机的目标角度
     */
    public setTieMotorTarget(side: "left" | "right", radians: number) {
        const joint = this.tieJoints.get(side);
        if (joint) {
            joint.configureMotorModel(RAPIER.MotorModel.AccelerationBased);
            joint.configureMotorPosition(radians, 20000.0, 5000.0);
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
