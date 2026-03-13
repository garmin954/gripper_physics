import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { initPhysics } from './physics';
import { loadGripperModel } from './loader';
import { initGUI, guiState, initControlsOverlay } from './gui';
import './style.css'
import gripperModelUrl from './assets/gripperg2.glb?url';

async function main() {
    // 1. 初始化物理
    const physics = await initPhysics();

    // 2. 初始化渲染器
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x202020);
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(0, 5, 5);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);

    // 半球光：提供整体环境照明
    const hemi = new THREE.HemisphereLight(0xbad7f0, 0x1d1a12, 0.65);
    scene.add(hemi);

    // 坐标系
    const axesHelper = new THREE.AxesHelper(2);
    scene.add(axesHelper);

    // 平行光：提供方向光与阴影
    const dir = new THREE.DirectionalLight(0xffffff, 1.1);
    dir.position.set(0, 10, 0);
    dir.castShadow = true;
    dir.shadow.mapSize.set(1024, 1024);
    scene.add(dir);

    // ===== 地面（渲染） =====
    const ground = new THREE.Mesh(
        new THREE.BoxGeometry(20, 0.1, 20),
        new THREE.MeshStandardMaterial({ color: 0x2a313d, metalness: 0.1, roughness: 0.9 }),
    );

    ground.position.setY(-0.1)
    scene.add(ground);

    // ===== 测试夹取方块 =====
    const testCube = new THREE.Mesh(
        new THREE.BoxGeometry(0.4, 0.4, 0.4),
        new THREE.MeshStandardMaterial({ color: 0xd97706, metalness: 0.1, roughness: 0.7 })
    );
    testCube.position.set(0, 1.1, 1);
    scene.add(testCube);
    physics.addTestCube(testCube, new THREE.Vector3(0.2, 0.2, 0.2));

    // 3. 加载 GLB 模型部件
    const overlay = document.getElementById('loading-overlay');
    const barFill = document.getElementById('progress-bar-fill');
    const progressText = document.getElementById('progress-text');

    let gripper: any = null;
    try {
        gripper = await loadGripperModel(gripperModelUrl, (percent) => {
            if (barFill) barFill.style.width = `${percent}%`;
            if (progressText) progressText.textContent = `${percent}%`;
        });

        if (!gripper) {
            throw new Error('模型加载失败');
        }

        // 隐藏加载等待
        if (overlay) {
            overlay.classList.add('fade-out');
        }

        // 调整模型初始姿态：让爪头向下
        // 这里可以通过旋转整个 scene 实现
        gripper.scene.rotation.set(Math.PI / 2, 0, 0); // (0, 0, -Math.PI / 2) 可尝试

        gripper.scene.position.set(0, 2, 0);
        // 旋转之后，确保子节点的矩阵被正确更新，因为物理引擎会马上抓取它们的世界坐标
        gripper.scene.updateMatrixWorld(true);

        scene.add(gripper.scene);

        physics.addGripperParts(gripper.parts);

    } catch (e) {
        console.error('模型加载失败:', e);
    }

    // 5. GUI 控制
    initGUI();
    initControlsOverlay();

    // 6. 物理调试渲染器
    const debugLines = new THREE.LineSegments(
        new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({ color: 0xffffff, vertexColors: true })
    );


    scene.add(debugLines);

    // 7. 动画循环
    function animate() {
        requestAnimationFrame(animate);

        // 计算基于帧率的速度积分（这里用固定系数 0.016 简化约等 60fps，也可以引入 THREE.Clock）
        const deltaTime = 0.016;

        guiState.posX += guiState.moveDir.x * guiState.moveSpeed * deltaTime;
        guiState.posY += guiState.moveDir.y * guiState.moveSpeed * deltaTime;
        guiState.posZ += guiState.moveDir.z * guiState.moveSpeed * deltaTime;

        if (gripper) {
            gripper.scene.position.set(0 + guiState.posX, 2 + guiState.posY, 0 + guiState.posZ);
        }

        // a. 更新主动件 tie 的变换
        if (gripper && gripper.parts.left.tie) {
            // 根据速度插值逼近目标角度
            const diff = guiState.targetAngle - guiState.currentAngle;
            if (Math.abs(diff) > 0.001) {
                // speed 映射：数值 1-100 意味着每帧度数在一定范围内（例如 0.05 ~ 5 度）
                const step = (guiState.speed / 100) * 5;
                if (Math.abs(diff) <= step) {
                    guiState.currentAngle = guiState.targetAngle;
                } else {
                    guiState.currentAngle += Math.sign(diff) * step;
                }
            }

            const rad = (guiState.currentAngle * Math.PI) / 180;
            const offset = new THREE.Vector3(guiState.posX, guiState.posY, guiState.posZ);

            // 【关键修改】不再直接设置 tie 的运动学状态旋转角度
            // 而是通过电机控制其目标位置。物理引擎会自动处理碰撞阻力。
            physics.setTieMotorTarget('left', -rad);
            physics.setTieMotorTarget('right', rad);

            // 同步移动负责支撑的隐形地基轴/锚点 (同步平移)
            physics.setGripperTranslation(offset);

            // 同步移动外壳(case/base)等没被额外驱动但必须参与运动学更新的刚体
            if (gripper.parts.base) {
                physics.setPartKinematicState(gripper.parts.base, new THREE.Euler(0, 0, 0), offset);
            }
        }

        // b. 步进物理世界
        physics.step();

        // c. 更新调试辅助线
        if (guiState.showDebugLines) {
            debugLines.visible = true;
            const buffers = physics.world.debugRender();
            debugLines.geometry.setAttribute('position', new THREE.BufferAttribute(buffers.vertices, 3));
            debugLines.geometry.setAttribute('color', new THREE.BufferAttribute(buffers.colors, 4));
        } else {
            debugLines.visible = false;
        }

        // d. 同步物理状态到视觉
        physics.partRigidBodies.forEach((rb, part) => {
            const p = rb.translation();
            const r = rb.rotation();

            // 因为 RigidBody 的 translation 处于世界坐标下，而 part 是嵌套在 gripper.scene（已被旋转）中的，
            // 所以我们必须把物理引擎里纯净的世界坐标，逆向转换回 part 的父级坐标系中！
            if (part.parent) {
                // 1. 位置转换：世界坐标 -> 局部坐标
                const worldPos = new THREE.Vector3(p.x, p.y, p.z);
                part.parent.worldToLocal(worldPos);
                part.position.copy(worldPos);

                // 2. 旋转转换：世界旋转 -> 局部旋转
                const worldQuat = new THREE.Quaternion(r.x, r.y, r.z, r.w);
                const parentWorldQuat = new THREE.Quaternion();
                part.parent.getWorldQuaternion(parentWorldQuat);
                // 局部旋转 = 父级世界旋转的逆 * 自身世界旋转
                const localQuat = parentWorldQuat.invert().multiply(worldQuat);
                part.quaternion.copy(localQuat);
            } else {
                part.position.set(p.x, p.y, p.z);
                part.quaternion.set(r.x, r.y, r.z, r.w);
            }
        });

        controls.update();
        renderer.render(scene, camera);
    }

    animate();

    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });
}

main();
