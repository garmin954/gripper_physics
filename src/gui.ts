import GUI from 'lil-gui';

export const guiState = {
    targetAngle: 0,
    currentAngle: 0,
    speed: 20, // 开合速度
    moveSpeed: 2, // 0-20 移动速度
    showDebugLines: false, // 是否显示物理辅助线
    posX: 0,
    posY: 0,
    posZ: 0,
    moveDir: { x: 0, y: 0, z: 0 } // 当前按键产生的移动方向向量
};

export function initGUI() {
    const gui = new GUI();

    const actions = {
        open: () => { guiState.targetAngle = 45; },
        close: () => { guiState.targetAngle = 0; }
    };

    const actionFolder = gui.addFolder('控制');
    actionFolder.add(actions, 'open').name('打开');
    actionFolder.add(actions, 'close').name('闭合');
    actionFolder.add(guiState, 'speed', 1, 100).name('开合速度 (0-100)').step(1);
    actionFolder.add(guiState, 'showDebugLines').name('物理调试线');

    const posFolder = gui.addFolder('机械爪位置');
    posFolder.add(guiState, 'moveSpeed', 0.1, 10).name('移动速度').step(0.1);
    posFolder.add(guiState, 'posX', -10, 10).name('X 位置').step(0.1).listen();
    posFolder.add(guiState, 'posY', -10, 10).name('Y 位置').step(0.1).listen();
    posFolder.add(guiState, 'posZ', -10, 10).name('Z 位置').step(0.1).listen();

    // 键盘监听逻辑：按下增加方向速度，松开方向归零
    const keyMap = { w: false, s: false, a: false, d: false, q: false, e: false };

    const updateMoveDir = () => {
        guiState.moveDir.x = (keyMap.d ? 1 : 0) - (keyMap.a ? 1 : 0);
        guiState.moveDir.y = (keyMap.q ? 1 : 0) - (keyMap.e ? 1 : 0);
        guiState.moveDir.z = (keyMap.s ? 1 : 0) - (keyMap.w ? 1 : 0);
    };

    window.addEventListener('keydown', (event) => {
        const key = event.key.toLowerCase();
        if (key in keyMap) {
            keyMap[key as keyof typeof keyMap] = true;
            updateMoveDir();
        } else if (key === '[') {
            actions.close();
        } else if (key === ']') {
            actions.open();
        }
    });

    window.addEventListener('keyup', (event) => {
        const key = event.key.toLowerCase();
        if (key in keyMap) {
            keyMap[key as keyof typeof keyMap] = false;
            updateMoveDir();
        }
    });


    return gui;
}

/**
 * 初始化左上角的操作提示面板
 */
export function initControlsOverlay() {
    const overlay = document.createElement('div');
    overlay.className = 'controls-overlay';
    overlay.innerHTML = `
        <h3>操作指南</h3>
        <div class="control-item"><span class="control-key">W</span><span class="control-key">S</span><span class="control-desc">前后移动</span></div>
        <div class="control-item"><span class="control-key">A</span><span class="control-key">D</span><span class="control-desc">左右移动</span></div>
        <div class="control-item"><span class="control-key">Q</span><span class="control-key">E</span><span class="control-desc">上下移动</span></div>
        <div class="control-item"><span class="control-key">[</span><span class="control-key">]</span><span class="control-desc">夹爪闭合 / 打开</span></div>
        <div class="control-item" style="margin-top: 12px; font-size: 12px; opacity: 0.6;">
            <span>鼠标右键旋转视角，滚轮缩放</span>
        </div>
    `;
    document.body.appendChild(overlay);
}
