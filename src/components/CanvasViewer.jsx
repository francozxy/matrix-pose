// src/components/CanvasViewer.jsx
import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';

export default function CanvasViewer({
  vrmList = [],
  activeVrmIndex = 0,
  poseVersion = 0,
  selectedBone = '',
  setSelectedBone,
  gizmoEnabled = true,
  gizmoMode = 'rotate',
  showBoneNodes = true,
  boneOffsets = {},
  onBoneOffsetChange
}) {
  const mountRef = useRef(null);
  const sceneRef = useRef(null);
  const controlsRef = useRef(null);
  const transformControlsRef = useRef(null);
  const boneMarkerGroupRef = useRef(new THREE.Group());

  // Pivote independiente para que el Gizmo nunca colapse la malla del VRM
  const gizmoPivotRef = useRef(new THREE.Object3D());
  const isDraggingRef = useRef(false);

  const dragStartRef = useRef({
    startPivotQuatInv: new THREE.Quaternion(),
    startBoneLocalQuat: new THREE.Quaternion(),
    baseMediaPipeQuatInv: new THREE.Quaternion()
  });

  const propsRef = useRef({
    vrmList,
    activeVrmIndex,
    selectedBone,
    showBoneNodes,
    gizmoEnabled,
    gizmoMode,
    boneOffsets,
    onBoneOffsetChange,
    setSelectedBone
  });

  useEffect(() => {
    propsRef.current = {
      vrmList,
      activeVrmIndex,
      selectedBone,
      showBoneNodes,
      gizmoEnabled,
      gizmoMode,
      boneOffsets,
      onBoneOffsetChange,
      setSelectedBone
    };
  }, [vrmList, activeVrmIndex, selectedBone, showBoneNodes, gizmoEnabled, gizmoMode, boneOffsets, onBoneOffsetChange, setSelectedBone]);

  const boneList = [
    'hips', 'spine', 'chest', 'upperChest', 'neck', 'head',
    'leftShoulder', 'leftUpperArm', 'leftLowerArm', 'leftHand',
    'rightShoulder', 'rightUpperArm', 'rightLowerArm', 'rightHand',
    'leftUpperLeg', 'leftLowerLeg', 'leftFoot',
    'rightUpperLeg', 'rightLowerLeg', 'rightFoot'
  ];

  // 1. Inicializar Escena WebGL una sola vez
  useEffect(() => {
    const container = mountRef.current;
    if (!container) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x161925);
    sceneRef.current = scene;

    const width = container.clientWidth || 300;
    const height = container.clientHeight || 270;

    const camera = new THREE.PerspectiveCamera(40, width / height, 0.05, 100);
    camera.position.set(0, 1.0, 2.35);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.innerHTML = '';
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0.92, 0);
    controls.update();
    controlsRef.current = controls;

    const pivot = gizmoPivotRef.current;
    scene.add(pivot);

    const tc = new TransformControls(camera, renderer.domElement);
    // Tamaño grande y constante para que no se achique al acercar la cámara
    tc.size = 1.35;
    tc.setSpace('local');
    transformControlsRef.current = tc;

    const tcHelper = typeof tc.getHelper === 'function' ? tc.getHelper() : tc;
    scene.add(tcHelper);

    tc.addEventListener('dragging-changed', (event) => {
      isDraggingRef.current = event.value;
      if (controlsRef.current) {
        controlsRef.current.enabled = !event.value;
      }

      const { vrmList: list, activeVrmIndex: idx, selectedBone: sBone, boneOffsets: offsets } = propsRef.current;
      const currentAvatar = list?.[idx];
      const activeVrm = currentAvatar?.vrm;
      if (!activeVrm?.humanoid || !sBone) return;

      const normNode = activeVrm.humanoid.getNormalizedBoneNode(sBone);
      if (!normNode) return;

      if (event.value) {
        // Capturar pose exacta al iniciar el giro para evitar saltos
        dragStartRef.current.startBoneLocalQuat.copy(normNode.quaternion);
        dragStartRef.current.startPivotQuatInv.copy(pivot.quaternion).invert();

        const curOff = offsets?.[sBone] || { rx: 0, ry: 0, rz: 0 };
        const prevOffQuat = new THREE.Quaternion().setFromEuler(
          new THREE.Euler(
            THREE.MathUtils.degToRad(curOff.rx || 0),
            THREE.MathUtils.degToRad(curOff.ry || 0),
            THREE.MathUtils.degToRad(curOff.rz || 0),
            'YXZ'
          )
        );
        const baseMPQuat = normNode.quaternion.clone().multiply(prevOffQuat.invert());
        dragStartRef.current.baseMediaPipeQuatInv.copy(baseMPQuat).invert();
      } else {
        // Guardar offset final al soltar
        const finalOffsetQuat = dragStartRef.current.baseMediaPipeQuatInv
          .clone()
          .multiply(normNode.quaternion);

        const euler = new THREE.Euler().setFromQuaternion(finalOffsetQuat, 'YXZ');
        const prevOff = offsets?.[sBone] || {};

        if (propsRef.current.onBoneOffsetChange) {
          propsRef.current.onBoneOffsetChange(sBone, {
            ...prevOff,
            rx: Math.round(THREE.MathUtils.radToDeg(euler.x)),
            ry: Math.round(THREE.MathUtils.radToDeg(euler.y)),
            rz: Math.round(THREE.MathUtils.radToDeg(euler.z))
          });
        }
      }
    });

    // Rotar en vivo mientras se mueve el dedo
    tc.addEventListener('objectChange', () => {
      if (!isDraggingRef.current) return;
      const { vrmList: list, activeVrmIndex: idx, selectedBone: sBone, boneOffsets: offsets, onBoneOffsetChange: cb } = propsRef.current;
      const currentAvatar = list?.[idx];
      const activeVrm = currentAvatar?.vrm;
      if (!activeVrm?.humanoid || !sBone) return;

      const normNode = activeVrm.humanoid.getNormalizedBoneNode(sBone);
      if (!normNode) return;

      const deltaLocalQuat = new THREE.Quaternion()
        .copy(dragStartRef.current.startPivotQuatInv)
        .multiply(pivot.quaternion);

      normNode.quaternion
        .copy(dragStartRef.current.startBoneLocalQuat)
        .multiply(deltaLocalQuat);

      if (cb) {
        const liveOffsetQuat = dragStartRef.current.baseMediaPipeQuatInv
          .clone()
          .multiply(normNode.quaternion);
        const euler = new THREE.Euler().setFromQuaternion(liveOffsetQuat, 'YXZ');
        const prevOff = offsets?.[sBone] || {};

        cb(sBone, {
          ...prevOff,
          rx: Math.round(THREE.MathUtils.radToDeg(euler.x)),
          ry: Math.round(THREE.MathUtils.radToDeg(euler.y)),
          rz: Math.round(THREE.MathUtils.radToDeg(euler.z))
        });
      }
    });

    // Crear esferas verdes
    const markerGroup = boneMarkerGroupRef.current;
    markerGroup.clear();
    const sphereGeo = new THREE.SphereGeometry(0.022, 14, 14);

    boneList.forEach((bName) => {
      const mat = new THREE.MeshBasicMaterial({
        color: 0x9ece6a,
        depthTest: false,
        transparent: true,
        opacity: 0.88
      });
      const mesh = new THREE.Mesh(sphereGeo, mat);
      mesh.renderOrder = 999;
      mesh.userData = { boneName: bName };
      markerGroup.add(mesh);
    });
    scene.add(markerGroup);

    scene.add(new THREE.GridHelper(4, 10, 0x3b82f6, 0x2a2f45));
    const light = new THREE.DirectionalLight(0xffffff, 1.8);
    light.position.set(1, 3, 3);
    scene.add(light);
    scene.add(new THREE.AmbientLight(0xffffff, 0.85));

    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();
    let downPos = { x: 0, y: 0 };

    const handlePointerDown = (e) => {
      downPos = { x: e.clientX, y: e.clientY };
    };

    const handlePointerUp = (e) => {
      if (isDraggingRef.current) return;
      if (Math.hypot(e.clientX - downPos.x, e.clientY - downPos.y) > 8) return;

      const rect = renderer.domElement.getBoundingClientRect();
      mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

      raycaster.setFromCamera(mouse, camera);
      const intersects = raycaster.intersectObjects(markerGroup.children, true);

      if (intersects.length > 0) {
        const hitBone = intersects[0].object.userData?.boneName;
        if (hitBone && propsRef.current.setSelectedBone) {
          propsRef.current.setSelectedBone(hitBone);
        }
      }
    };

    renderer.domElement.addEventListener('pointerdown', handlePointerDown);
    renderer.domElement.addEventListener('pointerup', handlePointerUp);

    // Usamos performance.now() en lugar de THREE.Clock para eliminar el warning de deprecación
    let lastFrameTime = performance.now();
    let reqId;
    const worldPos = new THREE.Vector3();
    const worldQuat = new THREE.Quaternion();

    const animate = () => {
      reqId = requestAnimationFrame(animate);
      const now = performance.now();
      const delta = Math.min(0.1, (now - lastFrameTime) / 1000);
      lastFrameTime = now;

      const {
        vrmList: list,
        activeVrmIndex: idx,
        selectedBone: sBone,
        showBoneNodes: showNodes
      } = propsRef.current;

      const activeAvatar = list?.[idx];
      const activeVrm = activeAvatar?.vrm;

      if (activeVrm) {
        activeVrm.update(delta);
        activeVrm.scene.updateMatrixWorld(true);

        if (activeVrm.humanoid) {
          markerGroup.children.forEach((marker) => {
            const bName = marker.userData.boneName;
            const normNode = activeVrm.humanoid.getNormalizedBoneNode(bName);
            const rawNode = activeVrm.humanoid.getRawBoneNode
              ? activeVrm.humanoid.getRawBoneNode(bName)
              : normNode;

            if (rawNode && normNode) {
              rawNode.getWorldPosition(worldPos);
              marker.position.copy(worldPos);
              marker.visible = showNodes;

              if (bName === sBone) {
                marker.material.color.setHex(0xe0af68);
                marker.scale.setScalar(0.85);

                pivot.position.copy(worldPos);

                if (!isDraggingRef.current) {
                  normNode.getWorldQuaternion(worldQuat);
                  pivot.quaternion.copy(worldQuat);
                  pivot.scale.set(1, 1, 1);
                }
              } else {
                marker.material.color.setHex(0x9ece6a);
                marker.scale.setScalar(1.0);
              }
            }
          });
        }
      }

      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    const resizeObs = new ResizeObserver(() => {
      if (!container) return;
      const w = container.clientWidth;
      const h = container.clientHeight;
      if (w > 0 && h > 0) {
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        renderer.setSize(w, h);
      }
    });
    resizeObs.observe(container);

    return () => {
      cancelAnimationFrame(reqId);
      resizeObs.disconnect();
      renderer.domElement.removeEventListener('pointerdown', handlePointerDown);
      renderer.domElement.removeEventListener('pointerup', handlePointerUp);
      renderer.dispose();
    };
  }, []);

  // 2. Insertar el modelo VRM en la escena cuando se sube
  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) return;

    const activeScenes = new Set(vrmList.map((v) => v.vrm?.scene).filter(Boolean));
    scene.children.forEach((child) => {
      if (child.userData?.isVRMScene && !activeScenes.has(child)) {
        scene.remove(child);
      }
    });

    vrmList.forEach((vData) => {
      if (vData.vrm && vData.vrm.scene) {
        const vrmScene = vData.vrm.scene;
        vrmScene.userData.isVRMScene = true;
        vrmScene.visible = true;

        if (!scene.children.includes(vrmScene)) {
          scene.add(vrmScene);
        }
      }
    });
  }, [vrmList, activeVrmIndex, poseVersion]);

  // 3. Acoplar el Gizmo solo cuando cambia el hueso seleccionado
  useEffect(() => {
    const tc = transformControlsRef.current;
    const pivot = gizmoPivotRef.current;
    if (!tc || !pivot) return;

    const hasAvatar = Boolean(vrmList?.[activeVrmIndex]?.vrm?.humanoid);
    if (gizmoEnabled && selectedBone && hasAvatar) {
      tc.setMode('rotate');
      if (tc.object !== pivot) {
        tc.attach(pivot);
      }
      tc.enabled = true;
    } else {
      tc.detach();
      tc.enabled = false;
    }
  }, [selectedBone, gizmoEnabled, gizmoMode, activeVrmIndex, vrmList.length]);

  return (
    <div
      ref={mountRef}
      style={{ width: '100%', height: '100%', position: 'relative', overflow: 'hidden', touchAction: 'none' }}
    />
  );
}
