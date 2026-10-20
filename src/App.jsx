// src/App.jsx
import React, { useState, useRef, useEffect } from 'react';
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { VRMLoaderPlugin, VRMUtils } from '@pixiv/three-vrm';
import { solvePoseToVRM, BONE_LIST } from './kinematics';
import { buildVRMABinary, buildVMDBinary, bakeOffsetsIntoFrames } from './exporters';
import PosingPanel from './components/PosingPanel';
import CanvasViewer from './components/CanvasViewer';

const DB_NAME = 'MatrixPoseDB';
const STORE_NAME = 'vrmStore';
const DB_VERSION = 1;

function openVrmDB() {
  return new Promise((resolve, reject) => {
    const request = indexedDB.open(DB_NAME, DB_VERSION);
    request.onerror = () => reject(request.error);
    request.onsuccess = () => resolve(request.result);
    request.onupgradeneeded = (e) => {
      const db = e.target.result;
      if (!db.objectStoreNames.contains(STORE_NAME)) {
        db.createObjectStore(STORE_NAME);
      }
    };
  });
}

async function saveVrmToIndexedDB(fileBlob, fileName) {
  try {
    const db = await openVrmDB();
    const tx = db.transaction(STORE_NAME, 'readwrite');
    const store = tx.objectStore(STORE_NAME);
    store.put({ blob: fileBlob, name: fileName }, 'last_vrm');
    return new Promise((resolve, reject) => {
      tx.oncomplete = () => resolve(true);
      tx.onerror = () => reject(tx.error);
    });
  } catch (err) {
    console.warn('No se pudo guardar en IndexedDB:', err);
  }
}

async function loadVrmFromIndexedDB() {
  try {
    const db = await openVrmDB();
    return new Promise((resolve, reject) => {
      const tx = db.transaction(STORE_NAME, 'readonly');
      const store = tx.objectStore(STORE_NAME);
      const request = store.get('last_vrm');
      request.onsuccess = () => resolve(request.result || null);
      request.onerror = () => reject(request.error);
    });
  } catch (err) {
    console.warn('No se pudo leer de IndexedDB:', err);
    return null;
  }
}

function loadMediaPipePoseScript() {
  if (window.Pose) return Promise.resolve(window.Pose);
  return new Promise((resolve, reject) => {
    if (document.querySelector('#mediapipe-pose-script')) {
      const check = setInterval(() => {
        if (window.Pose) {
          clearInterval(check);
          resolve(window.Pose);
        }
      }, 100);
      return;
    }
    const script = document.createElement('script');
    script.id = 'mediapipe-pose-script';
    script.src = 'https://cdn.jsdelivr.net/npm/@mediapipe/pose@0.5.1675469404/pose.js';
    script.crossOrigin = 'anonymous';
    script.onload = () => resolve(window.Pose);
    script.onerror = reject;
    document.head.appendChild(script);
  });
}

const POSE_CONNECTIONS = [
  [0, 2], [0, 5], [2, 5], [9, 10],
  [11, 12], [11, 13], [13, 15], [12, 14], [14, 16],
  [11, 23], [12, 24], [23, 24],
  [23, 25], [25, 27], [24, 26], [26, 28]
];

export default function App() {
  const [videoUrl, setVideoUrl] = useState(null);
  const [vrmName, setVrmName] = useState('Ningún Avatar');
  const [isRecording, setIsRecording] = useState(false);
  const [isPlayingPreview, setIsPlayingPreview] = useState(false);
  const [recordedFrames, setRecordedFrames] = useState([]);
  const [statusText, setStatusText] = useState('Cargando motor de pose...');
  const [fps, setFps] = useState(0);
  const [controlsOnTop, setControlsOnTop] = useState(false);
  const [copyFeedback, setCopyFeedback] = useState('');
  const [exportLogs, setExportLogs] = useState([]);
  const [exportFlipVrm0, setExportFlipVrm0] = useState(true);
  const [keepOffsetsOnRecord, setKeepOffsetsOnRecord] = useState(false);

  const [vrmList, setVrmList] = useState([]);
  const [activeVrmIndex, setActiveVrmIndex] = useState(0);
  const [selectedBone, setSelectedBone] = useState('');
  const [showBoneNodes, setShowBoneNodes] = useState(true);
  const [boneOffsets, setBoneOffsets] = useState({});
  const [gizmoEnabled, setGizmoEnabled] = useState(true);
  const [gizmoMode, setGizmoMode] = useState('rotate');
  const [poseVersion, setPoseVersion] = useState(0);

  const [config, setConfig] = useState({
    mirror: true,
    vrm0Mode: false,
    smoothing: 0.20,
    headSensitivity: 1.0,
    armVerticalOffset: -10,
    legSpreadOffset: 2,
    depthScale: 0.3,
    hipMoveScale: 0.7,
    invertArmZ: false,
    invertShoulderZ: false,
    unlockRotation: false
  });

  const videoRef = useRef(null);
  const canvas2DRef = useRef(null);
  const poseInstanceRef = useRef(null);
  const prevPoseRef = useRef(null);
  const isRecRef = useRef(false);
  const isPreviewRef = useRef(false);
  const configRef = useRef(config);
  const isProcessingRef = useRef(false);
  const lastProcessTimeRef = useRef(0);

  const vrmListRef = useRef(vrmList);
  const activeVrmIndexRef = useRef(activeVrmIndex);
  const boneOffsetsRef = useRef(boneOffsets);

  useEffect(() => { configRef.current = config; }, [config]);
  useEffect(() => { vrmListRef.current = vrmList; }, [vrmList]);
  useEffect(() => { activeVrmIndexRef.current = activeVrmIndex; }, [activeVrmIndex]);
  useEffect(() => { boneOffsetsRef.current = boneOffsets; }, [boneOffsets]);

  const applySolvedPose = (solved, customOffsetsOverride = null) => {
    const currentAvatar = vrmListRef.current?.[activeVrmIndexRef.current];
    const loadedVrm = currentAvatar?.vrm;
    if (!loadedVrm || !loadedVrm.humanoid) return null;

    const humanoid = loadedVrm.humanoid;
    const activeOffsets = customOffsetsOverride || boneOffsetsRef.current || {};
    const finalQuats = {};
    let finalHipsPos = [0, 0, 0];

    try {
      const hipsNode = humanoid.getNormalizedBoneNode('hips');
      const baseHips = solved?.hipsPos || [0, 0, 0];
      const hipOff = activeOffsets.hips || {};
      finalHipsPos = [
        baseHips[0] + (hipOff.px || 0),
        baseHips[1] + (hipOff.py || 0),
        baseHips[2] + (hipOff.pz || 0)
      ];

      if (hipsNode) {
        hipsNode.position.set(
          finalHipsPos[0],
          0.92 + finalHipsPos[1],
          finalHipsPos[2]
        );
      }

      BONE_LIST.forEach((boneName) => {
        const node = humanoid.getNormalizedBoneNode(boneName);
        if (!node) return;

        if (solved && solved.localQuats && solved.localQuats[boneName]) {
          node.quaternion.fromArray(solved.localQuats[boneName]);
        } else if (!solved && currentAvatar.baseQuats?.[boneName]) {
          node.quaternion.copy(currentAvatar.baseQuats[boneName]);
        }

        const custom = activeOffsets[boneName];
        if (custom && ((custom.rx || 0) !== 0 || (custom.ry || 0) !== 0 || (custom.rz || 0) !== 0)) {
          const addEuler = new THREE.Euler(
            THREE.MathUtils.degToRad(custom.rx || 0),
            THREE.MathUtils.degToRad(custom.ry || 0),
            THREE.MathUtils.degToRad(custom.rz || 0),
            'YXZ'
          );
          const addQuat = new THREE.Quaternion().setFromEuler(addEuler);
          node.quaternion.multiply(addQuat);
        }

        finalQuats[boneName] = node.quaternion.toArray();
      });

      return { hipsPos: finalHipsPos, localQuats: finalQuats };
    } catch (err) {
      console.error('Error aplicando pose:', err);
      return null;
    }
  };

  const getSyncedOffsetsFromDeelia = () => {
    const merged = { ...boneOffsetsRef.current };
    const currentAvatar = vrmListRef.current?.[activeVrmIndexRef.current];
    const humanoid = currentAvatar?.vrm?.humanoid;
    const baseSolved = prevPoseRef.current;

    if (humanoid && baseSolved?.localQuats) {
      BONE_LIST.forEach((bName) => {
        const node = humanoid.getNormalizedBoneNode(bName);
        const baseArr = baseSolved.localQuats[bName];
        if (node && baseArr) {
          const baseQ = new THREE.Quaternion(baseArr[0], baseArr[1], baseArr[2], baseArr[3]).normalize();
          const liveQ = node.quaternion.clone().normalize();
          const diffQ = baseQ.clone().invert().multiply(liveQ);
          const e = new THREE.Euler().setFromQuaternion(diffQ, 'YXZ');
          const rx = Math.round(THREE.MathUtils.radToDeg(e.x));
          const ry = Math.round(THREE.MathUtils.radToDeg(e.y));
          const rz = Math.round(THREE.MathUtils.radToDeg(e.z));

          if (Math.abs(rx) > 1 || Math.abs(ry) > 1 || Math.abs(rz) > 1) {
            merged[bName] = {
              ...(merged[bName] || {}),
              rx,
              ry,
              rz
            };
          }
        }
      });
    }
    return merged;
  };

  const handleBoneOffsetUpdate = (boneName, newOffset) => {
    const cleanOffset = {
      rx: Number.isFinite(newOffset.rx) ? newOffset.rx : 0,
      ry: Number.isFinite(newOffset.ry) ? newOffset.ry : 0,
      rz: Number.isFinite(newOffset.rz) ? newOffset.rz : 0,
      px: Number.isFinite(newOffset.px) ? newOffset.px : 0,
      py: Number.isFinite(newOffset.py) ? newOffset.py : 0,
      pz: Number.isFinite(newOffset.pz) ? newOffset.pz : 0
    };

    const updatedOffsets = {
      ...boneOffsetsRef.current,
      [boneName]: cleanOffset
    };
    boneOffsetsRef.current = updatedOffsets;
    setBoneOffsets(updatedOffsets);

    const currentAvatar = vrmListRef.current?.[activeVrmIndexRef.current];
    if (currentAvatar) {
      currentAvatar.offsets = updatedOffsets;
    }

    applySolvedPose(prevPoseRef.current, updatedOffsets);
  };

  // Cargar automáticamente el último VRM guardado en IndexedDB al abrir la app
  useEffect(() => {
    loadVrmFromIndexedDB().then((saved) => {
      if (saved && saved.blob) {
        setStatusText(`Cargando ${saved.name} desde la memoria local...`);
        const url = URL.createObjectURL(saved.blob);
        loadVrmFromUrl(url, saved.name);
      }
    });
  }, []);

  const loadVrmFromUrl = (url, fileName) => {
    const loader = new GLTFLoader();
    loader.register((parser) => new VRMLoaderPlugin(parser));

    loader.load(
      url,
      (gltf) => {
        const loadedVrm = gltf.userData.vrm;
        if (!loadedVrm) {
          setStatusText('El archivo no es un VRM válido');
          URL.revokeObjectURL(url);
          return;
        }

        VRMUtils.rotateVRM0(loadedVrm);

        const defaultPositions = {};
        const baseQuats = {};
        if (loadedVrm.humanoid) {
          BONE_LIST.forEach((bName) => {
            const node = loadedVrm.humanoid.getNormalizedBoneNode(bName);
            if (node) {
              defaultPositions[bName] = node.position.clone();
              baseQuats[bName] = node.quaternion.clone();
            }
          });
        }

        const newAvatarData = {
          vrm: loadedVrm,
          scene: loadedVrm.scene,
          offsets: {},
          defaultPositions,
          baseQuats
        };

        vrmListRef.current = [newAvatarData];
        setVrmList([newAvatarData]);
        setActiveVrmIndex(0);
        setBoneOffsets({});
        boneOffsetsRef.current = {};

        const isVrm0 = String(loadedVrm.meta?.metaVersion) === '0';
        setVrmName(`${fileName} (${isVrm0 ? 'VRM 0.x' : 'VRM 1.0'})`);
        setStatusText(`✅ Avatar ${fileName} listo y guardado`);
        URL.revokeObjectURL(url);
      },
      undefined,
      (err) => {
        console.error(err);
        setStatusText('Error al cargar el modelo .VRM');
        URL.revokeObjectURL(url);
      }
    );
  };

  const handleVRMUpload = (e) => {
    const file = e.target.files?.[0];
    if (!file) return;

    // Guardar en IndexedDB para persistencia sin límite de tamaño
    saveVrmToIndexedDB(file, file.name);

    const url = URL.createObjectURL(file);
    setStatusText(`Cargando avatar ${file.name}...`);
    loadVrmFromUrl(url, file.name);
  };

  const previewExportOnDeelia = () => {
    if (recordedFrames.length === 0 || isPlayingPreview) return;
    if (videoRef.current) videoRef.current.pause();
    isRecRef.current = false;
    setIsRecording(false);

    const syncedOffsets = getSyncedOffsetsFromDeelia();
    boneOffsetsRef.current = syncedOffsets;
    setBoneOffsets(syncedOffsets);

    const logs = [];
    const baked = bakeOffsetsIntoFrames(
      recordedFrames,
      syncedOffsets,
      { exportFlipVrm0: false },
      (msg) => logs.push(msg)
    );
    logs.push(`▶️ Reproduciendo ${baked.length} frames en Deelia...`);
    setExportLogs(logs);

    setIsPlayingPreview(true);
    isPreviewRef.current = true;

    let idx = 0;
    const interval = setInterval(() => {
      if (!isPreviewRef.current || idx >= baked.length) {
        clearInterval(interval);
        setIsPlayingPreview(false);
        isPreviewRef.current = false;
        setExportLogs((prev) => [...prev, `✅ Fin de previsualización.`]);
        return;
      }

      const frame = baked[idx];
      const currentAvatar = vrmListRef.current?.[activeVrmIndexRef.current];
      const humanoid = currentAvatar?.vrm?.humanoid;
      if (humanoid) {
        const hipsNode = humanoid.getNormalizedBoneNode('hips');
        if (hipsNode && frame.hipsPos) {
          hipsNode.position.set(frame.hipsPos[0], 0.92 + frame.hipsPos[1], frame.hipsPos[2]);
        }
        BONE_LIST.forEach((bName) => {
          const node = humanoid.getNormalizedBoneNode(bName);
          if (node && frame.localQuats?.[bName]) {
            node.quaternion.fromArray(frame.localQuats[bName]);
          }
        });
      }
      idx++;
    }, 33);
  };

  const copyBoneAngleData = (mode = 'selected') => {
    const currentAvatar = vrmListRef.current?.[activeVrmIndexRef.current];
    const humanoid = currentAvatar?.vrm?.humanoid;
    const syncedOffsets = getSyncedOffsetsFromDeelia();

    const getBoneReport = (bName) => {
      const off = syncedOffsets?.[bName] || { rx: 0, ry: 0, rz: 0 };
      let worldEuler = { x: 0, y: 0, z: 0 };

      if (humanoid) {
        const node = humanoid.getNormalizedBoneNode(bName);
        if (node) {
          const e = new THREE.Euler().setFromQuaternion(node.quaternion, 'YXZ');
          worldEuler = {
            x: Math.round(THREE.MathUtils.radToDeg(e.x)),
            y: Math.round(THREE.MathUtils.radToDeg(e.y)),
            z: Math.round(THREE.MathUtils.radToDeg(e.z))
          };
        }
      }

      return {
        bone: bName,
        offset_manual: { X: off.rx || 0, Y: off.ry || 0, Z: off.rz || 0 },
        rotacion_total_actual: { X: worldEuler.x, Y: worldEuler.y, Z: worldEuler.z }
      };
    };

    let textToCopy = '';

    if (mode === 'selected' && selectedBone) {
      const r = getBoneReport(selectedBone);
      textToCopy =
        `🦴 Hueso: ${r.bone}\n` +
        `📐 Ajuste Manual -> X: ${r.offset_manual.X}°, Y: ${r.offset_manual.Y}°, Z: ${r.offset_manual.Z}°\n` +
        `🎯 Rotación Total -> X: ${r.rotacion_total_actual.X}°, Y: ${r.rotacion_total_actual.Y}°, Z: ${r.rotacion_total_actual.Z}°\n` +
        `JSON: ${JSON.stringify(r)}`;
    } else {
      const modifiedKeys = Object.keys(syncedOffsets).filter((k) => {
        const o = syncedOffsets[k];
        return o && (o.rx !== 0 || o.ry !== 0 || o.rz !== 0);
      });

      const keysToExport = modifiedKeys.length > 0
        ? modifiedKeys
        : [
            'leftShoulder', 'leftUpperArm', 'leftLowerArm',
            'rightShoulder', 'rightUpperArm', 'rightLowerArm',
            'leftUpperLeg', 'leftLowerLeg', 'rightUpperLeg', 'rightLowerLeg'
          ];

      const lines = keysToExport.map((k) => {
        const r = getBoneReport(k);
        return `• ${r.bone}: Offset(X:${r.offset_manual.X}°, Y:${r.offset_manual.Y}°, Z:${r.offset_manual.Z}°) | Total(X:${r.rotacion_total_actual.X}°, Y:${r.rotacion_total_actual.Y}°, Z:${r.rotacion_total_actual.Z}°)`;
      });

      textToCopy = `📊 Reporte de Ángulos MatrixPose:\n` + lines.join('\n');
    }

    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(textToCopy).then(() => {
        setCopyFeedback('✅ ¡Ángulos X, Y, Z copiados al portapapeles!');
        setTimeout(() => setCopyFeedback(''), 3000);
      });
    } else {
      const ta = document.createElement('textarea');
      ta.value = textToCopy;
      document.body.appendChild(ta);
      ta.select();
      document.execCommand('copy');
      document.body.removeChild(ta);
      setCopyFeedback('✅ ¡Ángulos X, Y, Z copiados!');
      setTimeout(() => setCopyFeedback(''), 3000);
    }
  };

  const resetSelectedBone = () => {
    if (!selectedBone) return;
    handleBoneOffsetUpdate(selectedBone, { rx: 0, ry: 0, rz: 0, px: 0, py: 0, pz: 0 });
    setCopyFeedback(`🔄 Hueso ${selectedBone} reseteado a 0°`);
    setTimeout(() => setCopyFeedback(''), 2000);
  };

  const resetAllBones = () => {
    boneOffsetsRef.current = {};
    setBoneOffsets({});
    const currentAvatar = vrmListRef.current?.[activeVrmIndexRef.current];
    if (currentAvatar) currentAvatar.offsets = {};
    applySolvedPose(prevPoseRef.current, {});
    setCopyFeedback('🔄 Todos los huesos reseteados a 0°');
    setTimeout(() => setCopyFeedback(''), 2000);
  };

  // Pasa un filtro de suavizado temporal (ventana móvil) sobre los frames ya grabados
  // para eliminar picos de ruido o temblores sin perder el ritmo del baile
  const applyPostSmoothingToFrames = (passes = 2) => {
    if (recordedFrames.length < 3) return;

    let current = recordedFrames.map((f) => ({
      ...f,
      hipsPos: [...(f.hipsPos || [0, 0, 0])],
      rawHipsPos: [...(f.rawHipsPos || f.hipsPos || [0, 0, 0])],
      localQuats: { ...(f.localQuats || {}) },
      rawLocalQuats: { ...(f.rawLocalQuats || f.localQuats || {}) }
    }));

    const blendFactor = THREE.MathUtils.clamp(config.smoothing || 0.35, 0.15, 0.75);

    for (let p = 0; p < passes; p++) {
      const next = current.map((f) => ({
        ...f,
        hipsPos: [...f.hipsPos],
        rawHipsPos: [...f.rawHipsPos],
        localQuats: { ...f.localQuats },
        rawLocalQuats: { ...f.rawLocalQuats }
      }));

      for (let i = 1; i < current.length - 1; i++) {
        const prevF = current[i - 1];
        const currF = current[i];
        const nextF = current[i + 1];

        for (let axis = 0; axis < 3; axis++) {
          const avgPos = (prevF.rawHipsPos[axis] + nextF.rawHipsPos[axis]) * 0.5;
          next[i].rawHipsPos[axis] = currF.rawHipsPos[axis] * (1 - blendFactor) + avgPos * blendFactor;
          next[i].hipsPos[axis] = next[i].rawHipsPos[axis];
        }

        BONE_LIST.forEach((bName) => {
          const qPrev = new THREE.Quaternion(...(prevF.rawLocalQuats[bName] || [0, 0, 0, 1]));
          const qCurr = new THREE.Quaternion(...(currF.rawLocalQuats[bName] || [0, 0, 0, 1]));
          const qNext = new THREE.Quaternion(...(nextF.rawLocalQuats[bName] || [0, 0, 0, 1]));

          if (qPrev.dot(qNext) < 0) {
            qNext.set(-qNext.x, -qNext.y, -qNext.z, -qNext.w);
          }
          const qMid = qPrev.clone().slerp(qNext, 0.5).normalize();

          if (qCurr.dot(qMid) < 0) {
            qMid.set(-qMid.x, -qMid.y, -qMid.z, -qMid.w);
          }
          qCurr.slerp(qMid, blendFactor).normalize();

          const arr = [qCurr.x, qCurr.y, qCurr.z, qCurr.w];
          next[i].rawLocalQuats[bName] = arr;
          next[i].localQuats[bName] = arr;
        });
      }
      current = next;
    }

    setRecordedFrames(current);
    setExportLogs((prev) => [
      ...prev,
      `✨ Ruido suavizado en ${current.length} frames (Intensidad: ${Math.round((config.smoothing || 0.2) * 100)}%).`
    ]);
    setCopyFeedback('✨ ¡Animación suavizada!');
    setTimeout(() => setCopyFeedback(''), 2500);
  };

  useEffect(() => {
    let mounted = true;

    loadMediaPipePoseScript()
      .then((PoseConstructor) => {
        if (!mounted || !PoseConstructor) return;

        const pose = new PoseConstructor({
          locateFile: (file) => `https://cdn.jsdelivr.net/npm/@mediapipe/pose@0.5.1675469404/${file}`
        });

        pose.setOptions({
          modelComplexity: 1,
          smoothLandmarks: true,
          minDetectionConfidence: 0.4,
          minTrackingConfidence: 0.4
        });

        let lastTime = performance.now();

        pose.onResults((results) => {
          isProcessingRef.current = false;
          if (isPreviewRef.current) return;

          const now = performance.now();
          setFps(Math.round(1000 / Math.max(16, now - lastTime)));
          lastTime = now;

          const ctx = canvas2DRef.current?.getContext('2d');
          if (ctx && canvas2DRef.current) {
            const cw = canvas2DRef.current.width;
            const ch = canvas2DRef.current.height;
            ctx.clearRect(0, 0, cw, ch);

            if (results.poseLandmarks) {
              ctx.strokeStyle = '#f43f5e';
              ctx.lineWidth = 2.5;
              POSE_CONNECTIONS.forEach(([i, j]) => {
                const a = results.poseLandmarks[i];
                const b = results.poseLandmarks[j];
                if (a && b) {
                  ctx.beginPath();
                  ctx.moveTo(a.x * cw, a.y * ch);
                  ctx.lineTo(b.x * cw, b.y * ch);
                  ctx.stroke();
                }
              });

              ctx.fillStyle = '#38bdf8';
              results.poseLandmarks.forEach((lm) => {
                ctx.beginPath();
                ctx.arc(lm.x * cw, lm.y * ch, 4, 0, Math.PI * 2);
                ctx.fill();
              });
            }
          }

          if (results.poseLandmarks) {
            const vid = videoRef.current;
            const aspect =
              vid && vid.videoWidth && vid.videoHeight
                ? vid.videoWidth / vid.videoHeight
                : 0.6;

            const solved = solvePoseToVRM(
              results.poseWorldLandmarks,
              results.poseLandmarks,
              prevPoseRef.current,
              configRef.current,
              aspect
            );
            prevPoseRef.current = solved;

            const combinedPose = applySolvedPose(solved);

            if (isRecRef.current && videoRef.current) {
              const videoTimeMs = videoRef.current.currentTime * 1000;
              setRecordedFrames((prev) => [
                ...prev,
                {
                  time: videoTimeMs,
                  rawHipsPos: solved.hipsPos,
                  rawLocalQuats: solved.localQuats,
                  hipsPos: combinedPose ? combinedPose.hipsPos : solved.hipsPos,
                  localQuats: combinedPose ? combinedPose.localQuats : solved.localQuats
                }
              ]);
            }
          }
        });

        poseInstanceRef.current = pose;
        setStatusText('MediaPipe activo. ¡Listo para bailar!');
      })
      .catch((err) => {
        console.error(err);
        setStatusText('Error al cargar MediaPipe');
      });

    return () => {
      mounted = false;
    };
  }, []);

  const handleVideoUpload = (e) => {
    const file = e.target.files?.[0];
    if (!file) return;
    if (videoUrl) URL.revokeObjectURL(videoUrl);
    const url = URL.createObjectURL(file);
    setVideoUrl(url);
    setRecordedFrames([]);
    prevPoseRef.current = null;
    isProcessingRef.current = false;
    setStatusText(`Video listo: ${file.name}`);

    setTimeout(() => {
      const vid = videoRef.current;
      if (vid && poseInstanceRef.current) {
        vid.currentTime = 0.05;
        isProcessingRef.current = false;
        vid.play().then(() => {
          vid.pause();
          poseInstanceRef.current.send({ image: vid });
        }).catch(() => {});
      }
    }, 400);
  };

  useEffect(() => {
    let active = true;
    const processFrame = async () => {
      if (!active) return;
      const vid = videoRef.current;
      const now = performance.now();

      if (isProcessingRef.current && now - lastProcessTimeRef.current > 800) {
        isProcessingRef.current = false;
      }

      if (vid && poseInstanceRef.current && vid.readyState >= 2 && !vid.paused && !vid.ended && !isPreviewRef.current) {
        if (!isProcessingRef.current) {
          isProcessingRef.current = true;
          lastProcessTimeRef.current = now;
          try {
            await poseInstanceRef.current.send({ image: vid });
          } catch (e) {
            isProcessingRef.current = false;
          }
        }
      }
      requestAnimationFrame(processFrame);
    };
    requestAnimationFrame(processFrame);
    return () => { active = false; };
  }, []);

  const toggleRecording = () => {
    const vid = videoRef.current;
    if (!vid) return;

    isPreviewRef.current = false;
    setIsPlayingPreview(false);

    if (!isRecording) {
      if (!keepOffsetsOnRecord) {
        boneOffsetsRef.current = {};
        setBoneOffsets({});
        const currentAvatar = vrmListRef.current?.[activeVrmIndexRef.current];
        if (currentAvatar) currentAvatar.offsets = {};
      } else {
        const synced = getSyncedOffsetsFromDeelia();
        boneOffsetsRef.current = synced;
        setBoneOffsets(synced);
      }

      setSelectedBone('');
      setRecordedFrames([]);
      setExportLogs([
        keepOffsetsOnRecord
          ? `🔴 Grabando desde 0.00s manteniendo offsets globales...`
          : `🔴 Grabando animación limpia desde 0.00s (sin poses trabadas)...`
      ]);
      prevPoseRef.current = null;
      isProcessingRef.current = false;
      lastProcessTimeRef.current = performance.now();
      isRecRef.current = true;
      setIsRecording(true);

      vid.currentTime = 0;
      const playPromise = vid.play();
      if (playPromise !== undefined) {
        playPromise.catch((err) => {
          console.warn('Reintentando play de video:', err);
          setTimeout(() => vid.play().catch(() => {}), 100);
        });
      }
      setStatusText('🔴 Capturando movimiento...');
    } else {
      isRecRef.current = false;
      isProcessingRef.current = false;
      setIsRecording(false);
      vid.pause();
      setExportLogs((prev) => [
        ...prev,
        `⏹ Captura detenida (${recordedFrames.length}f).`
      ]);
      setStatusText(`✅ Pausado (${recordedFrames.length} frames) - Toca un hueso verde para ajustar`);
    }
  };

  const downloadFile = (blob, filename) => {
    if (!blob) return;
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    a.click();
    URL.revokeObjectURL(url);
  };

  const currentSelOffset = selectedBone ? (boneOffsets[selectedBone] || { rx: 0, ry: 0, rz: 0 }) : null;

  return (
    <div className="mp-container">
      <style>{`
        * { box-sizing: border-box; }
        body { margin: 0; background: #0f111a; color: #f1f5f9; font-family: system-ui, sans-serif; }
        .mp-container { max-width: 960px; margin: 0 auto; padding: 12px; display: flex; flex-direction: column; gap: 12px; }
        .mp-header { background: #1c2030; padding: 10px 14px; border-radius: 12px; display: flex; justify-content: space-between; align-items: center; border: 1px solid #2e344d; }
        .mp-title { font-size: 15px; font-weight: 700; color: #38bdf8; margin: 0; }
        .mp-badge { background: #262b40; padding: 4px 10px; border-radius: 8px; font-size: 12px; color: #94a3b8; }
        
        .mp-main-layout {
          display: flex;
          flex-direction: ${controlsOnTop ? 'column-reverse' : 'column'};
          gap: 12px;
        }
        
        .mp-viewers {
          display: flex;
          flex-direction: column;
          gap: 10px;
        }
        .mp-video-box { 
          position: relative; 
          background: #000; 
          border-radius: 10px; 
          overflow: hidden; 
          height: 270px; 
          width: 100%; 
          display: flex;
          align-items: center;
          justify-content: center;
        }
        .mp-video-box video { 
          height: 100%; 
          width: auto; 
          object-fit: contain; 
        }
        .mp-video-box canvas { 
          position: absolute; 
          height: 100%; 
          width: auto; 
          aspect-ratio: attr(width) / attr(height);
          pointer-events: none; 
        }
        .mp-3d-box { width: 100%; height: 270px; border-radius: 10px; overflow: hidden; background: #161925; position: relative; }

        @media (orientation: landscape) {
          .mp-main-layout {
            display: grid;
            grid-template-columns: 1.3fr 1fr;
            align-items: start;
          }
          .mp-viewers {
            flex-direction: row;
          }
          .mp-video-box, .mp-3d-box {
            height: 300px;
            flex: 1;
          }
        }

        .mp-card { background: #1c2030; border-radius: 12px; padding: 12px; border: 1px solid #2e344d; }
        .mp-btn-row { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 8px; }
        .mp-btn { flex: 1; padding: 10px 12px; border-radius: 8px; border: none; font-weight: 600; font-size: 13px; cursor: pointer; text-align: center; color: #fff; }
        .mp-btn-blue { background: #2563eb; }
        .mp-btn-amber { background: #d97706; }
        .mp-btn-red { background: #e11d48; }
        .mp-btn-green { background: #10b981; }
        .mp-btn-purple { background: #7c3aed; }
        .mp-btn:disabled { opacity: 0.4; cursor: not-allowed; }
        .mp-angle-bar {
          margin-top: 10px;
          padding: 10px;
          background: #141724;
          border: 1px solid #334155;
          border-radius: 10px;
          display: flex;
          flex-direction: column;
          gap: 8px;
        }
        .mp-xyz-pills {
          display: flex;
          gap: 6px;
          justify-content: space-between;
        }
        .mp-pill {
          flex: 1;
          background: #1e293b;
          padding: 6px 8px;
          border-radius: 6px;
          text-align: center;
          font-size: 12px;
          font-family: monospace;
          font-weight: 700;
        }
        .mp-log-box {
          margin-top: 10px;
          background: #090b10;
          border: 1px solid #1e293b;
          border-radius: 8px;
          padding: 8px 10px;
          font-family: monospace;
          font-size: 11px;
          color: #a7f3d0;
          max-height: 185px;
          overflow-y: auto;
          line-height: 1.45;
        }
      `}</style>

      <div className="mp-header">
        <h1 className="mp-title">⚡ MatrixPose Mobile V5.5</h1>
        <div style={{ display: 'flex', gap: '6px', alignItems: 'center' }}>
          <button
            onClick={() => setControlsOnTop(!controlsOnTop)}
            style={{ background: '#262b40', color: '#38bdf8', border: '1px solid #38bdf8', borderRadius: '6px', padding: '4px 8px', fontSize: '11px', cursor: 'pointer' }}
          >
            ⇅ {controlsOnTop ? 'Menú Abajo' : 'Menú Arriba'}
          </button>
          <span className="mp-badge">{fps} FPS | {recordedFrames.length}f</span>
        </div>
      </div>

      <div className="mp-main-layout">
        <div className="mp-card">
          <div className="mp-viewers">
            <div className="mp-video-box">
              <video
                ref={videoRef}
                src={videoUrl}
                playsInline
                muted
                onEnded={() => {
                  if (isRecRef.current) toggleRecording();
                }}
              />
              <canvas ref={canvas2DRef} width={360} height={480} />
            </div>

            <div className="mp-3d-box">
              <CanvasViewer
                vrmList={vrmList}
                activeVrmIndex={activeVrmIndex}
                poseVersion={poseVersion}
                selectedBone={selectedBone}
                setSelectedBone={setSelectedBone}
                gizmoEnabled={gizmoEnabled}
                gizmoMode={gizmoMode}
                showBoneNodes={showBoneNodes}
                boneOffsets={boneOffsets}
                onBoneOffsetChange={handleBoneOffsetUpdate}
              />
            </div>
          </div>

          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '12px', color: '#94a3b8', margin: '8px 0 4px' }}>
            <span>{statusText}</span>
            <span style={{ color: '#fbbf24' }}>🎭 {vrmName}</span>
          </div>

          <div className="mp-btn-row">
            <label className="mp-btn mp-btn-blue">
              📂 Video MP4
              <input type="file" accept="video/*" onChange={handleVideoUpload} style={{ display: 'none' }} />
            </label>
            <label className="mp-btn mp-btn-amber">
              🎭 Subir .VRM
              <input type="file" accept=".vrm" onChange={handleVRMUpload} style={{ display: 'none' }} />
            </label>
            <button
              className={`mp-btn ${isRecording ? 'mp-btn-red' : 'mp-btn-green'}`}
              disabled={!videoUrl}
              onClick={toggleRecording}
            >
              {isRecording ? '⏹ Detener' : '⏺ Grabar'}
            </button>
          </div>

          <div className="mp-btn-row">
            {[0.25, 0.5, 1.0].map((rate) => (
              <button
                key={rate}
                className="mp-btn"
                style={{ background: '#262b40', padding: '6px' }}
                onClick={() => {
                  if (videoRef.current) videoRef.current.playbackRate = rate;
                }}
              >
                Velocidad {rate}x
              </button>
            ))}
          </div>

       {/* Checkbox / Botón rápido para Desbloquear o Bloquear el Límite de Rotación en vivo */}
       <div
         onClick={() => setConfig((prev) => ({ ...prev, unlockRotation: !prev.unlockRotation }))}
         style={{
           marginTop: '8px',
           padding: '10px 12px',
           borderRadius: '10px',
           cursor: 'pointer',
           display: 'flex',
           alignItems: 'center',
           justifyContent: 'space-between',
           background: config.unlockRotation ? '#3b0764' : '#1e293b',
           border: config.unlockRotation ? '1px solid #c084fc' : '1px solid #334155',
           transition: 'all 0.15s ease'
         }}
       >
         <label style={{ display: 'flex', alignItems: 'center', gap: '8px', fontSize: '13px', fontWeight: 700, color: config.unlockRotation ? '#e9d5ff' : '#cbd5e1', cursor: 'pointer', pointerEvents: 'none' }}>
           <input
             type="checkbox"
             checked={Boolean(config.unlockRotation)}
             readOnly
             style={{ width: '16px', height: '16px', accentColor: '#a855f7' }}
           />
           {config.unlockRotation
             ? '🔓 Giro 360° DESBLOQUEADO (Toca para bloquear)'
             : '🔒 Límite de Rotación Bloqueado (Toca para giro 360°)'}
         </label>
         <span style={{ fontSize: '11px', padding: '3px 8px', borderRadius: '6px', background: config.unlockRotation ? '#9333ea' : '#334155', color: '#fff', fontWeight: 600 }}>
           {config.unlockRotation ? '360° LIBRE' : 'FRENTE FIJO'}
         </span>
       </div>

          

          {/* Control de Suavizado Anti-Ruido (En vivo + Post-grabación) */}
          <div style={{ marginTop: '10px', padding: '10px', background: '#141724', borderRadius: '10px', border: '1px solid #2e344d' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', fontSize: '12px', marginBottom: '6px' }}>
              <span style={{ color: '#38bdf8', fontWeight: 700 }}>
                🌊 Suavizar Ruido (Filtro): {Math.round((config.smoothing ?? 0.2) * 100)}%
              </span>
              <span style={{ color: '#94a3b8', fontSize: '11px' }}>
                {(config.smoothing ?? 0.2) < 0.25
                  ? '⚡ Rápido'
                  : (config.smoothing ?? 0.2) < 0.55
                  ? '⚖️ Equilibrado'
                  : '🧈 Súper Suave'}
              </span>
            </div>

            <input
              type="range"
              min="0.05"
              max="0.85"
              step="0.05"
              value={config.smoothing ?? 0.20}
              onChange={(e) => {
                const val = parseFloat(e.target.value);
                setConfig((prev) => ({ ...prev, smoothing: val }));
              }}
              style={{ width: '100%', accentColor: '#38bdf8', cursor: 'pointer' }}
            />

            <div className="mp-btn-row" style={{ marginTop: '6px' }}>
              <button
                className="mp-btn"
                style={{ background: '#0ea5e9', padding: '7px', fontSize: '12px' }}
                disabled={recordedFrames.length < 3}
                onClick={() => applyPostSmoothingToFrames(2)}
              >
                ✨ Pulir Ruido de Frames Grabados ({recordedFrames.length}f)
              </button>
            </div>
          </div>

          <div className="mp-angle-bar">
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', fontSize: '12px' }}>
              <span style={{ color: '#38bdf8', fontWeight: 700 }}>
                🦴 {selectedBone ? `Hueso activo: ${selectedBone}` : 'Toca una esfera verde para ver sus ángulos'}
              </span>
              {copyFeedback && (
                <span style={{ color: '#10b981', fontSize: '11px', fontWeight: 600 }}>{copyFeedback}</span>
              )}
            </div>

            {selectedBone && currentSelOffset && (
              <div className="mp-xyz-pills">
                <div className="mp-pill" style={{ color: '#f87171', border: '1px solid #7f1d1d' }}>
                  Eje X: {currentSelOffset.rx || 0}°
                </div>
                <div className="mp-pill" style={{ color: '#4ade80', border: '1px solid #14532d' }}>
                  Eje Y: {currentSelOffset.ry || 0}°
                </div>
                <div className="mp-pill" style={{ color: '#60a5fa', border: '1px solid #1e3a8a' }}>
                  Eje Z: {currentSelOffset.rz || 0}°
                </div>
              </div>
            )}

            <label style={{ display: 'flex', alignItems: 'center', gap: '8px', fontSize: '11px', color: '#94a3b8', cursor: 'pointer' }}>
              <input
                type="checkbox"
                checked={keepOffsetsOnRecord}
                onChange={(e) => setKeepOffsetsOnRecord(e.target.checked)}
              />
              📌 Mantener ajustes del Gizmo al grabar de nuevo
            </label>

            <div className="mp-btn-row" style={{ marginTop: '2px' }}>
              <button
                className="mp-btn mp-btn-purple"
                style={{ padding: '8px', fontSize: '12px' }}
                disabled={!selectedBone}
                onClick={() => copyBoneAngleData('selected')}
              >
                📋 Copiar Ángulo ({selectedBone || 'Hueso'})
              </button>
              <button
                className="mp-btn mp-btn-blue"
                style={{ padding: '8px', fontSize: '12px' }}
                onClick={() => copyBoneAngleData('all')}
              >
                📋 Copiar Todo Modificado
              </button>
            </div>

            <div className="mp-btn-row" style={{ marginTop: '0px' }}>
              <button
                className="mp-btn"
                style={{ background: '#334155', padding: '6px', fontSize: '11px' }}
                disabled={!selectedBone}
                onClick={resetSelectedBone}
              >
                🔄 Resetear Hueso Actual
              </button>
              <button
                className="mp-btn"
                style={{ background: '#475569', padding: '6px', fontSize: '11px' }}
                onClick={resetAllBones}
              >
                🧹 Limpiar Todos los Cambios
              </button>
            </div>
          </div>
        </div>

        <div style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
          <div className="mp-card">
            <strong style={{ fontSize: '14px', color: '#10b981' }}>💾 Exportar Animación + Diagnóstico</strong>

            <label style={{ display: 'flex', alignItems: 'center', gap: '8px', fontSize: '12px', color: '#cbd5e1', marginTop: '8px', cursor: 'pointer' }}>
              <input
                type="checkbox"
                checked={exportFlipVrm0}
                onChange={(e) => setExportFlipVrm0(e.target.checked)}
              />
              🔄 Invertir X/Z al exportar (Corrige Z:+46° a Z:-46° para visores externos)
            </label>

            <div className="mp-btn-row">
              <button
                className="mp-btn mp-btn-green"
                disabled={recordedFrames.length === 0}
                onClick={() => {
                  const syncedOffsets = getSyncedOffsetsFromDeelia();
                  boneOffsetsRef.current = syncedOffsets;
                  setBoneOffsets(syncedOffsets);

                  const blob = buildVRMABinary(
                    recordedFrames,
                    syncedOffsets,
                    { exportFlipVrm0 },
                    (logs) => setExportLogs(logs)
                  );
                  downloadFile(blob, `matrixpose_${recordedFrames.length}f.vrma`);
                }}
              >
                ⬇️ .VRMA ({recordedFrames.length}f)
              </button>
              <button
                className="mp-btn mp-btn-purple"
                disabled={recordedFrames.length === 0}
                onClick={() => {
                  const syncedOffsets = getSyncedOffsetsFromDeelia();
                  boneOffsetsRef.current = syncedOffsets;
                  setBoneOffsets(syncedOffsets);

                  const blob = buildVMDBinary(
                    recordedFrames,
                    syncedOffsets,
                    { exportFlipVrm0 },
                    (logs) => setExportLogs(logs)
                  );
                  downloadFile(blob, `matrixpose_${recordedFrames.length}f.vmd`);
                }}
              >
                ⬇️ .VMD (MMD)
              </button>
            </div>

            <div className="mp-btn-row">
              <button
                className="mp-btn mp-btn-amber"
                style={{ fontSize: '12px', padding: '8px' }}
                disabled={recordedFrames.length === 0 || isPlayingPreview}
                onClick={previewExportOnDeelia}
              >
                {isPlayingPreview ? '▶️ Reproduciendo en Deelia...' : '👁️ Previsualizar Grabación en Deelia'}
              </button>
            </div>

            <div className="mp-log-box">
              {exportLogs.length === 0 ? (
                <div style={{ color: '#64748b' }}>
                  📋 Aquí aparecerán los mensajes paso a paso cuando grabes o presiones exportar (.VRMA / .VMD)...
                </div>
              ) : (
                exportLogs.map((line, i) => (
                  <div key={i} style={{ marginBottom: '3px' }}>{line}</div>
                ))
              )}
            </div>
          </div>

          <PosingPanel
            vrmList={vrmList}
            activeVrmIndex={activeVrmIndex}
            selectedBone={selectedBone}
            setSelectedBone={setSelectedBone}
            showBoneNodes={showBoneNodes}
            setShowBoneNodes={setShowBoneNodes}
            onBoneOffsetChange={handleBoneOffsetUpdate}
            config={config}
            setConfig={setConfig}
          />
        </div>
      </div>
    </div>
  );
}
