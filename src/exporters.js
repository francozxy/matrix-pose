// src/exporters.js
import * as THREE from 'three';
import { BONE_LIST, BONE_OFFSETS } from './kinematics';

const PARENT_INDEX_MAP = {
  hips: -1,
  spine: 0,
  chest: 1,
  neck: 2,
  head: 3,
  leftUpperArm: 2,
  leftLowerArm: 5,
  leftHand: 6,
  rightUpperArm: 2,
  rightLowerArm: 8,
  rightHand: 9,
  leftUpperLeg: 0,
  leftLowerLeg: 11,
  leftFoot: 12,
  rightUpperLeg: 0,
  rightLowerLeg: 14,
  rightFoot: 15
};

function quatToDegStr(qArr) {
  if (!qArr || qArr.length < 4) return 'X:0° Y:0° Z:0°';
  const q = new THREE.Quaternion(qArr[0], qArr[1], qArr[2], qArr[3]);
  const e = new THREE.Euler().setFromQuaternion(q, 'YXZ');
  return `X:${Math.round(THREE.MathUtils.radToDeg(e.x))}° Y:${Math.round(THREE.MathUtils.radToDeg(e.y))}° Z:${Math.round(THREE.MathUtils.radToDeg(e.z))}°`;
}

/**
 * Hornea los offsets manuales (y los giros hechos en el visor con el video pausado)
 * y alinea los ejes con el estándar oficial VRMC_vrm_animation / VMD.
 */
export function bakeOffsetsIntoFrames(frames, boneOffsets = {}, options = {}, logFn = () => {}) {
  logFn(`🔹 [PASO 1] Procesando ${frames?.length || 0} frames para exportación...`);

  if (!frames || frames.length === 0) {
    logFn(`❌ [ERROR] No hay frames grabados.`);
    return [];
  }

  // Detectar todos los huesos que tengan un ajuste en boneOffsets
  const modifiedBones = Object.entries(boneOffsets).filter(
    ([, o]) =>
      o &&
      (Math.abs(o.rx || 0) > 0.1 ||
        Math.abs(o.ry || 0) > 0.1 ||
        Math.abs(o.rz || 0) > 0.1 ||
        Math.abs(o.px || 0) > 0.001 ||
        Math.abs(o.py || 0) > 0.001 ||
        Math.abs(o.pz || 0) > 0.001)
  );

  if (modifiedBones.length > 0) {
    logFn(`🛠️ [PASO 1.1] APLICANDO TUS MODIFICACIONES MANUALES (${modifiedBones.length} huesos):`);
    modifiedBones.forEach(([bName, o]) => {
      logFn(`   ↳ ${bName}: X=${o.rx || 0}°, Y=${o.ry || 0}°, Z=${o.rz || 0}°`);
    });
  } else {
    logFn(`ℹ️ [PASO 1.1] Exportando animación directa sin offsets manuales extras.`);
  }

  const midIdx = Math.floor(frames.length / 2);
  const sampleBefore = frames[midIdx]?.rawLocalQuats?.rightUpperArm || frames[midIdx]?.localQuats?.rightUpperArm;
  logFn(`🔍 [MUESTRA #${midIdx}] rightUpperArm INTERNO: ${quatToDegStr(sampleBefore)}`);

  const flipVrm0 = Boolean(options.exportFlipVrm0);
  if (flipVrm0) {
    logFn(`🔄 [CORRECCIÓN EJES ACTIVA] Convirtiendo ejes internos (Z:+46° -> Z:-46°) al estándar VRMA/VMD.`);
  }

  const bakedFrames = frames.map((f) => {
    const baseHips = f.rawHipsPos || f.hipsPos || [0, 0, 0];
    const hipOff = boneOffsets.hips || {};
    const hx = (baseHips[0] || 0) + (hipOff.px || 0);
    const hy = (baseHips[1] || 0) + (hipOff.py || 0);
    const hz = (baseHips[2] || 0) + (hipOff.pz || 0);

    const bakedHips = flipVrm0 ? [-hx, hy, -hz] : [hx, hy, hz];
    const sourceQuats = f.rawLocalQuats || f.localQuats || {};
    const bakedQuats = {};

    BONE_LIST.forEach((bName) => {
      const qArr = sourceQuats[bName] || [0, 0, 0, 1];
      const baseQ = new THREE.Quaternion(qArr[0], qArr[1], qArr[2], qArr[3]).normalize();

      // Sumar el offset manual de este hueso
      const custom = boneOffsets[bName];
      if (custom && (Math.abs(custom.rx || 0) > 0.1 || Math.abs(custom.ry || 0) > 0.1 || Math.abs(custom.rz || 0) > 0.1)) {
        const addEuler = new THREE.Euler(
          THREE.MathUtils.degToRad(custom.rx || 0),
          THREE.MathUtils.degToRad(custom.ry || 0),
          THREE.MathUtils.degToRad(custom.rz || 0),
          'YXZ'
        );
        const addQuat = new THREE.Quaternion().setFromEuler(addEuler);
        baseQ.multiply(addQuat).normalize();
      }

      // Si el modelo en visor tenía Z invertido (+46° para bajar brazo derecho),
      // al pasar a estándar VRMA/VMD invertimos X y Z para que el brazo derecho baje con -46°
      if (flipVrm0) {
        bakedQuats[bName] = [-baseQ.x, baseQ.y, -baseQ.z, baseQ.w];
      } else {
        bakedQuats[bName] = [baseQ.x, baseQ.y, baseQ.z, baseQ.w];
      }
    });

    return {
      ...f,
      hipsPos: bakedHips,
      localQuats: bakedQuats
    };
  });

  const sampleAfterR = bakedFrames[midIdx]?.localQuats?.rightUpperArm;
  const sampleAfterL = bakedFrames[midIdx]?.localQuats?.leftUpperArm;
  logFn(`🎯 [MUESTRA #${midIdx}] rightUpperArm FINAL: ${quatToDegStr(sampleAfterR)}`);
  logFn(`🎯 [MUESTRA #${midIdx}] leftUpperArm FINAL: ${quatToDegStr(sampleAfterL)}`);

  return bakedFrames;
}

// ============================================================================
// 1. EXPORTADOR .VRMA
// ============================================================================
export function buildVRMABinary(rawFrames, boneOffsets = {}, options = {}, onLog = null) {
  const logs = [];
  const log = (msg) => {
    console.log(msg);
    logs.push(msg);
    if (onLog) onLog([...logs]);
  };

  log(`🚀 === INICIANDO EXPORTACIÓN .VRMA ===`);

  const frames = bakeOffsetsIntoFrames(rawFrames, boneOffsets, options, log);
  if (!frames || frames.length === 0) return null;

  const numFrames = frames.length;
  const t0 = frames[0].time || 0;

  const times = new Float32Array(numFrames);
  let lastT = -0.001;
  for (let i = 0; i < numFrames; i++) {
    let t = ((frames[i].time || 0) - t0) / 1000;
    if (t <= lastT) t = lastT + 0.016;
    times[i] = t;
    lastT = t;
  }
  log(`⏱️ [PASO 2] Duración: ${lastT.toFixed(2)}s (${numFrames} frames).`);

  const hipsTranslations = new Float32Array(numFrames * 3);
  for (let i = 0; i < numFrames; i++) {
    const hp = frames[i].hipsPos || [0, 0, 0];
    hipsTranslations[i * 3 + 0] = hp[0] || 0;
    hipsTranslations[i * 3 + 1] = 0.92 + (hp[1] || 0);
    hipsTranslations[i * 3 + 2] = hp[2] || 0;
  }

  const boneRotations = {};
  BONE_LIST.forEach((bName) => {
    const arr = new Float32Array(numFrames * 4);
    for (let i = 0; i < numFrames; i++) {
      const q = frames[i].localQuats?.[bName] || [0, 0, 0, 1];
      arr[i * 4 + 0] = q[0];
      arr[i * 4 + 1] = q[1];
      arr[i * 4 + 2] = q[2];
      arr[i * 4 + 3] = q[3];
    }
    boneRotations[bName] = arr;
  });

  const childrenMap = {};
  BONE_LIST.forEach((_, idx) => { childrenMap[idx] = []; });
  BONE_LIST.forEach((bName, idx) => {
    const pIdx = PARENT_INDEX_MAP[bName];
    if (pIdx !== undefined && pIdx >= 0) {
      childrenMap[pIdx].push(idx);
    }
  });

  const nodes = BONE_LIST.map((bName, idx) => {
    const nodeObj = {
      name: bName,
      translation: BONE_OFFSETS[bName] || [0, 0, 0],
      rotation: [0, 0, 0, 1]
    };
    if (childrenMap[idx].length > 0) {
      nodeObj.children = childrenMap[idx];
    }
    return nodeObj;
  });

  const chunks = [];
  const bufferViews = [];
  const accessors = [];
  let byteOffset = 0;

  const pushBuffer = (typedArray, type, componentType, includeMinMax = false) => {
    const byteLength = typedArray.byteLength;
    const pad = (4 - (byteLength % 4)) % 4;

    chunks.push(new Uint8Array(typedArray.buffer, typedArray.byteOffset, byteLength));
    if (pad > 0) chunks.push(new Uint8Array(pad));

    const bvIndex = bufferViews.length;
    bufferViews.push({
      buffer: 0,
      byteOffset,
      byteLength
    });
    byteOffset += byteLength + pad;

    const count =
      type === 'SCALAR' ? typedArray.length :
      type === 'VEC3' ? typedArray.length / 3 :
      typedArray.length / 4;

    const acc = {
      bufferView: bvIndex,
      componentType,
      count,
      type
    };

    if (includeMinMax && count > 0) {
      acc.min = [typedArray[0]];
      acc.max = [typedArray[typedArray.length - 1]];
    }

    const accIndex = accessors.length;
    accessors.push(acc);
    return accIndex;
  };

  const timeAccIdx = pushBuffer(times, 'SCALAR', 5126, true);
  const hipsTransAccIdx = pushBuffer(hipsTranslations, 'VEC3', 5126, false);

  const samplers = [];
  const channels = [];

  samplers.push({
    input: timeAccIdx,
    output: hipsTransAccIdx,
    interpolation: 'LINEAR'
  });
  channels.push({
    sampler: 0,
    target: { node: 0, path: 'translation' }
  });

  BONE_LIST.forEach((bName, nodeIdx) => {
    const rotAccIdx = pushBuffer(boneRotations[bName], 'VEC4', 5126, false);
    const samplerIdx = samplers.length;
    samplers.push({
      input: timeAccIdx,
      output: rotAccIdx,
      interpolation: 'LINEAR'
    });
    channels.push({
      sampler: samplerIdx,
      target: { node: nodeIdx, path: 'rotation' }
    });
  });

  const humanBones = {};
  BONE_LIST.forEach((bName, nodeIdx) => {
    humanBones[bName] = { node: nodeIdx };
  });

  const gltfJson = {
    asset: { version: '2.0', generator: 'MatrixPose Mobile V5.5' },
    extensionsUsed: ['VRMC_vrm_animation'],
    extensions: {
      VRMC_vrm_animation: {
        specVersion: '1.0',
        humanoid: { humanBones }
      }
    },
    scene: 0,
    scenes: [{ nodes: [0] }],
    nodes,
    animations: [{ name: 'MatrixPoseAnimation', samplers, channels }],
    bufferViews,
    accessors,
    buffers: [{ byteLength: byteOffset }]
  };

  const jsonEncoder = new TextEncoder();
  const jsonBytes = jsonEncoder.encode(JSON.stringify(gltfJson));
  const jsonPad = (4 - (jsonBytes.byteLength % 4)) % 4;
  const jsonChunkLength = jsonBytes.byteLength + jsonPad;

  const totalByteLength = 12 + 8 + jsonChunkLength + 8 + byteOffset;
  const glbBuffer = new ArrayBuffer(totalByteLength);
  const dataView = new DataView(glbBuffer);
  const uint8Out = new Uint8Array(glbBuffer);

  dataView.setUint32(0, 0x46546c67, true);
  dataView.setUint32(4, 2, true);
  dataView.setUint32(8, totalByteLength, true);

  dataView.setUint32(12, jsonChunkLength, true);
  dataView.setUint32(16, 0x4e4f534a, true);
  uint8Out.set(jsonBytes, 20);
  for (let i = 0; i < jsonPad; i++) {
    uint8Out[20 + jsonBytes.byteLength + i] = 0x20;
  }

  const binHeaderStart = 20 + jsonChunkLength;
  dataView.setUint32(binHeaderStart, byteOffset, true);
  dataView.setUint32(binHeaderStart + 4, 0x004e4942, true);

  let cursor = binHeaderStart + 8;
  for (const chunk of chunks) {
    uint8Out.set(chunk, cursor);
    cursor += chunk.byteLength;
  }

  log(`✅ [PASO 3] Archivo .VRMA listo (${(totalByteLength / 1024).toFixed(1)} KB).`);
  return new Blob([glbBuffer], { type: 'model/gltf-binary' });
}

// ============================================================================
// 2. EXPORTADOR .VMD (MMD)
// ============================================================================
const MMD_BONE_NAMES = {
  hips: 'センター',
  spine: '上半身',
  chest: '上半身2',
  neck: '首',
  head: '頭',
  leftUpperArm: '左腕',
  leftLowerArm: '左ひじ',
  leftHand: '左手首',
  rightUpperArm: '右腕',
  rightLowerArm: '右ひじ',
  rightHand: '右手首',
  leftUpperLeg: '左足',
  leftLowerLeg: '左ひざ',
  leftFoot: '左足首',
  rightUpperLeg: '右足',
  rightLowerLeg: '右ひざ',
  rightFoot: '右足首'
};

const SHIFT_JIS_MAP = {
  'センター': [0x83, 0x5A, 0x83, 0x93, 0x83, 0x5E, 0x81, 0x5B],
  '上半身': [0x8F, 0xE3, 0x94, 0xBC, 0x90, 0x67],
  '上半身2': [0x8F, 0xE3, 0x94, 0xBC, 0x90, 0x67, 0x32],
  '首': [0x8E, 0xF1],
  '頭': [0x93, 0xAA],
  '左腕': [0x8D, 0xB6, 0x98, 0x72],
  '左ひじ': [0x8D, 0xB6, 0x82, 0xD0, 0x82, 0xB6],
  '左手首': [0x8D, 0xB6, 0x8E, 0xE8, 0x8E, 0xF1],
  '右腕': [0x89, 0x45, 0x98, 0x72],
  '右ひじ': [0x89, 0x45, 0x82, 0xD0, 0x82, 0xB6],
  '右手首': [0x89, 0x45, 0x8E, 0xE8, 0x8E, 0xF1],
  '左足': [0x8D, 0xB6, 0x91, 0xAB],
  '左ひざ': [0x8D, 0xB6, 0x82, 0xD0, 0x82, 0xB4],
  '左足首': [0x8D, 0xB6, 0x91, 0xAB, 0x8E, 0xF1],
  '右足': [0x89, 0x45, 0x91, 0xAB],
  '右ひざ': [0x89, 0x45, 0x82, 0xD0, 0x82, 0xB4],
  '右足首': [0x89, 0x45, 0x91, 0xAB, 0x8E, 0xF1]
};

export function buildVMDBinary(rawFrames, boneOffsets = {}, options = {}, onLog = null) {
  const logs = [];
  const log = (msg) => {
    console.log(msg);
    logs.push(msg);
    if (onLog) onLog([...logs]);
  };

  log(`🚀 === INICIANDO EXPORTACIÓN .VMD (MMD) ===`);

  const frames = bakeOffsetsIntoFrames(rawFrames, boneOffsets, options, log);
  if (!frames || frames.length === 0) return null;

  const numFrames = frames.length;
  const totalRecords = numFrames * BONE_LIST.length;
  const recordSize = 111;
  const totalSize = 30 + 20 + 4 + totalRecords * recordSize + 16;

  const buffer = new ArrayBuffer(totalSize);
  const view = new DataView(buffer);
  const out = new Uint8Array(buffer);

  const writeAscii = (str, offset, maxLen) => {
    for (let i = 0; i < Math.min(str.length, maxLen); i++) {
      out[offset + i] = str.charCodeAt(i);
    }
  };

  writeAscii('Vocaloid Motion Data 0002', 0, 30);
  writeAscii('MatrixPoseModel', 30, 20);
  view.setUint32(50, totalRecords, true);

  let offset = 54;
  const t0 = frames[0].time || 0;

  const leftAPoseComp = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), THREE.MathUtils.degToRad(-35));
  const rightAPoseComp = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), THREE.MathUtils.degToRad(35));

  for (let i = 0; i < numFrames; i++) {
    const f = frames[i];
    const frameNumber = Math.max(i, Math.round(((f.time || 0) - t0) / 33.333));

    for (const bName of BONE_LIST) {
      const jpName = MMD_BONE_NAMES[bName];
      const sjisBytes = SHIFT_JIS_MAP[jpName] || [];

      for (let b = 0; b < sjisBytes.length && b < 15; b++) {
        out[offset + b] = sjisBytes[b];
      }

      view.setUint32(offset + 15, frameNumber, true);

      let px = 0, py = 0, pz = 0;
      if (bName === 'hips' && f.hipsPos) {
        px = -f.hipsPos[0] * 12.5;
        py = f.hipsPos[1] * 12.5;
        pz = -f.hipsPos[2] * 12.5;
      }
      view.setFloat32(offset + 19, px, true);
      view.setFloat32(offset + 23, py, true);
      view.setFloat32(offset + 27, pz, true);

      const qArr = f.localQuats?.[bName] || [0, 0, 0, 1];
      const q = new THREE.Quaternion(qArr[0], qArr[1], qArr[2], qArr[3]);

      if (bName === 'leftUpperArm') q.premultiply(leftAPoseComp);
      if (bName === 'rightUpperArm') q.premultiply(rightAPoseComp);

      view.setFloat32(offset + 31, -q.x, true);
      view.setFloat32(offset + 35, q.y, true);
      view.setFloat32(offset + 39, -q.z, true);
      view.setFloat32(offset + 43, q.w, true);

      for (let k = 0; k < 64; k++) {
        out[offset + 47 + k] = (k % 4 === 0) ? 20 : 107;
      }

      offset += recordSize;
    }
  }

  log(`✅ [PASO 3] Archivo .VMD listo (${(totalSize / 1024).toFixed(1)} KB).`);
  return new Blob([buffer], { type: 'application/octet-stream' });
}
