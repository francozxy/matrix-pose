// src/kinematics.js
import * as THREE from 'three';

export const FINGER_BONE_LIST = [
  'leftThumbProximal', 'leftThumbIntermediate', 'leftThumbDistal',
  'leftIndexProximal', 'leftIndexIntermediate', 'leftIndexDistal',
  'leftMiddleProximal', 'leftMiddleIntermediate', 'leftMiddleDistal',
  'leftRingProximal', 'leftRingIntermediate', 'leftRingDistal',
  'leftLittleProximal', 'leftLittleIntermediate', 'leftLittleDistal',
  'rightThumbProximal', 'rightThumbIntermediate', 'rightThumbDistal',
  'rightIndexProximal', 'rightIndexIntermediate', 'rightIndexDistal',
  'rightMiddleProximal', 'rightMiddleIntermediate', 'rightMiddleDistal',
  'rightRingProximal', 'rightRingIntermediate', 'rightRingDistal',
  'rightLittleProximal', 'rightLittleIntermediate', 'rightLittleDistal'
];

export const BONE_LIST = [
  'hips', 'spine', 'chest', 'neck', 'head',
  'leftUpperArm', 'leftLowerArm', 'leftHand',
  'rightUpperArm', 'rightLowerArm', 'rightHand',
  'leftUpperLeg', 'leftLowerLeg', 'leftFoot',
  'rightUpperLeg', 'rightLowerLeg', 'rightFoot',
  ...FINGER_BONE_LIST
];

export const BONE_OFFSETS = {
  hips: [0, 0.92, 0],
  spine: [0, 0.12, 0],
  chest: [0, 0.14, 0],
  neck: [0, 0.14, 0],
  head: [0, 0.10, 0],
  leftUpperArm: [-0.15, 0.08, 0],
  leftLowerArm: [-0.23, 0, 0],
  leftHand: [-0.22, 0, 0],
  rightUpperArm: [0.15, 0.08, 0],
  rightLowerArm: [0.23, 0, 0],
  rightHand: [0.22, 0, 0],
  leftUpperLeg: [-0.09, -0.04, 0],
  leftLowerLeg: [0, -0.38, 0],
  leftFoot: [0, -0.38, 0],
  rightUpperLeg: [0.09, -0.04, 0],
  rightLowerLeg: [0, -0.38, 0],
  rightFoot: [0, -0.38, 0]
};

const T_POSE_DIRS = {
  leftLowerArm: new THREE.Vector3(-1, 0, 0),
  rightLowerArm: new THREE.Vector3(1, 0, 0),
  leftUpperLeg: new THREE.Vector3(0, -1, 0),
  leftLowerLeg: new THREE.Vector3(0, -1, 0),
  rightUpperLeg: new THREE.Vector3(0, -1, 0),
  rightLowerLeg: new THREE.Vector3(0, -1, 0)
};

const ARM_SET = new Set([
  'leftUpperArm', 'leftLowerArm', 'leftHand',
  'rightUpperArm', 'rightLowerArm', 'rightHand'
]);

function sanitizeQuat(q) {
  if (
    !q ||
    !Number.isFinite(q.x) ||
    !Number.isFinite(q.y) ||
    !Number.isFinite(q.z) ||
    !Number.isFinite(q.w) ||
    q.lengthSq() < 0.0001
  ) {
    return new THREE.Quaternion(0, 0, 0, 1);
  }
  return q.normalize();
}

function wrapPi(a) {
  while (a > Math.PI) a -= Math.PI * 2;
  while (a < -Math.PI) a += Math.PI * 2;
  return a;
}

function hybridVec3(screenLm, worldLm, config, aspect = 0.6) {
  if (!screenLm || typeof screenLm.x !== 'number' || typeof screenLm.y !== 'number') {
    return new THREE.Vector3(0, 0, 0);
  }
  const mirrorSign = config.mirror ? -1 : 1;
  const x = (screenLm.x - 0.5) * aspect * mirrorSign;
  const y = -(screenLm.y - 0.5);

  const rawZ = worldLm && Number.isFinite(worldLm.z) ? worldLm.z : (screenLm.z || 0);
  const z = -rawZ * (config.depthScale ?? 0.35);

  return new THREE.Vector3(x, y, z);
}

function getRawDepthDiff(idxRight, idxLeft, worldLandmarks, screenLandmarks, invertZ = false) {
  const wR = worldLandmarks?.[idxRight];
  const wL = worldLandmarks?.[idxLeft];
  let diff = 0;
  if (wR && wL && Number.isFinite(wR.z) && Number.isFinite(wL.z)) {
    diff = (-wR.z) - (-wL.z);
  } else {
    const sR = screenLandmarks?.[idxRight];
    const sL = screenLandmarks?.[idxLeft];
    if (sR && sL && Number.isFinite(sR.z) && Number.isFinite(sL.z)) {
      diff = (-sR.z) - (-sL.z);
    }
  }
  return invertZ ? -diff : diff;
}

function applyDeadzone(currentVal, prevVal, threshold = 0.0035) {
  if (prevVal === undefined) return currentVal;
  return Math.abs(currentVal - prevVal) < threshold ? prevVal : currentVal;
}

function applyQuatDeadzone(currQ, prevQArray, threshold = 0.008) {
  if (!prevQArray) return currQ;
  const prevQ = sanitizeQuat(new THREE.Quaternion(...prevQArray));
  const dot = Math.abs(currQ.dot(prevQ));
 const angle = 2 * Math.acos(dot); // Ángulo real de cambio entre frames
  // Si se movió menos de ~2 grados (0.035 rad), se queda 100% quieto
  if (angle < minAngleRad) {
    return prevQ;
  }
  return currQ;
}

function unwrapAngle(targetAngle, prevAngle) {
  if (!Number.isFinite(prevAngle)) return targetAngle;
  let diff = targetAngle - prevAngle;
  while (diff > Math.PI) diff -= Math.PI * 2;
  while (diff < -Math.PI) diff += Math.PI * 2;
  return prevAngle + diff;
}

function computeReliableYaw(
  shDx,
  rawShDz,
  pNose,
  pLEye,
  pREye,
  shCenter,
  hipCenter,
  pLKn,
  pRKn,
  unlockRotation = false,
  prevYaw = 0
) {
  const eyeCenter = new THREE.Vector3().addVectors(pLEye, pREye).multiplyScalar(0.5);
  const faceLookX = (pNose.x - eyeCenter.x) * 2.2 + (pNose.x - shCenter.x) * 0.8;
  const kneeCenter = new THREE.Vector3().addVectors(pLKn, pRKn).multiplyScalar(0.5);
  const lowerBodyLeadX = (kneeCenter.x - hipCenter.x) * 0.9;

  const shWidth = Math.abs(shDx);
  const profileRatio = THREE.MathUtils.clamp((0.17 - shWidth) / 0.11, 0, 1);

  const dir2D = faceLookX + lowerBodyLeadX * 0.5;
  const sign2D = Math.abs(dir2D) > 0.008 ? Math.sign(dir2D) : 0;

  if (unlockRotation) {
    const safeShDx = Math.abs(shDx) < 0.025 ? (shDx >= 0 ? 0.025 : -0.025) : shDx;
    const zYawUnclamped = -Math.atan2(rawShDz * 1.6, safeShDx);
    const prevSign = Math.abs(prevYaw) > 0.4 ? -Math.sign(prevYaw) : 0;
    const turnSign = prevSign !== 0 && shWidth < 0.05 ? prevSign : (sign2D !== 0 ? sign2D : (Math.sign(zYawUnclamped) || 1));

    let fullAngle = Math.abs(zYawUnclamped);
    if (shDx < -0.01) {
      fullAngle = Math.max(fullAngle, Math.PI * 0.55 + profileRatio * 1.25);
    } else {
      fullAngle = Math.max(fullAngle, profileRatio * 1.55);
    }
    const target = THREE.MathUtils.clamp(turnSign * fullAngle, -Math.PI, Math.PI);
    return target;
  }

  const zYaw = -Math.atan2(rawShDz * 1.3, Math.max(0.06, shWidth * 1.3));
  const profileYawMag = profileRatio * 1.05;
  const yaw2D = sign2D !== 0 ? sign2D * profileYawMag : dir2D * 6.0;

  if (sign2D !== 0 && Math.abs(dir2D) > 0.018 && Math.sign(zYaw) !== sign2D) {
    return THREE.MathUtils.clamp(yaw2D * 0.85, -1.25, 1.25);
  }

  return THREE.MathUtils.clamp(zYaw * 0.45 + yaw2D * 0.65, -1.3, 1.3);
}

// 1. MÓDULO DE CADERA
function solveHips(pLHip, pRHip, reliableTorsoYaw, screenLandmarks, mirrorSign, prevFrame, config) {
  const midX = (screenLandmarks[23]?.x + screenLandmarks[24]?.x) * 0.5 || 0.5;
  const midY = (screenLandmarks[23]?.y + screenLandmarks[24]?.y) * 0.5 || 0.5;

  const lAnkX = screenLandmarks[27]?.x ?? midX;
  const rAnkX = screenLandmarks[28]?.x ?? midX;
  const feetMidX = (lAnkX + rAnkX) * 0.5;

  const lAnkY = screenLandmarks[27]?.y ?? 0.85;
  const rAnkY = screenLandmarks[28]?.y ?? 0.85;
  const lowestFootScreenY = Math.max(lAnkY, rAnkY);

  const bodyCenterX = midX * 0.65 + feetMidX * 0.35;
  const prevOriginX = prevFrame?.originX ?? bodyCenterX;
  const originX = prevOriginX * 0.998 + bodyCenterX * 0.002;

  const prevBaseHipY = prevFrame?.baseHipY ?? midY;
  const prevBaseFootY = prevFrame?.baseFootY ?? lowestFootScreenY;

  const baseFootY =
    lowestFootScreenY >= prevBaseFootY
      ? prevBaseFootY * 0.85 + lowestFootScreenY * 0.15
      : prevBaseFootY * 0.992 + lowestFootScreenY * 0.008;

  const isFeetOnGround = (baseFootY - lowestFootScreenY) < 0.035;
  const baseHipY = isFeetOnGround
    ? prevBaseHipY * 0.92 + midY * 0.08
    : prevBaseHipY;

  const moveScale = config?.hipMoveScale ?? 0.7;

  const hipShiftX = (midX - originX) * mirrorSign;
  const feetShiftX = (feetMidX - originX) * mirrorSign;

  const isSteppingSideways =
    Math.sign(hipShiftX) === Math.sign(feetShiftX) && Math.abs(feetShiftX) > 0.02;

  let rawDx = (hipShiftX * 0.65 + feetShiftX * 0.35) * (1.75 * moveScale);
  if (isSteppingSideways) {
    rawDx += feetShiftX * (0.95 * moveScale);
  }

  const hipRise = baseHipY - midY;
  const feetLift = Math.max(0, baseFootY - lowestFootScreenY);

  let rawDy = hipRise * (1.15 * moveScale);
  if (hipRise > 0.015 && feetLift > 0.015) {
    rawDy += feetLift * 1.35;
  }

  const moveX = config?.moveEnabled !== false;
  const moveZ = Boolean(config?.moveDepthEnabled);

  // Izquierda/derecha: solo si "Permitir desplazamiento" está activado
  const dx = moveX ? applyDeadzone(rawDx, prevFrame?.hipsPos?.[0], 0.0025) : 0;
  const dy = applyDeadzone(rawDy, prevFrame?.hipsPos?.[1], 0.003);

  // Adelante/atrás: se estima por cuánto crece o se encoge el torso en pantalla
  // respecto al primer frame (más grande = más cerca de la cámara).
  const sh1 = screenLandmarks[11], sh2 = screenLandmarks[12];
  const hp1 = screenLandmarks[23], hp2 = screenLandmarks[24];
  let rawTorso = prevFrame?.torsoLenSm ?? 0;
  if (sh1 && sh2 && hp1 && hp2) {
    rawTorso = Math.hypot(
      ((sh1.x + sh2.x) - (hp1.x + hp2.x)) * 0.5 * 0.6,
      ((sh1.y + sh2.y) - (hp1.y + hp2.y)) * 0.5
    );
  }
  const torsoLenSm = prevFrame?.torsoLenSm ? prevFrame.torsoLenSm * 0.85 + rawTorso * 0.15 : rawTorso;
  const baseTorsoLen = prevFrame?.baseTorsoLen ?? torsoLenSm;
  let dz = 0;
  if (moveZ && baseTorsoLen > 0.02) {
    const r = torsoLenSm / baseTorsoLen - 1;
    const dead = 0.06;
    const rr = Math.abs(r) < dead ? 0 : r - Math.sign(r) * dead;
    dz = THREE.MathUtils.clamp(rr * 2.2 * moveScale, -1.2, 1.5);
  }

  const hipDx = pRHip.x - pLHip.x;
  const hipDy = pRHip.y - pLHip.y;

  const isUnlocked = Boolean(config?.unlockRotation);
  const rawHipYaw = isUnlocked ? reliableTorsoYaw * 0.92 : reliableTorsoYaw * 0.65;
  const maxHipYaw = isUnlocked ? Math.PI : 0.95;
  const clampedHipYaw = THREE.MathUtils.clamp(rawHipYaw, -maxHipYaw, maxHipYaw);

  const frontalStability = Math.max(0, Math.cos(clampedHipYaw));
  const hipRoll = Math.atan2(hipDy, Math.max(0.06, Math.abs(hipDx))) * frontalStability;

  const hipsQuat = new THREE.Quaternion().setFromEuler(
    new THREE.Euler(
      0,
      clampedHipYaw,
      THREE.MathUtils.clamp(hipRoll * 0.70, -0.30, 0.30),
      'YXZ'
    )
  );

  return {
    pos: [
      Number.isFinite(dx) ? THREE.MathUtils.clamp(dx, -1.35, 1.35) : 0,
      Number.isFinite(dy) ? THREE.MathUtils.clamp(dy, -0.35, 0.55) : 0,
      Number.isFinite(dz) ? dz : 0
    ],
    quat: sanitizeQuat(hipsQuat),
    hipYaw: clampedHipYaw,
    baseHipY,
    baseFootY,
    originX,
    torsoLenSm,
    baseTorsoLen
  };
}

// 2. MÓDULO DE DORSO Y PECHO
function solveSpineAndChest(shCenter, hipCenter, pRSh, pLSh, reliableTorsoYaw, hipsQuat, hipYaw) {
  const spineDir = new THREE.Vector3().subVectors(shCenter, hipCenter);
  // Evitar inversión del torso si en un frame MediaPipe pierde la altura de cadera
  const safeSpineY = Math.max(0.08, spineDir.y);

  const frontalWeight = Math.max(0.15, Math.abs(Math.cos(reliableTorsoYaw)));
  const spineRoll = -Math.atan2(spineDir.x, safeSpineY) * frontalWeight;

  const shDx = pRSh.x - pLSh.x;
  const shDy = pRSh.y - pLSh.y;

  const shRoll = Math.atan2(shDy, Math.max(0.06, Math.abs(shDx))) * frontalWeight;
  const spinePitch = THREE.MathUtils.clamp(
    Math.atan2(spineDir.z, safeSpineY) * 0.75 * frontalWeight,
    -0.38,
    0.38
  );

  const maxTwistDiff = 0.75;
  const safeChestYaw = THREE.MathUtils.clamp(
    reliableTorsoYaw,
    hipYaw - maxTwistDiff,
    hipYaw + maxTwistDiff
  );

  const chestWorld = sanitizeQuat(
    new THREE.Quaternion().setFromEuler(
      new THREE.Euler(
        spinePitch,
        safeChestYaw,
        THREE.MathUtils.clamp(spineRoll * 0.60 + shRoll * 0.40, -0.40, 0.40),
        'YXZ'
      )
    )
  );

  return {
    spine: sanitizeQuat(hipsQuat.clone().slerp(chestWorld, 0.5)),
    chest: chestWorld
  };
}

// 3. MÓDULO DE CABEZA (v2: la cabeza sigue la orientación REAL de la cara en el video)
//
// Idea: se estima hacia dónde mira la cara respecto a la cámara usando nariz + orejas
// (o nariz + ojos si las orejas no se ven). Ese yaw "de cámara" es el yaw mundial
// objetivo de la cabeza, y se usa la MISMA convención de signo que el torso
// (reliableTorsoYaw = -yaw2D), así cuando cuerpo y cara giran juntos el giro local
// de la cabeza queda ≈ 0, y cuando el cuerpo está de lado pero la cara mira al frente,
// la cabeza gira lo necesario para volver a mirar al frente.
function solveHead(
  pNose, pLEye, pREye, pLEar, pREar, fv,
  chestQuat, reliableTorsoYaw, config, prevHeadWorldArray, prevHeadYaw
) {
//const maxLocalYaw = lookAtCamera ? 1.05 : 0.45;
  const headGain = (config.headSensitivity ?? 1.0) * 0.55;
  const lookAtCamera = config?.headLookAtCamera !== false;

  const eyeDx = pREye.x - pLEye.x;
  const eyeDy = pREye.y - pLEye.y;
  const eyeCenter = new THREE.Vector3().addVectors(pLEye, pREye).multiplyScalar(0.5);
  const eyeWidth = Math.abs(eyeDx);

  // --- Yaw de la cara respecto a la cámara ---
  // nariz desplazada = R·sin(θ), media anchura de la cabeza = R·cos(θ)  ->  θ = atan2(...)
  const earsOk = Math.min(fv.lEar, fv.rEar) > 0.35;
  const earSpan = Math.abs(pREar.x - pLEar.x);
  const refX = earsOk ? (pLEar.x + pREar.x) * 0.5 : eyeCenter.x;
  const halfW = Math.max(earsOk ? earSpan * 0.5 : eyeWidth * 1.2, 0.012);
  const noseOff = pNose.x - refX;
  const faceYawCam = THREE.MathUtils.clamp(Math.atan2(noseOff, halfW), -1.45, 1.45);

  // Confianza: nariz y ojos visibles. Si el personaje está de espaldas o se pierde la cara,
  // la cabeza simplemente acompaña al torso.
  const eyeVis = (fv.lEye + fv.rEye) * 0.5;
  const reliability =
    THREE.MathUtils.clamp((fv.nose - 0.35) / 0.30, 0, 1) *
    THREE.MathUtils.clamp((eyeVis - 0.30) / 0.30, 0, 1);

  // "Mira de frente": zona suave alrededor de 0° donde se fuerza yaw mundial ≈ 0 (mirar a cámara)
  const frontLock = 1 - THREE.MathUtils.smoothstep(Math.abs(faceYawCam), 0.12, 0.38);
  const worldYawRaw = -faceYawCam * (1 - frontLock * 0.9);

  // Suavizado del yaw mundial objetivo (evita temblor del ruido de orejas/nariz)
  let worldYaw = worldYawRaw;
  if (Number.isFinite(prevHeadYaw)) {
    worldYaw = prevHeadYaw + wrapPi(worldYawRaw - prevHeadYaw) * 0.40;
  }

  // Yaw actual del pecho (mundo) y giro local que necesita la cabeza
  const chestYaw = new THREE.Euler().setFromQuaternion(chestQuat, 'YXZ').y;
  const maxLocalYaw = lookAtCamera ? 1.50 : 0.45;
  const desiredLocal = wrapPi(worldYaw - chestYaw);
  const gain = lookAtCamera ? 1.0 : 0.6;
  const localYaw = THREE.MathUtils.clamp(desiredLocal * reliability * gain, -maxLocalYaw, maxLocalYaw);

  // --- Pitch y roll (suaves) ---
  const frontFactor = Math.max(0.30, Math.cos(faceYawCam));
  const rawRoll = Math.atan2(eyeDy, Math.max(0.035, eyeWidth)) * headGain * frontFactor;
  const rawPitch = (-(pNose.y - eyeCenter.y + 0.02) * 1.15) * headGain * frontFactor;
  const localRoll = THREE.MathUtils.clamp(rawRoll, -0.25, 0.25) * (0.4 + 0.6 * reliability);
  const localPitch = THREE.MathUtils.clamp(rawPitch, -0.30, 0.30) * (0.4 + 0.6 * reliability);

  const localHeadOffset = sanitizeQuat(
    new THREE.Quaternion().setFromEuler(new THREE.Euler(localPitch, localYaw, localRoll, 'YXZ'))
  );

  let headWorld = sanitizeQuat(chestQuat.clone().multiply(localHeadOffset));

  // Suavizado final de la cabeza (más ligero que antes: el yaw ya viene filtrado)
  if (prevHeadWorldArray && prevHeadWorldArray.length === 4) {
    const prevHeadQ = sanitizeQuat(new THREE.Quaternion(...prevHeadWorldArray));
    if (prevHeadQ.dot(headWorld) < 0) {
      headWorld.set(-headWorld.x, -headWorld.y, -headWorld.z, -headWorld.w);
    }
    headWorld = sanitizeQuat(prevHeadQ.slerp(headWorld, 0.5));
  }

  return {
    // El cuello toma ~la mitad del giro para que se vea natural cuando el giro es grande
    neck: sanitizeQuat(chestQuat.clone().slerp(headWorld, 0.5)),
    head: headWorld,
    headYaw: worldYaw
  };
}

// 4. MÓDULO DE BRAZOS CON ESCUDO VOLUMÉTRICO ANTI-COLISIÓN
function solveArms(
  pLSh, pRSh, pLEl, pREl, pLWr, pRWr,
  pLPinky, pRPinky, pLIndex, pRIndex,
  pLHip, pRHip,
  chestQuat, visL, visR, config, prevWorld, prevMeta
) {
  const safeArmOffsetDeg = Math.max(-2, config.armVerticalOffset || 0);
  const armOffsetRad = THREE.MathUtils.degToRad(safeArmOffsetDeg);
  const leftBias = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -armOffsetRad);
  const rightBias = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), armOffsetRad);

  const chestInv = chestQuat.clone().invert();

  const lFallbackLocal = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.18, 0.15, 0.58, 'YXZ'));
  const rFallbackLocal = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.18, -0.15, -0.58, 'YXZ'));

  const buildArmWorld = (
    pSh, pEl, pWr, pPinky, pIndex, pHip,
    isLeft, vis, bias, fallbackLocal, prevUpper, prevLower, prevHand,
    prevIdle, prevRel, prevMotion
  ) => {
    const rawUpperVec = new THREE.Vector3().subVectors(pEl, pSh);

    if (vis.el < 0.04 || rawUpperVec.lengthSq() < 0.0001) {
      const uWorld = prevUpper
        ? sanitizeQuat(new THREE.Quaternion(...prevUpper))
        : sanitizeQuat(chestQuat.clone().multiply(fallbackLocal));
      const lWorld = prevLower ? sanitizeQuat(new THREE.Quaternion(...prevLower)) : uWorld.clone();
      const hWorld = prevHand ? sanitizeQuat(new THREE.Quaternion(...prevHand)) : lWorld.clone();
      return {
        upper: uWorld, lower: lWorld, hand: hWorld,
        idle: prevIdle ?? 0, rel: prevRel ?? null, motion: prevMotion ?? 0
      };
    }

    const sideSign = isLeft ? -1 : 1;
    const restDir = new THREE.Vector3(sideSign, 0, 0);
    const frontSign = config.invertArmZ ? 1 : -1;

    const upperDirLocal = rawUpperVec.clone().applyQuaternion(chestInv);
    const rawLowerVec = new THREE.Vector3().subVectors(pWr, pEl);
    const lowerInChest = rawLowerVec.clone().applyQuaternion(chestInv);

    const handInwardRatio = THREE.MathUtils.clamp((pSh.x - pWr.x) * sideSign * 2.8, 0, 1.2);
    const elbowInwardRatio = THREE.MathUtils.clamp((pSh.x - pEl.x) * sideSign * 3.0, 0, 1.0);

    const uNormTemp = upperDirLocal.clone().normalize();
    const lNormTemp = lowerInChest.lengthSq() > 0.0001 ? lowerInChest.clone().normalize() : uNormTemp;
    const elbowBend = THREE.MathUtils.clamp(1.0 - uNormTemp.dot(lNormTemp), 0, 1.2);

    const isArmDown = uNormTemp.y < 0.15;
    const outwardComp = uNormTemp.x * sideSign;
    const forwardComp = uNormTemp.z * frontSign;

    if (isArmDown) {
      const minOutwardWhenDown = 0.24 * Math.abs(uNormTemp.y);

      if (outwardComp < minOutwardWhenDown) {
        if (handInwardRatio < 0.35) {
          upperDirLocal.x = sideSign * Math.max(outwardComp, minOutwardWhenDown) * upperDirLocal.length();
        } else {
          const neededFront = (minOutwardWhenDown - outwardComp) * 1.15 + 0.22;
          if (forwardComp < neededFront) {
            upperDirLocal.z = frontSign * neededFront * upperDirLocal.length();
          }
        }
      }

      if (handInwardRatio > 0.18 || elbowInwardRatio > 0.15) {
        if ((upperDirLocal.z * frontSign) < 0.08) {
          upperDirLocal.z = frontSign * (0.12 + handInwardRatio * 0.24);
        } else {
          upperDirLocal.z += frontSign * (handInwardRatio * 0.18);
        }
      } else {
        upperDirLocal.z *= 0.5;
      }
    } else {
      upperDirLocal.z *= 0.55;
    }

    upperDirLocal.normalize();

    // --- Detección de brazo en reposo: colgando + casi quieto (respecto a la cadera) ---
    const relWr = new THREE.Vector3().subVectors(pWr, pHip);
    let motion = 0;
    if (prevRel) {
      const sp = relWr.distanceTo(new THREE.Vector3(...prevRel));
      motion = (prevMotion ?? sp) * 0.8 + sp * 0.2;
    }
    const hangingPose =
      THREE.MathUtils.smoothstep(-uNormTemp.y, 0.70, 0.92) *
      THREE.MathUtils.smoothstep(-lNormTemp.y, 0.45, 0.80) *
      THREE.MathUtils.smoothstep(pHip.y + 0.12 - pWr.y, 0.0, 0.10);
    const stillness = 1 - THREE.MathUtils.smoothstep(motion, 0.006, 0.020);
    const idleTarget = config.relaxIdleArms === false ? 0 : hangingPose * stillness;
    const idleBase = prevIdle ?? 0;
    // entra despacio al reposo, sale rápido cuando el brazo vuelve a moverse
    const idle = idleBase + (idleTarget - idleBase) * (idleTarget > idleBase ? 0.12 : 0.40);

    if (idle > 0.001) {
      const relaxedUpper = new THREE.Vector3(sideSign * 0.14, -1, frontSign * 0.04).normalize();
      upperDirLocal.lerp(relaxedUpper, Math.min(1, idle * 0.9)).normalize();
    }

    const localUpperQuat = new THREE.Quaternion().setFromUnitVectors(restDir, upperDirLocal).multiply(bias);

    if (lowerInChest.lengthSq() > 0.0001) {
      const wristVertical = lowerInChest.y;
      let targetSwivel = 0;

      if (wristVertical < 0.08) {
        targetSwivel = sideSign * (0.15 + handInwardRatio * 0.90);
      } else {
        targetSwivel = sideSign * (0.12 + handInwardRatio * 0.35);
      }

      const bendFactor = THREE.MathUtils.smoothstep(elbowBend, 0.12, 0.75);
      const effectiveTwist = targetSwivel * bendFactor * (1 - idle);

      const axialTwist = new THREE.Quaternion().setFromAxisAngle(restDir, effectiveTwist);
      localUpperQuat.multiply(axialTwist);
    }

    let upperWorld = sanitizeQuat(chestQuat.clone().multiply(localUpperQuat));

    if (vis.el < 0.20 && prevUpper) {
      const prevU = sanitizeQuat(new THREE.Quaternion(...prevUpper));
      upperWorld = sanitizeQuat(prevU.slerp(upperWorld, 0.45));
    }

    let lowerWorld = upperWorld.clone();

    if (vis.wr >= 0.04 && rawLowerVec.lengthSq() >= 0.0001) {
      const safeLowerChest = lowerInChest.clone();

      const isForearmGoingBack = (safeLowerChest.z * frontSign) < -0.01;
      if (isForearmGoingBack && (elbowBend > 0.15 || handInwardRatio > 0.15)) {
        safeLowerChest.z = -safeLowerChest.z;
      }

      const estElbowPos = new THREE.Vector3(sideSign * 0.15, 0.08, 0).addScaledVector(upperDirLocal, 0.23);
      const estLowerDir = safeLowerChest.clone().normalize();
      const estWristPos = estElbowPos.clone().addScaledVector(estLowerDir, 0.23);

      const inTorsoHeight = estWristPos.y > -0.38 && estWristPos.y < 0.22;
      const inTorsoWidth = Math.abs(estWristPos.x) < 0.22;

      if (inTorsoHeight && inTorsoWidth) {
        const centerProximity = 1.0 - Math.min(1.0, Math.abs(estWristPos.x) / 0.22);
        const minFrontSurfaceZ = 0.11 + centerProximity * 0.10;

        const currentWristFrontZ = estWristPos.z * frontSign;
        if (currentWristFrontZ < minFrontSurfaceZ) {
          const pushNeeded = (minFrontSurfaceZ - currentWristFrontZ) * 1.45;
          safeLowerChest.z += frontSign * pushNeeded;
        }
      } else if (inTorsoHeight && Math.abs(estWristPos.x) < 0.26 && elbowBend < 0.25) {
        const minSideX = 0.22;
        if (estWristPos.x * sideSign < minSideX) {
          safeLowerChest.x = sideSign * Math.max(Math.abs(safeLowerChest.x), 0.08);
          safeLowerChest.z += frontSign * 0.06;
        }
      }

      if (idle > 0.001) {
        // En reposo el antebrazo cuelga recto (con una leve flexión natural), sin el empuje hacia delante
        const relaxedLower = new THREE.Vector3(sideSign * 0.04, -1, frontSign * 0.12).normalize();
        safeLowerChest.normalize().lerp(relaxedLower, Math.min(1, idle * 0.9));
      }

      const lowerWorldVec = safeLowerChest.applyQuaternion(chestQuat);
      const upperInv = upperWorld.clone().invert();
      const lowerDirLocal = lowerWorldVec.applyQuaternion(upperInv);

      if (elbowBend > 0.18 && (lowerDirLocal.z * frontSign) < 0) {
        lowerDirLocal.z = Math.abs(lowerDirLocal.z) * frontSign;
      }

      lowerDirLocal.normalize();

      const rawLowerQuat = new THREE.Quaternion().setFromUnitVectors(restDir, lowerDirLocal);
      const boostAmount = 1.0 + (elbowBend > 0.25 ? 0.14 : 0.0);
      const boostedLowerQuat = new THREE.Quaternion().slerp(rawLowerQuat, boostAmount);

      lowerWorld = sanitizeQuat(upperWorld.clone().multiply(boostedLowerQuat));

      if (vis.wr < 0.20 && prevLower) {
        const prevL = sanitizeQuat(new THREE.Quaternion(...prevLower));
        lowerWorld = sanitizeQuat(prevL.slerp(lowerWorld, 0.45));
      }
    } else if (prevLower) {
      lowerWorld = sanitizeQuat(new THREE.Quaternion(...prevLower));
    }

    let handWorld = lowerWorld.clone();
    const distToHip = pWr.distanceTo(pHip);

    if (distToHip < 0.24 && pWr.y < pSh.y - 0.12 && handInwardRatio > 0.35) {
      const hipHandEuler = new THREE.Euler(
        THREE.MathUtils.degToRad(-60),
        0,
        THREE.MathUtils.degToRad(isLeft ? -22 : 22),
        'YXZ'
      );
      handWorld = sanitizeQuat(lowerWorld.clone().multiply(new THREE.Quaternion().setFromEuler(hipHandEuler)));
    } else if (pIndex && pPinky) {
      const handMid = new THREE.Vector3().addVectors(pIndex, pPinky).multiplyScalar(0.5);
      const rawHandVec = new THREE.Vector3().subVectors(handMid, pWr);
      if (rawHandVec.lengthSq() > 0.0001) {
        const lowerInv = lowerWorld.clone().invert();
        const handDirLocal = rawHandVec.clone().applyQuaternion(lowerInv).normalize();
        const localHandQuat = new THREE.Quaternion()
          .setFromUnitVectors(restDir, handDirLocal)
          .slerp(new THREE.Quaternion(), 0.45);
        handWorld = sanitizeQuat(lowerWorld.clone().multiply(localHandQuat));
      }
    }

    if (idle > 0.001) {
      handWorld = sanitizeQuat(handWorld.clone().slerp(lowerWorld, Math.min(1, idle * 0.7)));
    }

    return {
      upper: upperWorld, lower: lowerWorld, hand: handWorld,
      idle, rel: relWr.toArray(), motion
    };
  };

  const leftArm = buildArmWorld(
    pLSh, pLEl, pLWr, pLPinky, pLIndex, pLHip,
    true, visL, leftBias, lFallbackLocal,
    prevWorld?.leftUpperArm, prevWorld?.leftLowerArm, prevWorld?.leftHand,
    prevMeta?.left?.idle, prevMeta?.left?.rel, prevMeta?.left?.motion
  );
  const rightArm = buildArmWorld(
    pRSh, pREl, pRWr, pRPinky, pRIndex, pRHip,
    false, visR, rightBias, rFallbackLocal,
    prevWorld?.rightUpperArm, prevWorld?.rightLowerArm, prevWorld?.rightHand,
    prevMeta?.right?.idle, prevMeta?.right?.rel, prevMeta?.right?.motion
  );

  return {
    leftUpperArm: leftArm.upper,
    leftLowerArm: leftArm.lower,
    leftHand: leftArm.hand,
    rightUpperArm: rightArm.upper,
    rightLowerArm: rightArm.lower,
    rightHand: rightArm.hand,
    meta: {
      left: { idle: leftArm.idle, rel: leftArm.rel, motion: leftArm.motion },
      right: { idle: rightArm.idle, rel: rightArm.rel, motion: rightArm.motion }
    }
  };
}

// 5. MÓDULO DE DEDOS DE LAS MANOS
function solveFingers(
  pLEl, pREl, pLWr, pRWr,
  pLPinky, pRPinky, pLIndex, pRIndex, pLThumb, pRThumb,
  pLHip, pRHip, config
) {
  const localFingerQuats = {};

  if (!config?.fingerTrackingEnabled) {
    FINGER_BONE_LIST.forEach((b) => {
      localFingerQuats[b] = new THREE.Quaternion(0, 0, 0, 1);
    });
    return localFingerQuats;
  }

  const preset = config.fingerPreset || 'auto';
  const curlMul = config.fingerCurlSensitivity ?? 1.0;

  const computeHandCurls = (pEl, pWr, pPinky, pIndex, pThumb, pHip, isLeft) => {
    const sideSign = isLeft ? 1 : -1;

    if (preset === 'open') {
      return { thumb: 0.08, index: 0.06, middle: 0.08, ring: 0.10, little: 0.12 };
    }
    if (preset === 'fist') {
      return { thumb: 0.85, index: 1.15, middle: 1.20, ring: 1.25, little: 1.30 };
    }
    if (preset === 'peace') {
      return { thumb: 0.75, index: 0.05, middle: 0.05, ring: 1.15, little: 1.20 };
    }
    if (preset === 'rock') {
      return { thumb: 0.35, index: 0.05, middle: 1.18, ring: 1.20, little: 0.05 };
    }

    const forearmLen = Math.max(0.08, pWr.distanceTo(pEl));
    const indexDist = pIndex.distanceTo(pWr);
    const pinkyDist = pPinky.distanceTo(pWr);
    const spreadDist = pIndex.distanceTo(pPinky);
    const thumbSpread = pThumb ? pThumb.distanceTo(pPinky) : spreadDist;

    const opennessRatio = ((indexDist + pinkyDist) * 0.5 + spreadDist * 1.1) / forearmLen;
    let baseCurl = THREE.MathUtils.clamp((0.56 - opennessRatio) * 2.6 * curlMul, 0.12, 1.15);

    if (pWr.distanceTo(pHip) < 0.25) {
      baseCurl = Math.min(baseCurl, 0.38);
    }

    const thumbCurl = THREE.MathUtils.clamp((0.45 - (thumbSpread / forearmLen)) * 2.0 * curlMul, 0.15, 0.85);

    return {
      thumb: thumbCurl,
      index: baseCurl * 0.85,
      middle: baseCurl * 0.95,
      ring: baseCurl * 1.05,
      little: baseCurl * 1.15,
      sideSign
    };
  };

  const applyHandToBones = (prefix, curls, isLeft) => {
    const zSign = isLeft ? 1 : -1;
    const ySign = isLeft ? -1 : 1;

    const fingerNames = ['Index', 'Middle', 'Ring', 'Little'];
    const curlKeys = ['index', 'middle', 'ring', 'little'];

    fingerNames.forEach((fName, idx) => {
      const c = curls[curlKeys[idx]] ?? 0.25;
      localFingerQuats[`${prefix}${fName}Proximal`] = sanitizeQuat(
        new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, zSign * c * 0.65, 'XYZ'))
      );
      localFingerQuats[`${prefix}${fName}Intermediate`] = sanitizeQuat(
        new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, zSign * c * 0.75, 'XYZ'))
      );
      localFingerQuats[`${prefix}${fName}Distal`] = sanitizeQuat(
        new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, zSign * c * 0.55, 'XYZ'))
      );
    });

    const tc = curls.thumb ?? 0.25;
    localFingerQuats[`${prefix}ThumbProximal`] = sanitizeQuat(
      new THREE.Quaternion().setFromEuler(new THREE.Euler(0.15, ySign * tc * 0.55, zSign * tc * 0.30, 'XYZ'))
    );
    localFingerQuats[`${prefix}ThumbIntermediate`] = sanitizeQuat(
      new THREE.Quaternion().setFromEuler(new THREE.Euler(0, ySign * tc * 0.60, zSign * tc * 0.35, 'XYZ'))
    );
    localFingerQuats[`${prefix}ThumbDistal`] = sanitizeQuat(
      new THREE.Quaternion().setFromEuler(new THREE.Euler(0, ySign * tc * 0.45, zSign * tc * 0.30, 'XYZ'))
    );
  };

  const leftCurls = computeHandCurls(pLEl, pLWr, pLPinky, pLIndex, pLThumb, pLHip, true);
  const rightCurls = computeHandCurls(pREl, pRWr, pRPinky, pRIndex, pRThumb, pRHip, false);

  applyHandToBones('left', leftCurls, true);
  applyHandToBones('right', rightCurls, false);

  return localFingerQuats;
}

// 6. MÓDULO DE PIERNAS (Aumenta el rango SOLO en el muslo de la pierna levantada sin deformar los pies)
function solveLegs(pLHip, pRHip, pLKn, pRKn, pLAn, pRAn, hipsQuat, hipYaw, config, prevLegs) {
  const facingCos = Math.cos(hipYaw);
  const facingSign = facingCos >= 0 ? 1 : -1;
  const isTurned = Math.abs(hipYaw) > 0.65;
  const isHighBoostActive = Boolean(config?.highLegBoost);

  // Detectar cuál de las dos piernas está levantada comparando la altura de los tobillos y rodillas
  const lAnkleLift = Math.max(0, pLAn.y - pRAn.y);
  const rAnkleLift = Math.max(0, pRAn.y - pLAn.y);
  const lKneeLift = Math.max(0, (pLKn.y - pLHip.y) + 0.16);
  const rKneeLift = Math.max(0, (pRKn.y - pRHip.y) + 0.16);

  // Solo la pierna que realmente está más arriba recibe el desbloqueo de muslo
  const lIsLifted = isHighBoostActive && (lAnkleLift > 0.025 || (lKneeLift > rKneeLift + 0.03));
  const rIsLifted = isHighBoostActive && (rAnkleLift > 0.025 || (rKneeLift > lKneeLift + 0.03));

  // Si la pierna NO está en modo alto, mantenemos el candado clásico; si sí lo está, dejamos subir la rodilla hasta la cintura (+0.08)
  if (!lIsLifted && pLKn.y > pLHip.y - 0.03) pLKn.y = pLHip.y - 0.14;
  if (!rIsLifted && pRKn.y > pRHip.y - 0.03) pRKn.y = pRHip.y - 0.14;
  if (lIsLifted && pLKn.y > pLHip.y + 0.08) pLKn.y = pLHip.y + 0.08;
  if (rIsLifted && pRKn.y > pRHip.y + 0.08) pRKn.y = pRHip.y + 0.08;

  if (!isTurned && Math.abs(pLAn.y - pRAn.y) < 0.06) {
    if (pLKn.x > pRKn.x) {
      const midX = (pLKn.x + pRKn.x) * 0.5;
      pLKn.x = midX - 0.02;
      pRKn.x = midX + 0.02;
    }
    if (pLAn.x > pRAn.x) {
      const midX = (pLAn.x + pRAn.x) * 0.5;
      pLAn.x = midX - 0.02;
      pRAn.x = midX + 0.02;
    }
  }

  const buildLocalLegVectors = (pHip, pKn, pAn, isLeft, ankleLift, allowHighThigh) => {
    const sideSign = isLeft ? -1 : 1;

    let thighDx = (pKn.x - pHip.x) * facingSign;
    let calfDx = (pAn.x - pKn.x) * facingSign;

    if (isTurned && !allowHighThigh) {
      if (thighDx * sideSign < -0.02) thighDx = sideSign * -0.02;
      if (calfDx * sideSign < -0.03) calfDx = sideSign * -0.03;
      thighDx *= 0.65;
      calfDx *= 0.65;
    }

    // Si allowHighThigh está activo en esta pierna, liberamos thighDy para que el muslo suba sin aplastarse
    let thighDy = allowHighThigh
      ? Math.min(0.06, (pKn.y - pHip.y) + ankleLift * 0.35)
      : Math.min(-0.04, pKn.y - pHip.y);

    // La pantorrilla (calfDy) NUNCA se deforma hacia arriba para que el pie siempre apunte al suelo con naturalidad
    const calfDy = Math.min(-0.05, pAn.y - pKn.y);

    let thighDz = Math.min(-0.02, (pKn.z - pHip.z) * 0.35 * facingSign);
    let calfDz = Math.max(0.03, (pAn.z - pKn.z) * 0.35 * facingSign);

    if (allowHighThigh) {
      // Adelantamos el muslo suavemente para dar el rango extra sin quebrar la rodilla ni el pie
      const liftFactor = Math.max(ankleLift, 0.08);
      thighDz -= liftFactor * 0.65;
      thighDx *= 1.25;
      calfDz += liftFactor * 0.45;
    } else if (ankleLift > 0.035) {
      thighDz -= ankleLift * 0.45;
      calfDz += ankleLift * 0.90;
    }

    return {
      thighVec: new THREE.Vector3(thighDx, thighDy, thighDz),
      calfVec: new THREE.Vector3(calfDx, calfDy, calfDz)
    };
  };

  const leftVecs = buildLocalLegVectors(pLHip, pLKn, pLAn, true, lAnkleLift, lIsLifted);
  const rightVecs = buildLocalLegVectors(pRHip, pRKn, pRAn, false, rAnkleLift, rIsLifted);

  const legSpreadRad = THREE.MathUtils.degToRad(config.legSpreadOffset || 0);
  const lLegBias = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -legSpreadRad);
  const rLegBias = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), legSpreadRad);

  const zero = new THREE.Vector3(0, 0, 0);
  // Solo aumentamos el ángulo máximo en el muslo (UpperLeg) de la pierna que realmente está levantada (de 1.35 a 1.95 rad)
  const lMaxThighAngle = lIsLifted ? 1.95 : 1.35;
  const rMaxThighAngle = rIsLifted ? 1.95 : 1.35;

  const lUpperLocal = getVecQuat(zero, leftVecs.thighVec, T_POSE_DIRS.leftUpperLeg, lMaxThighAngle).multiply(lLegBias);
  const lLowerLocal = getVecQuat(zero, leftVecs.calfVec, T_POSE_DIRS.leftLowerLeg, 1.80);
  const rUpperLocal = getVecQuat(zero, rightVecs.thighVec, T_POSE_DIRS.rightUpperLeg, rMaxThighAngle).multiply(rLegBias);
  const rLowerLocal = getVecQuat(zero, rightVecs.calfVec, T_POSE_DIRS.rightLowerLeg, 1.80);

  const lUpper = sanitizeQuat(hipsQuat.clone().multiply(lUpperLocal));
  const lLower = sanitizeQuat(hipsQuat.clone().multiply(lLowerLocal));
  const rUpper = sanitizeQuat(hipsQuat.clone().multiply(rUpperLocal));
  const rLower = sanitizeQuat(hipsQuat.clone().multiply(rLowerLocal));

  return {
    leftUpperLeg: prevLegs?.leftUpperLeg ? prevLegs.leftUpperLeg.clone().slerp(lUpper, 0.7) : lUpper,
    leftLowerLeg: prevLegs?.leftLowerLeg ? prevLegs.leftLowerLeg.clone().slerp(lLower, 0.7) : lLower,
    leftFoot: hipsQuat.clone(),
    rightUpperLeg: prevLegs?.rightUpperLeg ? prevLegs.rightUpperLeg.clone().slerp(rUpper, 0.7) : rUpper,
    rightLowerLeg: prevLegs?.rightLowerLeg ? prevLegs.rightLowerLeg.clone().slerp(rLower, 0.7) : rLower,
    rightFoot: hipsQuat.clone()
  };
}

// 7. MÓDULO DE EXPRESIONES FACIALES LIGERAS (OPCIÓN 1)
function solveFaceExpressions(screenLandmarks, pLWr, pRWr, pLSh, pRSh, hipsPos, prevFrame, config) {
  if (!config?.faceTrackingEnabled) {
    return { blink: 0, mouthOpen: 0, smile: 0, relaxed: 0 };
  }

  const nose = screenLandmarks[0];
  const lEye = screenLandmarks[2];
  const rEye = screenLandmarks[5];
  const mLeft = screenLandmarks[9];
  const mRight = screenLandmarks[10];

  let rawMouthOpen = 0;
  let rawSmile = 0;

  if (nose && lEye && rEye && mLeft && mRight) {
    const eyeWidth = Math.hypot(rEye.x - lEye.x, rEye.y - lEye.y);
    const mouthWidth = Math.hypot(mRight.x - mLeft.x, mRight.y - mLeft.y);
    const mouthMidY = (mLeft.y + mRight.y) * 0.5;
    const noseToMouthY = Math.max(0, mouthMidY - nose.y);

    if (eyeWidth > 0.012) {
      const widthRatio = mouthWidth / eyeWidth;
      const dropRatio = noseToMouthY / eyeWidth;

      rawMouthOpen = THREE.MathUtils.clamp((dropRatio - 0.52) * 3.2, 0, 0.85);
      rawSmile = THREE.MathUtils.clamp((widthRatio - 0.82) * 2.5, 0, 0.90);
    }
  }

  const handsUp = (pLWr.y > pLSh.y ? 0.25 : 0) + (pRWr.y > pRSh.y ? 0.25 : 0);
  const jumpEnergy = Math.max(0, (hipsPos?.[1] || 0) * 1.5);
  const danceEnergy = THREE.MathUtils.clamp(handsUp + jumpEnergy, 0, 0.65);

  const preset = config?.faceExpressionPreset || 'auto';
  if (preset === 'happy') {
    rawSmile = Math.max(rawSmile, 0.78);
    rawMouthOpen = Math.max(rawMouthOpen * 0.6, danceEnergy * 0.35);
  } else if (preset === 'sing') {
    rawMouthOpen = Math.max(rawMouthOpen, 0.35 + danceEnergy * 0.55);
    rawSmile = Math.max(rawSmile * 0.5, 0.25);
  } else if (preset === 'relaxed') {
    rawSmile = 0.35;
    rawMouthOpen = rawMouthOpen * 0.4;
  } else {
    rawSmile = THREE.MathUtils.clamp(rawSmile + danceEnergy * 0.55, 0.15, 0.85);
    rawMouthOpen = THREE.MathUtils.clamp(rawMouthOpen + danceEnergy * 0.35, 0, 0.80);
  }

  const nowSec = performance.now() * 0.001;
  const blinkCycle = nowSec % 3.4;
  let targetBlink = 0;
  if (blinkCycle > 3.22 && blinkCycle < 3.38) {
    targetBlink = 1.0 - Math.abs((blinkCycle - 3.30) / 0.08);
  }

  const prevExp = prevFrame?.expressions || { blink: 0, mouthOpen: 0, smile: 0, relaxed: 0 };

  return {
    blink: THREE.MathUtils.clamp(prevExp.blink * 0.25 + targetBlink * 0.75, 0, 1),
    mouthOpen: THREE.MathUtils.clamp(prevExp.mouthOpen * 0.55 + rawMouthOpen * 0.45, 0, 1),
    smile: THREE.MathUtils.clamp(prevExp.smile * 0.65 + rawSmile * 0.35, 0, 1),
    relaxed: preset === 'relaxed' ? 0.65 : 0
  };
}

function getVecQuat(pStart, pEnd, restDir, maxAngle = 2.4) {
  const dir = new THREE.Vector3().subVectors(pEnd, pStart);
  if (dir.lengthSq() < 0.000001) return new THREE.Quaternion();
  dir.normalize();
  const q = new THREE.Quaternion();
  const dot = restDir.dot(dir);
  if (dot < -0.999) {
    q.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI);
  } else {
    q.setFromUnitVectors(restDir, dir);
  }
  sanitizeQuat(q);

  const angle = 2 * Math.acos(Math.max(-1, Math.min(1, Math.abs(q.w))));
  if (angle > maxAngle && angle > 0.001) {
    q.slerp(new THREE.Quaternion(0, 0, 0, 1), 1 - maxAngle / angle);
  }
  return sanitizeQuat(q);
}

export function solvePoseToVRM(worldLandmarks, screenLandmarks, prevFrame, config, videoAspect = 0.6) {
  if (!screenLandmarks || screenLandmarks.length === 0) {
    return prevFrame || {
      hipsPos: [0, 0, 0],
      localQuats: {},
      worldQuats: {},
      torsoYaw: 0,
      headYaw: 0,
      expressions: { blink: 0, mouthOpen: 0, smile: 0, relaxed: 0 }
    };
  }

  const mirrorSign = config.mirror ? -1 : 1;
  // App marca prevFrame.cut = true cuando detecta un cambio de plano: no se mezcla con el frame anterior
  const cutReset = Boolean(prevFrame?.cut);

  const L = config.mirror
    ? { sh: 11, el: 13, wr: 15, pinky: 17, index: 19, thumb: 21, hip: 23, kn: 25, an: 27, eye: 2, ear: 7 }
    : { sh: 12, el: 14, wr: 16, pinky: 18, index: 20, thumb: 22, hip: 24, kn: 26, an: 28, eye: 5, ear: 8 };

  const R = config.mirror
    ? { sh: 12, el: 14, wr: 16, pinky: 18, index: 20, thumb: 22, hip: 24, kn: 26, an: 28, eye: 5, ear: 8 }
    : { sh: 11, el: 13, wr: 15, pinky: 17, index: 19, thumb: 21, hip: 23, kn: 25, an: 27, eye: 2, ear: 7 };

  const pt = (idx) => hybridVec3(screenLandmarks[idx], worldLandmarks?.[idx], config, videoAspect);
  const vis = (idx) => screenLandmarks[idx]?.visibility ?? 1.0;

  const pLSh = pt(L.sh), pRSh = pt(R.sh);
  const pLEl = pt(L.el), pREl = pt(R.el);
  const pLWr = pt(L.wr), pRWr = pt(R.wr);
  const pLPinky = pt(L.pinky), pRPinky = pt(R.pinky);
  const pLIndex = pt(L.index), pRIndex = pt(R.index);
  const pLThumb = pt(L.thumb), pRThumb = pt(R.thumb);
  const pLHip = pt(L.hip), pRHip = pt(R.hip);
  const pLKn = pt(L.kn), pRKn = pt(R.kn);
  const pLAn = pt(L.an), pRAn = pt(R.an);
  const pNose = pt(0), pLEye = pt(L.eye), pREye = pt(R.eye);
  const pLEar = pt(L.ear), pREar = pt(R.ear);

  const rawShDz = getRawDepthDiff(R.sh, L.sh, worldLandmarks, screenLandmarks, config.invertShoulderZ);

  const hipCenter = new THREE.Vector3().addVectors(pLHip, pRHip).multiplyScalar(0.5);
  const shCenter = new THREE.Vector3().addVectors(pLSh, pRSh).multiplyScalar(0.5);

  const rawTorsoYaw = -computeReliableYaw(
    pRSh.x - pLSh.x,
    rawShDz,
    pNose,
    pLEye,
    pREye,
    shCenter,
    hipCenter,
    pLKn,
    pRKn,
    Boolean(config.unlockRotation),
    prevFrame?.torsoYaw ?? 0
  );

  const prevYaw = cutReset ? rawTorsoYaw : (prevFrame?.torsoYaw ?? rawTorsoYaw);
  const unwrappedYaw = unwrapAngle(rawTorsoYaw, prevYaw);
  const reliableTorsoYaw = prevYaw + (unwrappedYaw - prevYaw) * 0.35;

  const hipsData = solveHips(pLHip, pRHip, reliableTorsoYaw, screenLandmarks, mirrorSign, cutReset ? null : prevFrame, config);
  const torsoData = solveSpineAndChest(shCenter, hipCenter, pRSh, pLSh, reliableTorsoYaw, hipsData.quat, hipsData.hipYaw);
  const headData = solveHead(
    pNose,
    pLEye,
    pREye,
    pLEar,
    pREar,
    {
      nose: vis(0),
      lEye: vis(L.eye),
      rEye: vis(R.eye),
      lEar: vis(L.ear),
      rEar: vis(R.ear)
    },
    torsoData.chest,
    reliableTorsoYaw,
    config,
    cutReset ? undefined : prevFrame?.worldQuats?.head,
    cutReset ? undefined : prevFrame?.headYaw
  );
  const { meta: armMeta, ...armsData } = solveArms(
    pLSh, pRSh, pLEl, pREl, pLWr, pRWr,
    pLPinky, pRPinky, pLIndex, pRIndex,
    pLHip, pRHip,
    torsoData.chest,
    { el: vis(L.el), wr: vis(L.wr) },
    { el: vis(R.el), wr: vis(R.wr) },
    config,
    prevFrame?.worldQuats,
    cutReset ? undefined : prevFrame?.armMeta
  );

  const fingersLocalData = solveFingers(
    pLEl, pREl, pLWr, pRWr,
    pLPinky, pRPinky, pLIndex, pRIndex, pLThumb, pRThumb,
    pLHip, pRHip, config
  );

  const prevLegQuats = prevFrame?.worldQuats && prevFrame.worldQuats.leftUpperLeg ? {
    leftUpperLeg: sanitizeQuat(new THREE.Quaternion(...prevFrame.worldQuats.leftUpperLeg)),
    leftLowerLeg: sanitizeQuat(new THREE.Quaternion(...prevFrame.worldQuats.leftLowerLeg)),
    rightUpperLeg: sanitizeQuat(new THREE.Quaternion(...prevFrame.worldQuats.rightUpperLeg)),
    rightLowerLeg: sanitizeQuat(new THREE.Quaternion(...prevFrame.worldQuats.rightLowerLeg))
  } : null;

  let legsData = solveLegs(
    pLHip,
    pRHip,
    pLKn,
    pRKn,
    pLAn,
    pRAn,
    hipsData.quat,
    hipsData.hipYaw,
    config,
    prevLegQuats
  );

  // Plano cerrado (cara/torso): cadera y rodillas casi invisibles -> MediaPipe inventa las piernas.
  // Con la opción de cambio de plano activa, se conserva la pose previa de las piernas y la posición de cadera.
  const lowBodyVis = (vis(23) + vis(24) + vis(25) + vis(26)) / 4;
  const lowBodyHidden = Boolean(config.shotCutDetection) && lowBodyVis < 0.35;
  if (lowBodyHidden && prevLegQuats) {
    legsData = {
      ...legsData,
      leftUpperLeg: prevLegQuats.leftUpperLeg,
      leftLowerLeg: prevLegQuats.leftLowerLeg,
      rightUpperLeg: prevLegQuats.rightUpperLeg,
      rightLowerLeg: prevLegQuats.rightLowerLeg
    };
  }
  if (lowBodyHidden && prevFrame) {
    hipsData.pos = [...(prevFrame.hipsPos || [0, 0, 0])];
    hipsData.baseHipY = prevFrame.baseHipY ?? hipsData.baseHipY;
    hipsData.baseFootY = prevFrame.baseFootY ?? hipsData.baseFootY;
    hipsData.originX = prevFrame.originX ?? hipsData.originX;
  }

  const worldQuats = {
    hips: hipsData.quat,
    spine: torsoData.spine,
    chest: torsoData.chest,
    neck: headData.neck,
    head: headData.head,
    ...armsData,
    ...legsData
  };

  const PARENT_MAP = {
    hips: null, spine: 'hips', chest: 'spine', neck: 'chest', head: 'neck',
    leftUpperArm: 'chest', leftLowerArm: 'leftUpperArm', leftHand: 'leftLowerArm',
    rightUpperArm: 'chest', rightLowerArm: 'rightUpperArm', rightHand: 'rightLowerArm',
    leftUpperLeg: 'hips', leftLowerLeg: 'leftUpperLeg', leftFoot: 'leftLowerLeg',
    rightUpperLeg: 'hips', rightLowerLeg: 'rightUpperLeg', rightFoot: 'rightLowerLeg'
  };

  const localQuats = {};
  const serializedWorld = {};

  const FINGER_SET = new Set(FINGER_BONE_LIST);

  for (const bone of BONE_LIST) {
    if (FINGER_SET.has(bone)) {
      localQuats[bone] = sanitizeQuat(fingersLocalData[bone] || new THREE.Quaternion());
      serializedWorld[bone] = localQuats[bone].toArray();
    } else {
      sanitizeQuat(worldQuats[bone]);
      serializedWorld[bone] = worldQuats[bone].toArray();

      const parentName = PARENT_MAP[bone];
      if (!parentName) {
        localQuats[bone] = worldQuats[bone].clone();
      } else {
        const parentInv = worldQuats[parentName].clone().invert();
        localQuats[bone] = sanitizeQuat(parentInv.multiply(worldQuats[bone].clone()));
      }
    }

    if (config.vrm0Mode) {
      localQuats[bone].x = -localQuats[bone].x;
      localQuats[bone].z = -localQuats[bone].z;
    }

    if (!cutReset && prevFrame && prevFrame.localQuats && prevFrame.localQuats[bone]) {
      // Brazos: filtro adaptativo. Movimiento pequeño (ruido/temblor) -> casi se queda quieto;
      // movimiento grande (baile real) -> sigue rápido. Controlado por config.armStability (0..1).
      const armStab = THREE.MathUtils.clamp(config.armStability ?? 0.6, 0, 1);
      const isArm = ARM_SET.has(bone) && armStab > 0;
      const filteredQ = isArm
        ? localQuats[bone]
        : applyQuatDeadzone(localQuats[bone], prevFrame.localQuats[bone], 0.008);
      const prevQ = sanitizeQuat(new THREE.Quaternion(...prevFrame.localQuats[bone]));
      if (prevQ.dot(filteredQ) < 0) {
        filteredQ.set(-filteredQ.x, -filteredQ.y, -filteredQ.z, -filteredQ.w);
      }
      let follow = 1 - (config.smoothing ?? 0.2);
      if (isArm) {
        const ang = 2 * Math.acos(Math.min(1, Math.abs(prevQ.dot(filteredQ))));
        const t = THREE.MathUtils.smoothstep(ang, 0.04, 0.22); // ~2° .. ~13° por frame
        const slowFollow = THREE.MathUtils.lerp(follow, 0.12, armStab);
        const fastFollow = Math.max(follow, 0.55 + 0.35 * (1 - armStab));
        follow = THREE.MathUtils.lerp(slowFollow, fastFollow, t);
      }
      prevQ.slerp(filteredQ, follow);
      localQuats[bone].copy(sanitizeQuat(prevQ));
    }
  }

  const finalHipsPos = [...hipsData.pos];
  if (!cutReset && prevFrame && prevFrame.hipsPos) {
    for (let i = 0; i < 3; i++) {
      finalHipsPos[i] = prevFrame.hipsPos[i] * config.smoothing + finalHipsPos[i] * (1 - config.smoothing);
    }
  }

  const serializedLocal = {};
  for (const bone of BONE_LIST) {
    serializedLocal[bone] = localQuats[bone].toArray();
  }

  const expressions = solveFaceExpressions(
    screenLandmarks,
    pLWr,
    pRWr,
    pLSh,
    pRSh,
    finalHipsPos,
    prevFrame,
    config
  );

  return {
    hipsPos: finalHipsPos,
    localQuats: serializedLocal,
    worldQuats: serializedWorld,
    torsoYaw: reliableTorsoYaw,
    headYaw: headData.headYaw,
    armMeta,
    baseHipY: hipsData.baseHipY,
    baseFootY: hipsData.baseFootY,
    originX: hipsData.originX,
    torsoLenSm: hipsData.torsoLenSm,
    baseTorsoLen: hipsData.baseTorsoLen,
    expressions
  };
}
