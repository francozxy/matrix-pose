// src/kinematics.js
import * as THREE from 'three';

export const BONE_LIST = [
  'hips', 'spine', 'chest', 'neck', 'head',
  'leftUpperArm', 'leftLowerArm', 'leftHand',
  'rightUpperArm', 'rightLowerArm', 'rightHand',
  'leftUpperLeg', 'leftLowerLeg', 'leftFoot',
  'rightUpperLeg', 'rightLowerLeg', 'rightFoot'
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
  if (dot > 1.0 - threshold * 0.1) {
    return prevQ;
  }
  return currQ;
}

function computeReliableYaw(shDx, rawShDz, pNose, pLEye, pREye, shCenter, hipCenter, pLKn, pRKn) {
  const eyeCenter = new THREE.Vector3().addVectors(pLEye, pREye).multiplyScalar(0.5);
  const faceLookX = (pNose.x - eyeCenter.x) * 2.2 + (pNose.x - shCenter.x) * 0.8;
  const kneeCenter = new THREE.Vector3().addVectors(pLKn, pRKn).multiplyScalar(0.5);
  const lowerBodyLeadX = (kneeCenter.x - hipCenter.x) * 0.9;

  const shWidth = Math.abs(shDx);
  const profileRatio = THREE.MathUtils.clamp((0.17 - shWidth) / 0.11, 0, 1);

  const dir2D = faceLookX + lowerBodyLeadX * 0.5;
  const sign2D = Math.abs(dir2D) > 0.008 ? Math.sign(dir2D) : 0;

  const zYaw = -Math.atan2(rawShDz * 1.3, Math.max(0.06, shWidth * 1.3));
  const profileYawMag = profileRatio * 1.05;
  const yaw2D = sign2D !== 0 ? sign2D * profileYawMag : dir2D * 6.0;

  if (sign2D !== 0 && Math.abs(dir2D) > 0.018 && Math.sign(zYaw) !== sign2D) {
    return THREE.MathUtils.clamp(yaw2D * 0.85, -1.25, 1.25);
  }

  return THREE.MathUtils.clamp(zYaw * 0.45 + yaw2D * 0.65, -1.3, 1.3);
}

// 1. MÓDULO DE CADERA
function solveHips(pLHip, pRHip, reliableTorsoYaw, screenLandmarks, mirrorSign, prevPos) {
  const midX = (screenLandmarks[23]?.x + screenLandmarks[24]?.x) * 0.5 || 0.5;
  const midY = (screenLandmarks[23]?.y + screenLandmarks[24]?.y) * 0.5 || 0.5;

  const dx = applyDeadzone((midX - 0.5) * mirrorSign * 0.6, prevPos?.[0], 0.003);
  const dy = applyDeadzone((0.50 - midY) * 0.5, prevPos?.[1], 0.003);

  const hipDx = pRHip.x - pLHip.x;
  const hipDy = pRHip.y - pLHip.y;

  const hipRoll = Math.atan2(hipDy, Math.max(0.04, Math.abs(hipDx)));
  const hipYaw = reliableTorsoYaw * 0.65;

  const hipsQuat = new THREE.Quaternion().setFromEuler(
    new THREE.Euler(
      0,
      THREE.MathUtils.clamp(hipYaw, -0.95, 0.95),
      THREE.MathUtils.clamp(hipRoll * 0.75, -0.35, 0.35),
      'YXZ'
    )
  );

  return {
    pos: [
      Number.isFinite(dx) ? THREE.MathUtils.clamp(dx, -0.35, 0.35) : 0,
      Number.isFinite(dy) ? THREE.MathUtils.clamp(dy, -0.25, 0.18) : 0,
      0
    ],
    quat: sanitizeQuat(hipsQuat)
  };
}

// 2. MÓDULO DE DORSO Y PECHO
function solveSpineAndChest(shCenter, hipCenter, pRSh, pLSh, reliableTorsoYaw, hipsQuat) {
  const spineDir = new THREE.Vector3().subVectors(shCenter, hipCenter);
  const spineRoll = -Math.atan2(spineDir.x, Math.max(0.05, Math.abs(spineDir.y)));

  const shDx = pRSh.x - pLSh.x;
  const shDy = pRSh.y - pLSh.y;

  const shRoll = Math.atan2(shDy, Math.max(0.05, Math.abs(shDx)));
  const spinePitch = Math.atan2(spineDir.z, Math.max(0.06, Math.abs(spineDir.y))) * 0.8;

  const chestWorld = sanitizeQuat(
    new THREE.Quaternion().setFromEuler(
      new THREE.Euler(
        spinePitch,
        reliableTorsoYaw,
        THREE.MathUtils.clamp(spineRoll * 0.65 + shRoll * 0.45, -0.45, 0.45),
        'YXZ'
      )
    )
  );

  return {
    spine: sanitizeQuat(hipsQuat.clone().slerp(chestWorld, 0.5)),
    chest: chestWorld
  };
}

// 3. MÓDULO DE CABEZA
function solveHead(pNose, pLEye, pREye, chestQuat, config) {
  const headGain = (config.headSensitivity ?? 1.0) * 0.6;
  const eyeDx = pREye.x - pLEye.x;
  const eyeDy = pREye.y - pLEye.y;
  const eyeCenter = new THREE.Vector3().addVectors(pLEye, pREye).multiplyScalar(0.5);

  const eyeRollAngle = Math.atan2(eyeDy, Math.max(0.02, Math.abs(eyeDx)));
  const localRoll = THREE.MathUtils.clamp(eyeRollAngle * headGain, -0.3, 0.3);
  const localYaw = THREE.MathUtils.clamp(((pNose.x - eyeCenter.x) * 1.8) * headGain, -0.45, 0.45);
  const localPitch = THREE.MathUtils.clamp((-(pNose.y - eyeCenter.y + 0.02) * 1.4) * headGain, -0.3, 0.3);

  const localHeadOffset = sanitizeQuat(
    new THREE.Quaternion().setFromEuler(new THREE.Euler(localPitch, localYaw, localRoll, 'YXZ'))
  );

  const headWorld = sanitizeQuat(chestQuat.clone().multiply(localHeadOffset));

  return {
    neck: sanitizeQuat(chestQuat.clone().slerp(headWorld, 0.45)),
    head: headWorld
  };
}

// 4. MÓDULO DE BRAZOS CON ESCUDO VOLUMÉTRICO ANTI-COLISIÓN
function solveArms(
  pLSh, pRSh, pLEl, pREl, pLWr, pRWr,
  pLPinky, pRPinky, pLIndex, pRIndex,
  pLHip, pRHip,
  chestQuat, visL, visR, config, prevWorld
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
    isLeft, vis, bias, fallbackLocal, prevUpper, prevLower, prevHand
  ) => {
    const rawUpperVec = new THREE.Vector3().subVectors(pEl, pSh);

    if (vis.el < 0.04 || rawUpperVec.lengthSq() < 0.0001) {
      const uWorld = prevUpper
        ? sanitizeQuat(new THREE.Quaternion(...prevUpper))
        : sanitizeQuat(chestQuat.clone().multiply(fallbackLocal));
      const lWorld = prevLower ? sanitizeQuat(new THREE.Quaternion(...prevLower)) : uWorld.clone();
      const hWorld = prevHand ? sanitizeQuat(new THREE.Quaternion(...prevHand)) : lWorld.clone();
      return { upper: uWorld, lower: lWorld, hand: hWorld };
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
      const effectiveTwist = targetSwivel * bendFactor;

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

    return { upper: upperWorld, lower: lowerWorld, hand: handWorld };
  };

  const leftArm = buildArmWorld(
    pLSh, pLEl, pLWr, pLPinky, pLIndex, pLHip,
    true, visL, leftBias, lFallbackLocal,
    prevWorld?.leftUpperArm, prevWorld?.leftLowerArm, prevWorld?.leftHand
  );
  const rightArm = buildArmWorld(
    pRSh, pREl, pRWr, pRPinky, pRIndex, pRHip,
    false, visR, rightBias, rFallbackLocal,
    prevWorld?.rightUpperArm, prevWorld?.rightLowerArm, prevWorld?.rightHand
  );

  return {
    leftUpperArm: leftArm.upper,
    leftLowerArm: leftArm.lower,
    leftHand: leftArm.hand,
    rightUpperArm: rightArm.upper,
    rightLowerArm: rightArm.lower,
    rightHand: rightArm.hand
  };
}

// 5. MÓDULO DE PIERNAS DEL VIDEO DE 53s (Con signo Z corregido para que la rodilla apunte adelante y el talón atrás)
function solveLegs(pLHip, pRHip, pLKn, pRKn, pLAn, pRAn, config, prevLegs) {
  if (pLKn.y > pLHip.y - 0.03) pLKn.y = pLHip.y - 0.14;
  if (pRKn.y > pRHip.y - 0.03) pRKn.y = pRHip.y - 0.14;

  // Candado anti-cruce suave solo cuando ambos pies están apoyados a nivel similar
  if (Math.abs(pLAn.y - pRAn.y) < 0.06) {
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

  // CORRECCIÓN PUNTUAL SOBRE TU VERSIÓN DE 53s:
  // Antes tenías: pLKn.z = Math.max(0.02, pLKn.z) y pLAn.z = Math.min(pLKn.z - 0.02, pLAn.z)
  // En el espacio VRM de Deelia, -Z es hacia ADELANTE (frente) y +Z es hacia ATRÁS (espalda).
  // Por lo tanto, la rodilla nunca debe irse a +Z (atrás de la cadera) y el tobillo siempre debe
  // estar igual o más hacia +Z (atrás) que la rodilla para que jamás se quiebre como flamenco:
  pLKn.z = Math.min(-0.02, pLKn.z * 0.5);
  pRKn.z = Math.min(-0.02, pRKn.z * 0.5);

  pLAn.z = Math.max(pLKn.z + 0.02, pLAn.z * 0.5);
  pRAn.z = Math.max(pRKn.z + 0.02, pRAn.z * 0.5);

  // Si levanta mucho un pie (como en el segundo 00:02 o 00:13), adelantamos esa rodilla hacia el frente (-Z)
  // y llevamos su tobillo hacia atrás (+Z) de forma proporcional a cuánto subió el pie:
  const lLift = Math.max(0, pLAn.y - pRAn.y);
  const rLift = Math.max(0, pRAn.y - pLAn.y);

  if (lLift > 0.04) {
    pLKn.z -= lLift * 0.45; // Rodilla hacia adelante
    pLAn.z = pLKn.z + lLift * 0.95; // Talón hacia atrás
  }
  if (rLift > 0.04) {
    pRKn.z -= rLift * 0.45;
    pRAn.z = pRKn.z + rLift * 0.95;
  }

  const legSpreadRad = THREE.MathUtils.degToRad(config.legSpreadOffset || 0);
  const lLegBias = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -legSpreadRad);
  const rLegBias = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), legSpreadRad);

  const lUpper = getVecQuat(pLHip, pLKn, T_POSE_DIRS.leftUpperLeg, 1.4).multiply(lLegBias);
  const lLower = getVecQuat(pLKn, pLAn, T_POSE_DIRS.leftLowerLeg, 1.85);
  const rUpper = getVecQuat(pRHip, pRKn, T_POSE_DIRS.rightUpperLeg, 1.4).multiply(rLegBias);
  const rLower = getVecQuat(pRKn, pRAn, T_POSE_DIRS.rightLowerLeg, 1.85);

  return {
    leftUpperLeg: prevLegs?.leftUpperLeg ? prevLegs.leftUpperLeg.clone().slerp(lUpper, 0.7) : lUpper,
    leftLowerLeg: prevLegs?.leftLowerLeg ? prevLegs.leftLowerLeg.clone().slerp(lLower, 0.7) : lLower,
    leftFoot: new THREE.Quaternion(0, 0, 0, 1),
    rightUpperLeg: prevLegs?.rightUpperLeg ? prevLegs.rightUpperLeg.clone().slerp(rUpper, 0.7) : rUpper,
    rightLowerLeg: prevLegs?.rightLowerLeg ? prevLegs.rightLowerLeg.clone().slerp(rLower, 0.7) : rLower,
    rightFoot: new THREE.Quaternion(0, 0, 0, 1)
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
    return prevFrame || { hipsPos: [0, 0, 0], localQuats: {}, worldQuats: {} };
  }

  const mirrorSign = config.mirror ? -1 : 1;

  const L = config.mirror
    ? { sh: 11, el: 13, wr: 15, pinky: 17, index: 19, hip: 23, kn: 25, an: 27, eye: 2 }
    : { sh: 12, el: 14, wr: 16, pinky: 18, index: 20, hip: 24, kn: 26, an: 28, eye: 5 };

  const R = config.mirror
    ? { sh: 12, el: 14, wr: 16, pinky: 18, index: 20, hip: 24, kn: 26, an: 28, eye: 5 }
    : { sh: 11, el: 13, wr: 15, pinky: 17, index: 19, hip: 23, kn: 25, an: 27, eye: 2 };

  const pt = (idx) => hybridVec3(screenLandmarks[idx], worldLandmarks?.[idx], config, videoAspect);
  const vis = (idx) => screenLandmarks[idx]?.visibility ?? 1.0;

  const pLSh = pt(L.sh), pRSh = pt(R.sh);
  const pLEl = pt(L.el), pREl = pt(R.el);
  const pLWr = pt(L.wr), pRWr = pt(R.wr);
  const pLPinky = pt(L.pinky), pRPinky = pt(R.pinky);
  const pLIndex = pt(L.index), pRIndex = pt(R.index);
  const pLHip = pt(L.hip), pRHip = pt(R.hip);
  const pLKn = pt(L.kn), pRKn = pt(R.kn);
  const pLAn = pt(L.an), pRAn = pt(R.an);
  const pNose = pt(0), pLEye = pt(L.eye), pREye = pt(R.eye);

  const rawShDz = getRawDepthDiff(R.sh, L.sh, worldLandmarks, screenLandmarks, config.invertShoulderZ);

  const hipCenter = new THREE.Vector3().addVectors(pLHip, pRHip).multiplyScalar(0.5);
  const shCenter = new THREE.Vector3().addVectors(pLSh, pRSh).multiplyScalar(0.5);

  const reliableTorsoYaw = -computeReliableYaw(
    pRSh.x - pLSh.x,
    rawShDz,
    pNose,
    pLEye,
    pREye,
    shCenter,
    hipCenter,
    pLKn,
    pRKn
  );

  const hipsData = solveHips(pLHip, pRHip, reliableTorsoYaw, screenLandmarks, mirrorSign, prevFrame?.hipsPos);
  const torsoData = solveSpineAndChest(shCenter, hipCenter, pRSh, pLSh, reliableTorsoYaw, hipsData.quat);
  const headData = solveHead(pNose, pLEye, pREye, torsoData.chest, config);
  const armsData = solveArms(
    pLSh, pRSh, pLEl, pREl, pLWr, pRWr,
    pLPinky, pRPinky, pLIndex, pRIndex,
    pLHip, pRHip,
    torsoData.chest,
    { el: vis(L.el), wr: vis(L.wr) },
    { el: vis(R.el), wr: vis(R.wr) },
    config,
    prevFrame?.worldQuats
  );

  const prevLegQuats = prevFrame?.worldQuats ? {
    leftUpperLeg: sanitizeQuat(new THREE.Quaternion(...prevFrame.worldQuats.leftUpperLeg)),
    leftLowerLeg: sanitizeQuat(new THREE.Quaternion(...prevFrame.worldQuats.leftLowerLeg)),
    rightUpperLeg: sanitizeQuat(new THREE.Quaternion(...prevFrame.worldQuats.rightUpperLeg)),
    rightLowerLeg: sanitizeQuat(new THREE.Quaternion(...prevFrame.worldQuats.rightLowerLeg))
  } : null;

  const legsData = solveLegs(pLHip, pRHip, pLKn, pRKn, pLAn, pRAn, config, prevLegQuats);

  const worldQuats = {
    hips: hipsData.quat,
    spine: torsoData.spine,
    chest: torsoData.chest,
    neck: headData.neck,
    head: torsoData.chest ? headData.head : new THREE.Quaternion(),
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

  for (const bone of BONE_LIST) {
    sanitizeQuat(worldQuats[bone]);
    serializedWorld[bone] = worldQuats[bone].toArray();

    const parentName = PARENT_MAP[bone];
    if (!parentName) {
      localQuats[bone] = worldQuats[bone].clone();
    } else {
      const parentInv = worldQuats[parentName].clone().invert();
      localQuats[bone] = sanitizeQuat(parentInv.multiply(worldQuats[bone].clone()));
    }

    if (config.vrm0Mode) {
      localQuats[bone].x = -localQuats[bone].x;
      localQuats[bone].z = -localQuats[bone].z;
    }

    if (prevFrame && prevFrame.localQuats && prevFrame.localQuats[bone]) {
      const filteredQ = applyQuatDeadzone(localQuats[bone], prevFrame.localQuats[bone], 0.008);
      const prevQ = sanitizeQuat(new THREE.Quaternion(...prevFrame.localQuats[bone]));
      prevQ.slerp(filteredQ, 1 - (config.smoothing ?? 0.2));
      localQuats[bone].copy(sanitizeQuat(prevQ));
    }
  }

  const finalHipsPos = [...hipsData.pos];
  if (prevFrame && prevFrame.hipsPos) {
    for (let i = 0; i < 3; i++) {
      finalHipsPos[i] = prevFrame.hipsPos[i] * config.smoothing + finalHipsPos[i] * (1 - config.smoothing);
    }
  }

  const serializedLocal = {};
  for (const bone of BONE_LIST) {
    serializedLocal[bone] = localQuats[bone].toArray();
  }

  return {
    hipsPos: finalHipsPos,
    localQuats: serializedLocal,
    worldQuats: serializedWorld
  };
}
