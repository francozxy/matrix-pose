// src/components/PosingPanel.jsx
import React, { useState, useEffect } from 'react';

const FULL_EXPRESSION_CATALOG = [
  { id: 'happy', label: '😊 Feliz (Happy)', group: 'Emociones' },
  { id: 'angry', label: '😠 Enojado (Angry)', group: 'Emociones' },
  { id: 'sad', label: '😢 Triste (Sad)', group: 'Emociones' },
  { id: 'relaxed', label: '😌 Relajado (Relaxed)', group: 'Emociones' },
  { id: 'Surprised', label: '😲 Sorprendido (Surprised)', group: 'Emociones' },
  { id: 'neutral', label: '😐 Neutral', group: 'Emociones' },
  { id: 'aa', label: '🗣️ Boca: A', group: 'Boca / Habla' },
  { id: 'ih', label: '🗣️ Boca: I', group: 'Boca / Habla' },
  { id: 'ou', label: '🗣️ Boca: U', group: 'Boca / Habla' },
  { id: 'ee', label: '🗣️ Boca: E', group: 'Boca / Habla' },
  { id: 'oh', label: '🗣️ Boca: O', group: 'Boca / Habla' },
  { id: 'blink', label: '😑 Cerrar Ambos Ojos', group: 'Ojos' },
  { id: 'blinkLeft', label: '😉 Guiño Ojo Izquierdo', group: 'Ojos' },
  { id: 'blinkRight', label: '😉 Guiño Ojo Derecho', group: 'Ojos' },
  { id: 'lookUp', label: '🙄 Mirar Arriba', group: 'Ojos' },
  { id: 'lookDown', label: '😒 Mirar Abajo', group: 'Ojos' },
  { id: 'lookLeft', label: '👈 Mirar Izquierda', group: 'Ojos' },
  { id: 'lookRight', label: '👉 Mirar Derecha', group: 'Ojos' },
  { id: 'joy', label: '😆 Alegría Extrema (Joy)', group: 'Extras' },
  { id: 'sorrow', label: '😭 Llanto / Dolor (Sorrow)', group: 'Extras' },
  { id: 'fun', label: '😸 Sonrisa Pícara (Fun)', group: 'Extras' },
  { id: 'smug', label: '😏 Presumido (Smug)', group: 'Extras' },
  { id: 'thinking', label: '🤔 Pensativo', group: 'Extras' },
  { id: 'confused', label: '😵 Confundido / Mareado', group: 'Extras' }
];

export default function PosingPanel({
  vrmList = [],
  activeVrmIndex = 0,
  selectedBone = '',
  setSelectedBone,
  showBoneNodes,
  setShowBoneNodes,
  onBoneOffsetChange,
  config,
  setConfig
}) {
  const [selectedExpId, setSelectedExpId] = useState('happy');
  const [expressionValues, setExpressionValues] = useState({});
  const [availableList, setAvailableList] = useState(FULL_EXPRESSION_CATALOG);

  useEffect(() => {
    const currentVrm = vrmList[activeVrmIndex]?.vrm || vrmList[activeVrmIndex];
    if (currentVrm) {
      const vrmExpNames = currentVrm.expressionManager?.expressions?.map(e => e.expressionName) || [];
      if (vrmExpNames.length > 0) {
        const unified = FULL_EXPRESSION_CATALOG.map(cat => ({ ...cat }));
        vrmExpNames.forEach(rawName => {
          if (!unified.some(u => u.id.toLowerCase() === rawName.toLowerCase())) {
            unified.push({ id: rawName, label: `✨ ${rawName}`, group: 'Avatar Custom' });
          }
        });
        setAvailableList(unified);
      }
    }
  }, [vrmList, activeVrmIndex]);

  const applyExpressionValue = (expId, rawVal) => {
    const val = Math.max(0, Math.min(1, parseFloat(rawVal) || 0));
    setExpressionValues(prev => ({ ...prev, [expId]: val }));

    const currentVrm = vrmList[activeVrmIndex]?.vrm || vrmList[activeVrmIndex];
    if (currentVrm) {
      if (currentVrm.expressionManager) {
        currentVrm.expressionManager.setValue(expId, val);
      } else if (currentVrm.blendShapeProxy) {
        currentVrm.blendShapeProxy.setValue(expId, val);
      }
    }
  };

  const handleResetAllExpressions = () => {
    const currentVrm = vrmList[activeVrmIndex]?.vrm || vrmList[activeVrmIndex];
    if (currentVrm) {
      availableList.forEach(exp => {
        if (currentVrm.expressionManager) {
          currentVrm.expressionManager.setValue(exp.id, 0);
        } else if (currentVrm.blendShapeProxy) {
          currentVrm.blendShapeProxy.setValue(exp.id, 0);
        }
      });
    }
    setExpressionValues({});
  };

  const currentAvatar = vrmList[activeVrmIndex];
  const activeOffset = (currentAvatar?.offsets && selectedBone)
    ? (currentAvatar.offsets[selectedBone] || { rx: 0, ry: 0, rz: 0 })
    : { rx: 0, ry: 0, rz: 0 };

  const handleRotationInput = (axis, value) => {
    if (!selectedBone || !onBoneOffsetChange) return;
    const num = parseFloat(value) || 0;
    onBoneOffsetChange(selectedBone, {
      ...activeOffset,
      [axis]: num
    });
  };

  const activeExpData = availableList.find(e => e.id === selectedExpId) || availableList[0];
  const currentExpVal = expressionValues[selectedExpId] ?? 0;

  // Gestos rápidos de manos (Puño / Abierta)
  const handleApplyHandPreset = (presetType) => {
    if (!onBoneOffsetChange) return;
    const fingerBones = [
      'leftThumbMetacarpal', 'leftThumbProximal', 'leftThumbDistal',
      'leftIndexProximal', 'leftIndexIntermediate', 'leftIndexDistal',
      'leftMiddleProximal', 'leftMiddleIntermediate', 'leftMiddleDistal',
      'leftRingProximal', 'leftRingIntermediate', 'leftRingDistal',
      'leftLittleProximal', 'leftLittleIntermediate', 'leftLittleDistal'
    ];

    if (presetType === 'fist') {
      fingerBones.forEach(boneName => {
        onBoneOffsetChange(boneName, { rx: -20, ry: 10, rz: 45 });
      });
    } else {
      fingerBones.forEach(boneName => {
        onBoneOffsetChange(boneName, { rx: 0, ry: 0, rz: 0 });
      });
    }
  };

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
      
      {/* 🦴 Ajuste Físico de Articulaciones y Ejes Z */}
      <div className="mp-card">
        <strong style={{ fontSize: '14px', color: '#38bdf8' }}>🦴 Posing & Calibración Z</strong>

        <div style={{ marginTop: '10px' }}>
          <label style={{ fontSize: '12px', color: '#cbd5e1' }}>Seleccionar Hueso a Ajustar:</label>
          <select 
            value={selectedBone} 
            onChange={(e) => setSelectedBone(e.target.value)} 
            style={{ width: '100%', marginTop: '4px', padding: '8px', background: '#262b40', color: '#fff', border: '1px solid #38bdf8', borderRadius: '6px' }}
          >
            <option value="">-- Ningún hueso seleccionado --</option>
            <option value="hips">Caderas (Hips)</option>
            <option value="spine">Columna (Spine)</option>
            <option value="chest">Pecho (Chest)</option>
            <option value="neck">Cuello (Neck)</option>
            <option value="head">Cabeza (Head)</option>
            <option value="leftUpperArm">Brazo Izquierdo</option>
            <option value="leftLowerArm">Antebrazo Izquierdo</option>
            <option value="rightUpperArm">Brazo Derecho</option>
            <option value="rightLowerArm">Antebrazo Derecho</option>
          </select>
        </div>

        {selectedBone && (
          <div style={{ background: '#13141f', padding: '10px', borderRadius: '8px', marginTop: '10px', border: '1px solid #282a36' }}>
            <div style={{ fontSize: '12px', color: '#7aa2f7', marginBottom: '6px', fontWeight: 'bold' }}>
              📐 Rotación Manual de {selectedBone} (°):
            </div>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '6px' }}>
              <div>
                <span style={{ fontSize: '10px', color: '#f7768e' }}>Rot X:</span>
                <input
                  type="number"
                  step="1"
                  value={Math.round(activeOffset.rx || 0)}
                  onChange={(e) => handleRotationInput('rx', e.target.value)}
                  style={{ width: '100%', padding: '4px', fontSize: '12px', background: '#1c2030', color: '#fff', border: '1px solid #2e344d', borderRadius: '4px' }}
                />
              </div>
              <div>
                <span style={{ fontSize: '10px', color: '#9ece6a' }}>Rot Y:</span>
                <input
                  type="number"
                  step="1"
                  value={Math.round(activeOffset.ry || 0)}
                  onChange={(e) => handleRotationInput('ry', e.target.value)}
                  style={{ width: '100%', padding: '4px', fontSize: '12px', background: '#1c2030', color: '#fff', border: '1px solid #2e344d', borderRadius: '4px' }}
                />
              </div>
              <div>
                <span style={{ fontSize: '10px', color: '#7aa2f7' }}>Rot Z:</span>
                <input
                  type="number"
                  step="1"
                  value={Math.round(activeOffset.rz || 0)}
                  onChange={(e) => handleRotationInput('rz', e.target.value)}
                  style={{ width: '100%', padding: '4px', fontSize: '12px', background: '#1c2030', color: '#fff', border: '1px solid #2e344d', borderRadius: '4px' }}
                />
              </div>
            </div>
          </div>
        )}

        <div style={{ display: 'flex', gap: '8px', marginTop: '10px' }}>
          <button
            className="mp-btn"
            style={{ background: '#2563eb', padding: '8px' }}
            onClick={() => handleApplyHandPreset('open')}
          >
            🖐️ Manos Abiertas
          </button>
          <button
            className="mp-btn"
            style={{ background: '#d97706', padding: '8px' }}
            onClick={() => handleApplyHandPreset('fist')}
          >
            ✊ Manos en Puño
          </button>
        </div>

        {/* Checkboxes de control Z en vivo */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: '6px', marginTop: '12px', fontSize: '13px' }}>
          <label style={{ display: 'flex', alignItems: 'center', gap: '8px', cursor: 'pointer' }}>
            <input
              type="checkbox"
              checked={config?.invertArmZ || false}
              onChange={(e) => setConfig({ ...config, invertArmZ: e.target.checked })}
            />
            <span>Invertir Eje Z de Brazos (Frente / Espalda)</span>
          </label>

          <label style={{ display: 'flex', alignItems: 'center', gap: '8px', cursor: 'pointer' }}>
            <input
              type="checkbox"
              checked={config?.invertShoulderZ || false}
              onChange={(e) => setConfig({ ...config, invertShoulderZ: e.target.checked })}
            />
            <span>Invertir Eje Z de Hombros</span>
          </label>

          <label style={{ display: 'flex', alignItems: 'center', gap: '8px', cursor: 'pointer' }}>
            <input
              type="checkbox"
              checked={showBoneNodes}
              onChange={(e) => setShowBoneNodes(e.target.checked)}
            />
            <span>🟢 Ver Puntos Esqueléticos 3D</span>
          </label>
        </div>
      </div>

      {/* 🎭 Expresiones Faciales */}
      <div className="mp-card">
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
          <strong style={{ fontSize: '14px', color: '#38bdf8' }}>🎭 Expresiones Faciales</strong>
          <button
            onClick={handleResetAllExpressions}
            style={{
              background: '#262b40',
              border: '1px solid #e11d48',
              color: '#e11d48',
              borderRadius: '6px',
              padding: '4px 8px',
              fontSize: '11px',
              cursor: 'pointer'
            }}
          >
            Limpiar Carita
          </button>
        </div>

        <select
          value={selectedExpId}
          onChange={(e) => setSelectedExpId(e.target.value)}
          style={{ width: '100%', padding: '8px', background: '#262b40', color: '#fff', border: '1px solid #38bdf8', borderRadius: '6px', marginBottom: '10px' }}
        >
          {['Emociones', 'Boca / Habla', 'Ojos', 'Extras', 'Avatar Custom'].map(grp => {
            const items = availableList.filter(e => e.group === grp);
            if (items.length === 0) return null;
            return (
              <optgroup key={grp} label={grp}>
                {items.map(e => (
                  <option key={e.id} value={e.id}>
                    {e.label} {expressionValues[e.id] > 0 ? `(${(expressionValues[e.id] * 100).toFixed(0)}%)` : ''}
                  </option>
                ))}
              </optgroup>
            );
          })}
        </select>

        <div className="mp-slider-group">
          <div className="mp-slider-head">
            <span>{activeExpData.label}</span>
            <span>{Math.round(currentExpVal * 100)}%</span>
          </div>
          <input
            type="range"
            min="0"
            max="1"
            step="0.01"
            value={currentExpVal}
            onChange={(e) => applyExpressionValue(selectedExpId, e.target.value)}
          />
        </div>
      </div>

    </div>
  );
}
