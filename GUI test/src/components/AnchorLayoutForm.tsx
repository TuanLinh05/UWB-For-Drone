import { useState, useEffect } from 'react';
import { MapPin, Save, RotateCcw } from 'lucide-react';
import {
  DEFAULT_ANCHOR_LAYOUT,
  SUPPORTED_ANCHOR_IDS,
  type AnchorPosition,
} from '../lib/types';

const STORAGE_KEY = 'uwb_anchor_layout';

function cloneDefaultLayout(): AnchorPosition[] {
  return DEFAULT_ANCHOR_LAYOUT.map(anchor => ({ ...anchor }));
}

/**
 * Migrates the previous three-anchor localStorage value without discarding the
 * user's coordinates. Missing/invalid entries are filled from the defaults.
 */
export function normalizeAnchorLayout(value: unknown): AnchorPosition[] {
  const candidates = Array.isArray(value) ? value : [];
  return SUPPORTED_ANCHOR_IDS.map(id => {
    const candidate = candidates.find(item => (
      item !== null
      && typeof item === 'object'
      && Number((item as AnchorPosition).id) === id
    )) as Partial<AnchorPosition> | undefined;
    const fallback = DEFAULT_ANCHOR_LAYOUT.find(anchor => anchor.id === id)!;
    const x = Number(candidate?.x);
    const y = Number(candidate?.y);
    return {
      id,
      x: Number.isFinite(x) ? x : fallback.x,
      y: Number.isFinite(y) ? y : fallback.y,
    };
  });
}

export function loadAnchorLayout(): AnchorPosition[] {
  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (stored) return normalizeAnchorLayout(JSON.parse(stored));
  } catch {}
  return cloneDefaultLayout();
}

interface Props {
  layout: AnchorPosition[];
  onSave: (layout: AnchorPosition[]) => void;
}

export function AnchorLayoutForm({ layout, onSave }: Props) {
  const [local, setLocal] = useState<AnchorPosition[]>(layout);
  const [unsaved, setUnsaved] = useState(false);

  useEffect(() => { setLocal(layout); setUnsaved(false); }, [layout]);

  const update = (idx: number, field: 'x' | 'y', val: string) => {
    const n = parseFloat(val);
    if (isNaN(n)) return;
    setLocal(prev => prev.map((a, i) => i === idx ? { ...a, [field]: n } : a));
    setUnsaved(true);
  };

  const save = () => {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(local));
    onSave(local);
    setUnsaved(false);
  };

  const reset = () => {
    const defaults = cloneDefaultLayout();
    setLocal(defaults);
    localStorage.setItem(STORAGE_KEY, JSON.stringify(defaults));
    onSave(defaults);
    setUnsaved(false);
  };

  return (
    <div className="panel panel--compact">
      <div className="panel__header">
        <div className="panel__title">
          <MapPin size={15} color="var(--color-primary)" />
          Anchor Positions (m)
        </div>
        {unsaved && (
          <span style={{ fontSize: '0.75rem', color: 'var(--color-warning)' }}>Unsaved changes</span>
        )}
      </div>

      {local.map((a, i) => (
        <div key={a.id} style={{ display: 'flex', alignItems: 'center', gap: 6, marginBottom: 8 }}>
          <span style={{ width: 24, color: 'var(--color-primary)', fontWeight: 700, fontSize: '0.875rem', flexShrink: 0 }}>
            A{a.id}
          </span>

          {/* P1-09: labels properly linked via htmlFor */}
          <label className="form-label" htmlFor={`anchor-${a.id}-x`} style={{ width: 18 }}>X:</label>
          <input
            id={`anchor-${a.id}-x`}
            aria-label={`Anchor ${a.id} X coordinate in meters`}
            className="form-input"
            type="number" step="0.1" value={a.x}
            onChange={e => update(i, 'x', e.target.value)}
            style={{ width: 64, padding: '0 6px', height: 30 }}
          />

          <label className="form-label" htmlFor={`anchor-${a.id}-y`} style={{ width: 18 }}>Y:</label>
          <input
            id={`anchor-${a.id}-y`}
            aria-label={`Anchor ${a.id} Y coordinate in meters`}
            className="form-input"
            type="number" step="0.1" value={a.y}
            onChange={e => update(i, 'y', e.target.value)}
            style={{ width: 64, padding: '0 6px', height: 30 }}
          />
        </div>
      ))}

      <div style={{ display: 'flex', gap: 8, marginTop: 8 }}>
        <button className="btn btn--primary btn--sm" style={{ flex: 1 }} onClick={save}>
          <Save size={13} /> Save
        </button>
        {/* P1-08: icon-only button must have aria-label */}
        <button
          className="icon-btn"
          style={{ width: 30, height: 30 }}
          onClick={reset}
          aria-label="Reset anchor layout to defaults"
          title="Reset anchor layout to defaults"
        >
          <RotateCcw size={14} />
        </button>
      </div>
    </div>
  );
}
