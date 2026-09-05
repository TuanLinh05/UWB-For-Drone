import { useState, useRef, useCallback, useEffect, useMemo } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';
import { Upload, Sliders, Copy, Check, AlertTriangle } from 'lucide-react';
import { runFilterPipeline, DEFAULT_GATE } from '../lib/scalarRangeFilter';
import type { OutlierGateParams } from '../lib/scalarRangeFilter';
import { parseReplayCsv } from '../lib/replayDataset';
import { SUPPORTED_ANCHOR_IDS, type RangeSample } from '../lib/types';

interface Props {
  rangeHistory: RangeSample[];
}

const FRESH_SAMPLE_MAX_AGE_MS = 200;

interface FilterLabSample {
  time: number;
  rawMm: number;
  firmwareFiltMm: number | null;
  fppDbm: number | null;
}
interface PlotPoint { time: number; raw: number; firmware: number | null; lab: number; }

/* Session samples use wall-clock time, while replay uses uint32_t MCU time.
 * Preserve normal wall-clock deltas and make a one-wrap replay readable. */
function elapsedForPlotMs(nowMs: number, startMs: number): number {
  const direct = nowMs - startMs;
  if (direct >= 0 && direct < 0x8000_0000) return direct;
  return ((nowMs >>> 0) - (startMs >>> 0)) >>> 0;
}

export function FilterTuningLab({ rangeHistory }: Props) {
  const [selectedAnchorId, setSelectedAnchorId] = useState(1);
  const [q, setQ] = useState(0.05);
  const [jumpUp, setJumpUp] = useState(DEFAULT_GATE.jumpUpMm);
  const [jumpDown, setJumpDown] = useState(Math.abs(DEFAULT_GATE.jumpDownMm));
  const [snapAfter, setSnapAfter] = useState(DEFAULT_GATE.snapAfter);
  const [samples, setSamples] = useState<FilterLabSample[]>([]);
  const [hasMissingFpp, setHasMissingFpp] = useState(false);
  const [copied, setCopied] = useState(false);

  const fileInputRef = useRef<HTMLInputElement>(null);

  const anchorIds = useMemo(() => {
    const ids = new Set<number>(SUPPORTED_ANCHOR_IDS);
    for (const sample of rangeHistory) {
      for (const id of Object.keys(sample.anchorsById)) ids.add(Number(id));
    }
    return [...ids].sort((a, b) => a - b);
  }, [rangeHistory]);

  useEffect(() => {
    if (!anchorIds.includes(selectedAnchorId)) setSelectedAnchorId(anchorIds[0]);
  }, [anchorIds, selectedAnchorId]);

  // Use current session data
  const loadFromSession = useCallback(() => {
    if (rangeHistory.length === 0) {
      alert('No data in current session. Connect and receive some data first.');
      return;
    }
    const data: FilterLabSample[] = [];
    for (const sample of rangeHistory) {
      const anchor = sample.anchorsById[selectedAnchorId];
      if (!anchor?.valid || anchor.ageMs > FRESH_SAMPLE_MAX_AGE_MS) continue;
      if (anchor.rawMm === null || anchor.filtMm === null) continue;
      if (!Number.isFinite(anchor.rawMm) || !Number.isFinite(anchor.filtMm)) continue;
      data.push({
        time: sample.clientTime,
        rawMm: anchor.rawMm,
        firmwareFiltMm: anchor.filtMm,
        fppDbm: anchor.fppDbm !== null && Number.isFinite(anchor.fppDbm) ? anchor.fppDbm : null,
      });
    }
    if (data.length === 0) {
      alert(`No fresh, valid range samples are available for anchor ${selectedAnchorId}.`);
      return;
    }
    setSamples(data);
    setHasMissingFpp(data.some(sample => sample.fppDbm === null));
  }, [rangeHistory, selectedAnchorId]);

  // Replay CSV v2 contains quoted Anchor_Layout_JSON.  Do not split rows on
  // commas here: that silently shifts range/FPP columns and invalidates A/B.
  const handleFileLoad = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (ev) => {
      const text = ev.target?.result as string;
      const parsed = parseReplayCsv(text);
      if (!parsed.ok) {
        alert(`Replay CSV is invalid:\n${parsed.errors.slice(0, 3).join('\n')}`);
        return;
      }
      if (!parsed.dataset.anchorIds.includes(selectedAnchorId)) {
        alert(`Replay CSV does not contain anchor A${selectedAnchorId}.`);
        return;
      }

      const data: FilterLabSample[] = [];
      for (const replayFrame of parsed.dataset.frames) {
        const record = replayFrame.records.find(candidate => candidate.id === selectedAnchorId);
        if (!record || !record.valid || record.calibrationMissing) continue;
        if (record.ageMs > FRESH_SAMPLE_MAX_AGE_MS) continue;
        const rawMm = record.correctedRawMm;
        const firmwareFiltMm = record.filteredMm;
        const fppDbm = record.fppDbm;
        if (!Number.isFinite(rawMm) || (rawMm as number) <= 0) continue;
        if (firmwareFiltMm !== null && (!Number.isFinite(firmwareFiltMm) || firmwareFiltMm <= 0)) continue;
        if (fppDbm !== null && !Number.isFinite(fppDbm)) continue;
        data.push({
          time: record.measurementTimeMs ?? replayFrame.timeMs,
          rawMm: rawMm as number,
          firmwareFiltMm,
          fppDbm,
        });
      }
      if (data.length === 0) {
        alert(`CSV has no fresh, valid range rows for anchor ${selectedAnchorId}.`);
        return;
      }
      setSamples(data);
      setHasMissingFpp(data.some(sample => sample.fppDbm === null));
    };
    reader.readAsText(file);
    e.target.value = '';
  }, [selectedAnchorId]);

  // Run filter pipeline with current slider values
  const plotData = useMemo((): PlotPoint[] => {
    if (samples.length === 0) return [];
    const gate: OutlierGateParams = { jumpUpMm: jumpUp, jumpDownMm: -jumpDown, snapAfter };
    const labFiltered = runFilterPipeline(
      samples.map(s => ({ rawMm: s.rawMm, fppDbm: s.fppDbm ?? -70 })),
      q, gate
    );
    // Downsample for chart if needed
    const step = Math.max(1, Math.floor(samples.length / 500));
    return samples
      .filter((_, i) => i % step === 0)
      .map((s, i) => ({
        time: Math.round(elapsedForPlotMs(s.time, samples[0].time) / 1000),
        raw: s.rawMm,
        firmware: s.firmwareFiltMm === null ? null : Math.round(s.firmwareFiltMm),
        lab: Math.round(labFiltered[i * step]),
      }));
  }, [samples, q, jumpUp, jumpDown, snapAfter]);

  const copySnippet = () => {
    const snippet = `s_kf[anchor_idx].Q = ${q.toFixed(4)}f;  /* jump_up=${jumpUp}mm  jump_down=${-jumpDown}mm  snap=${snapAfter} */`;
    navigator.clipboard.writeText(snippet).then(() => {
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    });
  };

  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100%', padding: '1rem', gap: '0.75rem', overflow: 'auto' }}>

      {/* Controls row */}
      <div style={{ display: 'flex', gap: '0.75rem', flexWrap: 'wrap', alignItems: 'flex-start' }}>

        {/* Anchor & load */}
        <div className="glass-panel" style={{ padding: '0.75rem', minWidth: 200 }}>
          <div style={{ fontWeight: 600, fontSize: '0.85rem', marginBottom: '0.5rem', color: 'var(--accent-blue)' }}>Data Source</div>
          <div className="input-group" style={{ marginBottom: '0.5rem' }}>
            <label>Anchor:</label>
            <select value={selectedAnchorId} onChange={e => setSelectedAnchorId(Number(e.target.value))}>
              {anchorIds.map(id => <option key={id} value={id}>A{id}</option>)}
            </select>
          </div>
          <div style={{ display: 'flex', gap: '0.4rem' }}>
            <button className="btn" style={{ fontSize: '0.75rem', padding: '0.3rem 0.6rem', flex: 1 }} onClick={loadFromSession}>
              Use Session
            </button>
            <button className="btn" style={{ fontSize: '0.75rem', padding: '0.3rem 0.6rem', flex: 1, backgroundColor: 'var(--accent-orange)', borderColor: 'var(--accent-orange)' }} onClick={() => fileInputRef.current?.click()}>
              <Upload size={12} /> Load CSV
            </button>
            <input ref={fileInputRef} type="file" accept=".csv" style={{ display: 'none' }} onChange={handleFileLoad} />
          </div>
          {samples.length > 0 && (
            <div style={{ fontSize: '0.7rem', color: 'var(--text-muted)', marginTop: '0.4rem' }}>
              {samples.length} samples loaded
            </div>
          )}
        </div>

        {/* Sliders */}
        <div className="glass-panel" style={{ padding: '0.75rem', flex: 1, minWidth: 280 }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: '0.4rem', marginBottom: '0.5rem' }}>
            <Sliders size={14} color="var(--accent-blue)" />
            <span style={{ fontWeight: 600, fontSize: '0.85rem' }}>Filter Parameters</span>
          </div>
          <SliderRow label="Process Noise Q" value={q} min={0.001} max={2} step={0.001} onChange={setQ} format={v => v.toFixed(3)} />
          <SliderRow label="Jump Up Gate (mm)" value={jumpUp} min={50} max={500} step={10} onChange={setJumpUp} format={v => String(v)} />
          <SliderRow label="Jump Down Gate (mm)" value={jumpDown} min={100} max={1000} step={10} onChange={setJumpDown} format={v => String(v)} />
          <SliderRow label="Snap After (samples)" value={snapAfter} min={5} max={200} step={5} onChange={setSnapAfter} format={v => String(v)} />
        </div>

        {/* Copy snippet */}
        <div className="glass-panel" style={{ padding: '0.75rem', minWidth: 200 }}>
          <div style={{ fontWeight: 600, fontSize: '0.85rem', marginBottom: '0.5rem', color: 'var(--accent-green)' }}>Export to Firmware</div>
          <div style={{ fontFamily: 'monospace', fontSize: '0.7rem', background: 'rgba(0,0,0,0.3)', padding: '0.4rem', borderRadius: 4, marginBottom: '0.5rem', wordBreak: 'break-all', color: 'var(--text-muted)' }}>
            s_kf[idx].Q = {q.toFixed(4)}f;
          </div>
          <button className="btn" style={{ width: '100%', fontSize: '0.78rem', padding: '0.3rem 0.6rem', backgroundColor: 'var(--accent-green)', borderColor: 'var(--accent-green)' }} onClick={copySnippet}>
            {copied ? <><Check size={13} /> Copied!</> : <><Copy size={13} /> Copy C Snippet</>}
          </button>
        </div>
      </div>

      {hasMissingFpp && (
        <div style={{ display: 'flex', gap: '0.5rem', padding: '0.5rem 0.75rem', background: 'rgba(245,158,11,0.1)', border: '1px solid rgba(245,158,11,0.3)', borderRadius: 8, fontSize: '0.78rem', color: 'var(--accent-orange)' }}>
          <AlertTriangle size={14} style={{ flexShrink: 0, marginTop: 2 }} />
          Some replay samples are missing FPP. Using default fpp=-70dBm for the Lab-only comparison; its R-adaptive response will not reflect real signal quality.
        </div>
      )}

      {/* Chart */}
      <div className="glass-panel" style={{ flex: 1, display: 'flex', flexDirection: 'column', minHeight: 300 }}>
        <div style={{ fontWeight: 600, fontSize: '0.9rem', marginBottom: '0.5rem', display: 'flex', gap: '0.5rem', alignItems: 'center' }}>
          <span style={{ color: '#64748b', fontSize: '0.78rem' }}>─ Raw</span>
          <span style={{ color: 'var(--accent-green)', fontSize: '0.78rem' }}>─ Firmware Filtered</span>
          <span style={{ color: 'var(--accent-orange)', fontSize: '0.78rem' }}>─ Lab Filtered (slider)</span>
        </div>
        <div style={{ flex: 1, minHeight: 0 }}>
          {plotData.length === 0 ? (
            <div style={{ height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', color: 'var(--text-muted)', fontSize: '0.85rem' }}>
              Load data to compare filter responses
            </div>
          ) : (
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={plotData} margin={{ top: 10, right: 20, left: 10, bottom: 5 }}>
                <CartesianGrid strokeDasharray="3 3" stroke="rgba(0,0,0,0.1)" />
                <XAxis dataKey="time" type="number" domain={['dataMin', 'dataMax']} tickFormatter={v => `${v}s`} stroke="#94a3b8" />
                <YAxis stroke="#94a3b8" unit=" mm" />
                <Tooltip contentStyle={{ backgroundColor: 'var(--color-surface-1)', border: '1px solid var(--color-border)', borderRadius: 8 }} />
                <Legend />
                <Line type="monotone" dataKey="raw" name="Raw" stroke="#64748b" strokeWidth={1} dot={false} isAnimationActive={false} />
                <Line type="monotone" dataKey="firmware" name="Firmware" stroke="#10b981" strokeWidth={2} dot={false} isAnimationActive={false} />
                <Line type="monotone" dataKey="lab" name="Lab" stroke="#f59e0b" strokeWidth={2} dot={false} isAnimationActive={false} />
              </LineChart>
            </ResponsiveContainer>
          )}
        </div>
      </div>
    </div>
  );
}

function SliderRow({ label, value, min, max, step, onChange, format }: {
  label: string; value: number; min: number; max: number; step: number;
  onChange: (v: number) => void; format: (v: number) => string;
}) {
  return (
    <div style={{ marginBottom: '0.5rem' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.75rem', color: 'var(--text-muted)', marginBottom: 2 }}>
        <span>{label}</span>
        <span style={{ color: 'var(--accent-blue)', fontVariantNumeric: 'tabular-nums' }}>{format(value)}</span>
      </div>
      <input
        type="range" min={min} max={max} step={step} value={value}
        onChange={e => onChange(Number(e.target.value))}
        style={{ width: '100%', accentColor: 'var(--accent-blue)' }}
      />
    </div>
  );
}
