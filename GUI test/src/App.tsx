import { useState, useMemo, memo, useEffect, useCallback } from 'react';
import {
  LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer, ReferenceLine
} from 'recharts';
import {
  Activity, Usb, AlertCircle, Wifi, WifiOff, Target, Crosshair, BarChart2,
  Download, Signal, Radio, Map, FlaskConical, HeartPulse, Wrench, X, RotateCcw
} from 'lucide-react';
import { useUwbTelemetry } from './hooks/useUwbTelemetry';
import { usePositionEstimate } from './hooks/usePositionEstimate';
import { AnchorLayoutForm, loadAnchorLayout } from './components/AnchorLayoutForm';
import { PositionMap } from './components/PositionMap';
import { FilterTuningLab } from './components/FilterTuningLab';
import { SystemHealthTab } from './components/SystemHealthTab';
import { CalibrationWizard } from './components/CalibrationWizard';
import {
  SUPPORTED_ANCHOR_IDS,
  type ChartPoint,
  type AnchorPosition,
} from './lib/types';
import {
  MEASUREMENT_FRESH_MS,
  deriveAnchorLinkState,
  updateCalibrationMissingIds,
} from './lib/anchorState';

type Tab = 'dashboard' | 'position' | 'filterlab' | 'health' | 'calibration';

const TABS: { id: Tab; label: string; Icon: React.ElementType }[] = [
  { id: 'dashboard',   label: 'Dashboard',    Icon: Activity },
  { id: 'position',    label: 'Position',     Icon: Map },
  { id: 'filterlab',   label: 'Filter Lab',   Icon: FlaskConical },
  { id: 'health',      label: 'System Health',Icon: HeartPulse },
  { id: 'calibration', label: 'Calibration',  Icon: Wrench },
];

// ---- Memoized Chart (does not change API) ----
const MemoizedChart = memo(({ data, referenceDist, targetAnchor, calibrationMissing }: {
  data: ChartPoint[];
  referenceDist: number;
  targetAnchor: string;
  calibrationMissing: boolean;
}) => {
  const rawKey = targetAnchor.toLowerCase() + 'Raw';
  const filterKey = targetAnchor.toLowerCase() + 'Filter';
  return (
    <ResponsiveContainer width="100%" height="100%">
      <LineChart data={data} margin={{ top: 12, right: 24, left: 8, bottom: 4 }}>
        <CartesianGrid strokeDasharray="3 3" stroke="rgba(0,0,0,0.08)" vertical={false} />
        <XAxis dataKey="time" type="number" domain={['dataMin', 'dataMax']}
          tickFormatter={(t) => new Date(t).toLocaleTimeString([], { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' })}
          stroke="var(--color-text-subtle)" tick={{ fontSize: 11, fill: 'var(--color-text-subtle)' }} />
        <YAxis domain={['auto', 'auto']} stroke="var(--color-text-subtle)" unit=" mm"
          tick={{ fontSize: 11, fill: 'var(--color-text-subtle)' }} />
        <Tooltip
          contentStyle={{ backgroundColor: 'var(--color-surface-2)', border: '1px solid var(--color-border)', borderRadius: 8, fontSize: 12 }}
          labelFormatter={(l) => new Date(l).toLocaleTimeString()} />
        <Legend wrapperStyle={{ fontSize: '12px', paddingTop: '8px' }} />
        {referenceDist > 0 && (
          <ReferenceLine y={referenceDist}
            label={{ position: 'insideTopLeft', fill: 'var(--color-warning)', fontSize: 11, value: `Ref ${referenceDist}mm` }}
            stroke="var(--color-warning)" strokeDasharray="4 4" />
        )}
        <Line type="monotone" dataKey={rawKey}
          name={calibrationMissing ? `${targetAnchor} Diagnostic Raw (uncalibrated)` : `${targetAnchor} Raw`}
          stroke="rgba(148,163,184,0.45)" strokeWidth={1.5} dot={false} isAnimationActive={false} />
        <Line type="monotone" dataKey={filterKey} name={`${targetAnchor} Filtered`}
          stroke="var(--color-success)" strokeWidth={2} dot={false} isAnimationActive={false} />
      </LineChart>
    </ResponsiveContainer>
  );
});
MemoizedChart.displayName = 'MemoizedChart';

// ---- Connection Dialog ----
function ConnectionDialog({
  open, onClose, ipAddress, setIpAddress, connect, connectWifi, isConnecting,
}: {
  open: boolean;
  onClose: () => void;
  ipAddress: string;
  setIpAddress: (v: string) => void;
  connect: () => void;
  connectWifi: (ip: string) => void;
  isConnecting: boolean;
}) {
  const [mode, setMode] = useState<'usb' | 'wifi'>('wifi');

  const handleKeyDown = useCallback((e: React.KeyboardEvent) => {
    if (e.key === 'Escape') onClose();
  }, [onClose]);

  if (!open) return null;

  return (
    <div className="dialog-backdrop" role="dialog" aria-modal="true" aria-labelledby="conn-dialog-title"
      onKeyDown={handleKeyDown}>
      <div className="dialog" style={{ maxWidth: 440 }}>
        <div className="dialog__header">
          <h2 className="dialog__title" id="conn-dialog-title">Connect to UWB system</h2>
          <button className="icon-btn" onClick={onClose} aria-label="Close dialog">
            <X size={16} />
          </button>
        </div>
        <div className="dialog__body">
          {/* Mode toggle */}
          <div style={{ display: 'flex', gap: 8 }}>
            <button className={`btn ${mode === 'usb' ? 'btn--primary' : 'btn--secondary'}`}
              onClick={() => setMode('usb')} style={{ flex: 1 }}>
              <Usb size={15} /> USB
            </button>
            <button className={`btn ${mode === 'wifi' ? 'btn--primary' : 'btn--secondary'}`}
              onClick={() => setMode('wifi')} style={{ flex: 1 }}>
              <Wifi size={15} /> WiFi
            </button>
          </div>

          {mode === 'usb' && (
            <div>
              <p style={{ fontSize: '0.875rem', color: 'var(--color-text-muted)', marginBottom: 12 }}>
                Requires Chrome or Edge browser. Serial port at 115200 baud.
              </p>
              <button className="btn btn--primary btn--lg" style={{ width: '100%' }}
                onClick={() => { connect(); onClose(); }} disabled={isConnecting}>
                {isConnecting ? 'Connecting…' : 'Select serial device'}
              </button>
            </div>
          )}

          {mode === 'wifi' && (
            <div style={{ display: 'flex', flexDirection: 'column', gap: 12 }}>
              <div className="form-field">
                <label className="form-label" htmlFor="ip-input">ESP32 IP Address</label>
                <div style={{ display: 'flex', gap: 8, alignItems: 'center' }}>
                  <input id="ip-input" className="form-input" type="text" value={ipAddress}
                    onChange={e => setIpAddress(e.target.value)}
                    placeholder="192.168.1.105"
                    style={{ flex: 1 }} />
                  <span style={{ color: 'var(--color-text-muted)', fontSize: '0.875rem', whiteSpace: 'nowrap' }}>:81</span>
                </div>
                <span className="form-hint">Port 81 is fixed (WebSocket bridge on ESP32)</span>
              </div>
              <button className="btn btn--primary btn--lg" style={{ width: '100%' }}
                onClick={() => { connectWifi(ipAddress); onClose(); }}
                disabled={isConnecting || !ipAddress.trim()}>
                {isConnecting ? 'Connecting…' : 'Connect via WiFi'}
              </button>
            </div>
          )}
        </div>
        <div className="dialog__footer">
          <button className="btn btn--ghost" onClick={onClose}>Cancel</button>
        </div>
      </div>
    </div>
  );
}

// ---- Main App ----
function App() {
  const [activeTab, setActiveTab] = useState<Tab>('dashboard');
  const [connectionDialogOpen, setConnectionDialogOpen] = useState(false);
  const [referenceDist, setReferenceDist] = useState(0);
  const [targetAnchor, setTargetAnchor] = useState('A1');
  const [anchorLayout, setAnchorLayout] = useState<AnchorPosition[]>(loadAnchorLayout);
  const [showRangeCircles, setShowRangeCircles] = useState(true);
  const [showTrilateration, setShowTrilateration] = useState(true);
  const [showKalman, setShowKalman] = useState(true);
  const [showGrid, setShowGrid] = useState(true);
  const [gridSpacing, setGridSpacing] = useState(1);
  const [useRaw, setUseRaw] = useState(false);
  const [calibrationMissingIds, setCalibrationMissingIds] = useState<ReadonlySet<number>>(
    () => new Set<number>(),
  );
  const [ipAddress, setIpAddress] = useState(() => {
    try { return localStorage.getItem('uwb_last_ip') || '192.168.1.105'; }
    catch { return '192.168.1.105'; }
  });

  const {
    isConnected, isConnecting, isDataFresh, dataSessionId, connectionMode, telemetryState, lastPacketAt, error,
    latestRange, latestStats, firmwareInfo,
    dataHistory, rangeHistory,
    connect, connectWifi, disconnect,
    isLogging, startLog, stopLog, exportCsv,
    linkHealth, usbDiagnostics, onRangeSampleRef,
  } = useUwbTelemetry();

  const posEst = usePositionEstimate(anchorLayout, useRaw, firmwareInfo);

  useEffect(() => {
    onRangeSampleRef.current = posEst.feed;
    return () => { onRangeSampleRef.current = null; };
  }, [posEst.feed, onRangeSampleRef]);

  useEffect(() => {
    if (telemetryState !== 'fresh') posEst.reset();
  }, [dataSessionId, posEst.reset, telemetryState]);

  // Calibration-required is a profile property, not a one-packet link state.
  // Keep it latched across radio timeouts and clear it as soon as that anchor
  // produces a calibrated valid sample (or a new session starts).
  useEffect(() => {
    setCalibrationMissingIds(new Set<number>());
  }, [dataSessionId]);

  useEffect(() => {
    if (!latestRange || !isDataFresh) return;

    setCalibrationMissingIds(previous => {
      return updateCalibrationMissingIds(
        previous,
        SUPPORTED_ANCHOR_IDS,
        latestRange.anchorsById,
      );
    });
  }, [isDataFresh, latestRange]);

  // Distances in meters from latest range (for range circles)
  const distancesM = useMemo(() => {
    if (!latestRange) return null;
    const result: Record<number, number> = {};
    for (const anchor of anchorLayout) {
      const sample = latestRange.anchorsById[anchor.id];
      const distanceMm = useRaw ? sample?.rawMm : sample?.filtMm;
      if (sample?.valid && sample.ageMs <= MEASUREMENT_FRESH_MS
        && typeof distanceMm === 'number' && Number.isFinite(distanceMm) && distanceMm > 0) {
        result[anchor.id] = distanceMm / 1000;
      }
    }
    return result;
  }, [anchorLayout, latestRange, useRaw]);

  // Anchor display data — chuẩn hóa theo id, không theo index (P1-03)
  const anchorDisplayData = useMemo(() => {
    // §8.4: anchorsById helper — normalize at boundary, never use index as identity
    const byId = latestRange?.anchorsById ?? {};
    return SUPPORTED_ANCHOR_IDS.map(id => {
      const a = byId[id] ?? null;
      const ageMs = a?.ageMs ?? 65535;
      const isCalibrationMissing = isDataFresh && calibrationMissingIds.has(id);
      const hasCurrentMeasurement = isDataFresh
        && (a?.valid ?? false)
        && ageMs <= MEASUREMENT_FRESH_MS;
      const { isOnline, isStale } = deriveAnchorLinkState(
        isDataFresh,
        ageMs,
        isCalibrationMissing,
      );
      return {
        label: `A${id}`, id,
        // P1-04: null cho missing data, không dùng 0
        raw: hasCurrentMeasurement && a ? a.rawMm : isCalibrationMissing ? a?.diagnosticRawMm ?? null : null,
        filter: hasCurrentMeasurement && a ? a.filtMm : null,
        isOnline,
        isCalibrationMissing,
        isStale,
        hasCurrentMeasurement,
        ageMs,
        fppDbm: hasCurrentMeasurement
          ? a?.fppDbm ?? null
          : isCalibrationMissing
            ? a?.diagnosticFppDbm ?? null
            : null,
      };
    });
  }, [calibrationMissingIds, isDataFresh, latestRange]);

  const targetAnchorId = Number(targetAnchor.slice(1));
  const targetCalibrationMissing = isDataFresh
    && calibrationMissingIds.has(targetAnchorId);

  const isWifi = connectionMode === 'wifi';
  const transportStats = latestStats && isWifi
    ? { crcErrorCount: latestStats.crcErrorCount ?? 0, uartGapCount: latestStats.uartGapCount ?? 0 }
    : { crcErrorCount: usbDiagnostics.crcErrors, uartGapCount: 0 };

  // Stats — null khi chưa có dữ liệu (P1-04)
  const stats = useMemo(() => {
    if (dataHistory.length === 0) return { meanError: null, maxError: null, successRate: null };
    if (referenceDist <= 0) return { meanError: null, maxError: null, successRate: null };
    const recent = dataHistory.slice(-60);
    const filterKey = (targetAnchor.toLowerCase() + 'Filter') as keyof ChartPoint;
    let sum = 0, max = 0, valid = 0;
    recent.forEach(d => {
      const v = d[filterKey];
      if (typeof v === 'number' && v > 0) { const e = Math.abs(v - referenceDist); sum += e; if (e > max) max = e; valid++; }
    });
    return {
      meanError: valid > 0 ? Math.round(sum / valid) : null,
      maxError: max > 0 ? Math.round(max) : null,
      successRate: null, // requires counter delta — P1-02, will implement Phase 3
    };
  }, [dataHistory, referenceDist, targetAnchor]);

  const packetAgeMs = lastPacketAt === null ? null : Math.max(0, Date.now() - lastPacketAt);
  const connectionLabel = !isConnected
    ? (isConnecting ? 'Connecting...' : 'Disconnected')
    : telemetryState === 'fresh'
      ? `${connectionMode === 'usb' ? 'USB' : 'WiFi'} Live`
      : telemetryState === 'waiting'
        ? `${connectionMode === 'usb' ? 'USB' : 'WiFi'} Waiting for data`
        : `${connectionMode === 'usb' ? 'USB' : 'WiFi'} Telemetry stale${packetAgeMs === null ? '' : ` (${packetAgeMs} ms)`}`;

  const formatVal = (v: number | null, unit = '', decimals = 0) =>
    v === null ? '—' : (decimals > 0 ? v.toFixed(decimals) : String(v)) + (unit ? ` ${unit}` : '');

  return (
    <div className="app-shell">
      {/* ===== HEADER ===== */}
      <header className="app-header">
        <span className="app-header__brand">
          <Activity size={18} color="var(--color-primary)" />
          UWB Ground Control
        </span>

        <div className="app-header__spacer" />

        {/* Link quality warning badge */}
        {isConnected && (linkHealth.rangeGapCount > 0 || transportStats.crcErrorCount > 0) && (
          <div className="status-badge" style={{ borderColor: 'rgba(245,158,11,0.4)', color: 'var(--color-warning)' }}>
            <Signal size={13} />
            {linkHealth.rangeGapCount > 0 && <span>Gap: {linkHealth.rangeGapCount}</span>}
            {transportStats.crcErrorCount > 0 && <span>CRC: {transportStats.crcErrorCount}</span>}
          </div>
        )}

        {/* Connection status */}
        <div className="app-header__status" aria-live="polite">
          <div className={`status-dot status-dot--${isDataFresh ? 'connected' : 'disconnected'}`} />
          {connectionLabel}
        </div>

        {/* Connect / Disconnect */}
        {isConnected ? (
          <button className="btn btn--danger" onClick={disconnect}>
            <WifiOff size={15} /> Disconnect
          </button>
        ) : (
          <button className="btn btn--primary" onClick={() => setConnectionDialogOpen(true)} disabled={isConnecting}>
            <Wifi size={15} /> {isConnecting ? 'Connecting...' : 'Connect'}
          </button>
        )}
      </header>

      {/* ===== TAB NAVIGATION ===== */}
      <nav className="tab-nav" role="tablist" aria-label="Main navigation">
        {TABS.map(({ id, label, Icon }) => (
          <button key={id}
            className={`tab-nav__btn${activeTab === id ? ' is-active' : ''}`}
            role="tab"
            aria-selected={activeTab === id}
            aria-controls={`${id}-panel`}
            onClick={() => setActiveTab(id)}>
            <Icon size={14} /> {label}
          </button>
        ))}
      </nav>

      {/* ===== MAIN CONTENT ===== */}
      <main className="app-main">

        {/* Error banner */}
        {error && (
          <div style={{
            display: 'flex', gap: 8, alignItems: 'center', padding: '10px 14px',
            background: 'rgba(239,68,68,0.1)', border: '1px solid rgba(239,68,68,0.3)',
            borderRadius: 8, marginBottom: 12, fontSize: '0.875rem', color: '#fca5a5',
          }}>
            <AlertCircle size={16} />
            <span>{error}</span>
          </div>
        )}

        {/* ---- DASHBOARD ---- */}
        {activeTab === 'dashboard' && (
          <div id="dashboard-panel" role="tabpanel" className="dashboard-layout">
            {/* Toolbar */}
            <div style={{ display: 'flex', gap: 12, alignItems: 'center', flexWrap: 'wrap' }}>
              <div className="input-group">
                <label className="form-label" htmlFor="target-anchor-sel" style={{ whiteSpace: 'nowrap' }}>Anchor:</label>
                <select id="target-anchor-sel" className="form-select" style={{ width: 120 }}
                  value={targetAnchor} onChange={e => setTargetAnchor(e.target.value)}>
                  {SUPPORTED_ANCHOR_IDS.map(id => (
                    <option key={id} value={`A${id}`}>Anchor {id}</option>
                  ))}
                </select>
              </div>
              <div className="input-group">
                <label className="form-label" htmlFor="ref-dist-inp" style={{ whiteSpace: 'nowrap' }}>Reference:</label>
                <input id="ref-dist-inp" className="form-input" type="number" style={{ width: 120 }}
                  value={referenceDist || ''} onChange={e => setReferenceDist(Number(e.target.value))}
                  placeholder="e.g. 2000" />
                <span className="text-subtle" style={{ fontSize: '0.8125rem' }}>mm</span>
              </div>
              {referenceDist <= 0 && (
                <span className="text-subtle" style={{ fontSize: '0.8125rem' }}>
                  Set a reference distance to calculate error metrics
                </span>
              )}
            </div>

            {/* Grid: sidebar + chart */}
            <div className="dashboard-grid">
              {/* Sidebar */}
              <div style={{ display: 'flex', flexDirection: 'column', gap: 12, overflow: 'auto' }}>
                {/* System status */}
                <div className="panel panel--compact">
                  <div className="panel__header" style={{ marginBottom: 8 }}>
                    <span className="panel__title">System Status</span>
                  </div>
                  <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 8 }}>
                    <div className="metric-card">
                      <span className="metric-card__label">Cycle Freq</span>
                      <span className="metric-card__value">
                        {latestStats ? latestStats.cycHz : '—'}<small>{latestStats ? ' Hz' : ''}</small>
                      </span>
                    </div>
                    <div className="metric-card">
                      <span className="metric-card__label">Ranging Ops</span>
                      <span className="metric-card__value">
                        {latestStats ? latestStats.opsHz : '—'}<small>{latestStats ? ' /s' : ''}</small>
                      </span>
                    </div>
                  </div>
                </div>

                {/* Anchor cards */}
                {anchorDisplayData.map(anchor => (
                  <div key={anchor.id}
                    className={`anchor-card${targetAnchor === anchor.label ? ' is-selected' : ''}`}
                    onClick={() => setTargetAnchor(anchor.label)}
                    role="button"
                    tabIndex={0}
                    aria-label={`Select ${anchor.label}`}
                    onKeyDown={e => e.key === 'Enter' && setTargetAnchor(anchor.label)}>
                    <div className="anchor-card__header">
                      <div className="anchor-card__title">
                        <Radio size={15}
                          color={anchor.isOnline
                            ? 'var(--color-success)'
                            : anchor.isCalibrationMissing || anchor.isStale
                              ? 'var(--color-warning)'
                              : 'var(--color-text-subtle)'} />
                        Anchor {anchor.id}
                      </div>
                      {anchor.isCalibrationMissing
                        ? <span style={{ color: 'var(--color-warning)', fontSize: '0.7rem', fontWeight: 600 }}>CALIBRATION REQUIRED</span>
                        : anchor.isOnline
                        ? <span style={{ color: 'var(--color-success)', fontSize: '0.75rem', fontWeight: 600 }}>ONLINE</span>
                        : anchor.isStale
                        ? <span style={{ color: 'var(--color-warning)', fontSize: '0.75rem' }}>STALE</span>
                        : <span style={{ color: 'var(--color-text-subtle)', fontSize: '0.75rem' }}>OFFLINE</span>}
                    </div>
                    <div className="anchor-card__values">
                      <div>
                        <div className="anchor-card__val-label">
                          {anchor.isCalibrationMissing ? 'Diagnostic Raw (mm)' : 'Raw (mm)'}
                        </div>
                        <div className={`anchor-card__val-num${anchor.raw === null ? ' anchor-card__val-num--na' : ' anchor-card__val-num--raw'}`}>
                          {anchor.raw === null ? '—' : anchor.raw}
                        </div>
                      </div>
                      <div>
                        <div className="anchor-card__val-label">Filter (mm)</div>
                        <div className={`anchor-card__val-num${anchor.filter === null ? ' anchor-card__val-num--na' : ' anchor-card__val-num--filter'}`}>
                          {anchor.filter === null ? '—' : anchor.filter}
                        </div>
                      </div>
                    </div>
                    {(anchor.isOnline || anchor.isStale || anchor.isCalibrationMissing) && (
                      <div className="anchor-card__footer">
                        <span>{anchor.fppDbm === null ? 'FPP: —' : `FPP: ${anchor.fppDbm.toFixed(1)} dBm`}</span>
                        <span>
                          {anchor.isCalibrationMissing
                            ? 'Pre-offset · excluded from solver'
                            : `Age: ${anchor.ageMs} ms${anchor.hasCurrentMeasurement ? '' : ' · latest sample lost'}`}
                        </span>
                      </div>
                    )}
                  </div>
                ))}
              </div>

              {/* Chart */}
              <div className="panel" style={{ display: 'flex', flexDirection: 'column', minHeight: 0 }}>
                <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 12, flexWrap: 'wrap', gap: 8 }}>
                  <h2 style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: '0.9375rem', fontWeight: 600, margin: 0 }}>
                    <Activity size={16} color="var(--color-primary)" />
                    Real-time Range — {targetAnchor}
                    {targetCalibrationMissing && (
                      <span style={{ color: 'var(--color-warning)', fontSize: '0.7rem', fontWeight: 600 }}>
                        UNCALIBRATED · DIAGNOSTIC ONLY
                      </span>
                    )}
                  </h2>
                  <div style={{ display: 'flex', gap: 8, alignItems: 'center' }}>
                    {isLogging && <span style={{ fontSize: '0.8125rem', color: 'var(--color-danger)' }}>● Logging</span>}
                    {!isLogging
                      ? <button className="btn btn--sm btn--danger" onClick={() => startLog({
                          anchorLayout,
                          positionRangeSource: useRaw ? 'raw' : 'filtered',
                        })} disabled={!isConnected}>● Start Log</button>
                      : <button className="btn btn--sm btn--secondary" onClick={stopLog}>■ Stop Log</button>}
                    <button className="btn btn--sm btn--secondary" onClick={exportCsv} disabled={dataHistory.length === 0}>
                      <Download size={13} /> Export CSV
                    </button>
                  </div>
                </div>
                <div className="chart-wrapper" style={{ flex: 1, minHeight: 200 }}>
                  {dataHistory.length === 0
                    ? (
                      <div className="empty-state">
                        <Activity size={32} className="empty-state__icon" />
                        <div className="empty-state__title">No data yet</div>
                        <div className="empty-state__body">
                          {isConnected ? 'Waiting for telemetry packets…' : 'Connect a device to start receiving data.'}
                        </div>
                        {!isConnected && (
                          <button className="btn btn--primary" onClick={() => setConnectionDialogOpen(true)}>
                            <Wifi size={15} /> Connect device
                          </button>
                        )}
                      </div>
                    )
                    : <MemoizedChart
                        data={dataHistory}
                        referenceDist={referenceDist}
                        targetAnchor={targetAnchor}
                        calibrationMissing={targetCalibrationMissing}
                      />
                  }
                </div>
              </div>
            </div>

            {/* Bottom KPI row */}
            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: 12 }}>
              <div className="panel panel--compact" style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
                <Target size={20} color="var(--color-primary)" style={{ flexShrink: 0 }} />
                <div className="metric-card" style={{ border: 'none', background: 'transparent', padding: 0 }}>
                  <span className="metric-card__label">Mean Error</span>
                  <span className="metric-card__value">
                    {formatVal(stats.meanError, 'mm')}
                  </span>
                </div>
              </div>
              <div className="panel panel--compact" style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
                <Crosshair size={20} color="var(--color-primary)" style={{ flexShrink: 0 }} />
                <div className="metric-card" style={{ border: 'none', background: 'transparent', padding: 0 }}>
                  <span className="metric-card__label">Max Error</span>
                  <span className="metric-card__value">
                    {formatVal(stats.maxError, 'mm')}
                  </span>
                </div>
              </div>
              <div className="panel panel--compact" style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
                <BarChart2 size={20} color="var(--color-primary)" style={{ flexShrink: 0 }} />
                <div className="metric-card" style={{ border: 'none', background: 'transparent', padding: 0 }}>
                  <span className="metric-card__label">Response Success</span>
                  <span className="metric-card__value">
                    {stats.successRate !== null ? `${stats.successRate}%` : '—'}
                  </span>
                </div>
              </div>
            </div>
          </div>
        )}

        {/* ---- POSITION MAP ---- */}
        {activeTab === 'position' && (
          <div id="position-panel" role="tabpanel" className="position-layout" style={{ height: 'calc(100vh - 120px)' }}>
            {/* Sidebar */}
            <div className="position-sidebar">
              <AnchorLayoutForm layout={anchorLayout} onSave={newLayout => { setAnchorLayout(newLayout); posEst.reset(); }} />
              <div className="panel panel--compact">
                <div className="section-label">Display Options</div>
                {([
                  ['showRangeCircles', 'Range Circles', showRangeCircles, setShowRangeCircles],
                  ['showTrilateration', 'Trilat Trail', showTrilateration, setShowTrilateration],
                  ['showKalman', 'Kalman Trail', showKalman, setShowKalman],
                  ['showGrid', 'Grid', showGrid, setShowGrid],
                ] as const).map(([key, label, val, setter]) => (
                  <label key={key} className="check-row">
                    <input type="checkbox" checked={val} onChange={e => (setter as (v: boolean) => void)(e.target.checked)} />
                    {label}
                  </label>
                ))}
                <div className="input-group" style={{ marginTop: 8 }}>
                  <label className="form-label" htmlFor="grid-size-sel">Grid size:</label>
                  <select id="grid-size-sel" className="form-select" style={{ width: 80 }}
                    value={gridSpacing} onChange={e => setGridSpacing(Number(e.target.value))}>
                    <option value={0.5}>0.5 m</option>
                    <option value={0.6}>0.6 m</option>
                    <option value={1}>1 m</option>
                    <option value={2}>2 m</option>
                    <option value={5}>5 m</option>
                  </select>
                </div>
                <hr className="divider" />
                <label className="check-row">
                  <input type="checkbox" checked={useRaw} onChange={e => setUseRaw(e.target.checked)} />
                  Use Raw (not filtered)
                </label>
                <button className="btn btn--danger" style={{ marginTop: 8, width: '100%' }} onClick={posEst.reset}
                  aria-label="Reset position trail">
                  <RotateCcw size={14} /> Reset Trail
                </button>
              </div>

              {/* Position readout */}
              <div className="panel panel--compact">
                <div className="section-label">Position (m)</div>
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px 12px', fontFamily: 'var(--font-mono)', fontSize: '0.875rem' }}>
                  <span className="text-subtle">T.x</span><span>{posEst.trilatPoint?.x.toFixed(2) ?? '—'}</span>
                  <span className="text-subtle">T.y</span><span>{posEst.trilatPoint?.y.toFixed(2) ?? '—'}</span>
                  <span className="text-subtle">K.x</span><span>{posEst.kalmanPoint?.x.toFixed(2) ?? '—'}</span>
                  <span className="text-subtle">K.y</span><span>{posEst.kalmanPoint?.y.toFixed(2) ?? '—'}</span>
                </div>
                <div style={{ marginTop: 8, fontSize: '0.75rem', color: posEst.quality.status === 'valid' ? 'var(--color-success)' : 'var(--color-text-muted)' }}>
                  {posEst.quality.status === 'valid'
                    ? `Valid · anchors ${posEst.quality.usedAnchorIds.join(', ')} · RMS ${(posEst.quality.residualRmsM ?? 0).toFixed(3)} m`
                    : posEst.quality.status === 'invalid'
                      ? `No solution: ${posEst.quality.reason}`
                      : 'Waiting for a valid 3-anchor geometry'}
                </div>
                <hr className="divider" />
                <div className="section-label">C6/C8 Shadow (not controlling)</div>
                <div style={{
                  display: 'grid',
                  gridTemplateColumns: 'auto 1fr',
                  gap: '2px 8px',
                  fontSize: '0.72rem',
                  color: 'var(--color-text-muted)',
                }}>
                  <span>Output</span>
                  <span>{posEst.shadowQuality.outputMode.toUpperCase()} · {posEst.shadowQuality.trackingMode}</span>
                  <span>Solve</span>
                  <span>{posEst.shadowQuality.result?.mode ?? posEst.shadowQuality.reason ?? 'waiting'}</span>
                  <span>Used</span>
                  <span>[{posEst.shadowQuality.result?.usedAnchorIds.join(', ')
                    ?? posEst.shadowQuality.usableAnchorIds.join(', ')}]</span>
                  <span>Excluded</span>
                  <span>{posEst.shadowQuality.excluded.length > 0
                    ? posEst.shadowQuality.excluded
                      .map(item => `${item.id === null ? '?' : `A${item.id}`}:${item.reason}`)
                      .join(', ')
                    : 'none'}</span>
                  <span>RMS / χ²</span>
                  <span>{posEst.shadowQuality.result
                    ? `${posEst.shadowQuality.result.rmsResidualM.toFixed(3)} m / `
                      + posEst.shadowQuality.result.normalizedChi2.toFixed(2)
                    : '—'}</span>
                  <span>GDOP / cond</span>
                  <span>{posEst.shadowQuality.result
                    ? `${posEst.shadowQuality.result.gdop.toFixed(2)} / `
                      + posEst.shadowQuality.result.conditionNumber.toFixed(1)
                    : '—'}</span>
                  <span>NIS / age</span>
                  <span>{posEst.shadowQuality.nis === null
                    ? '—'
                    : posEst.shadowQuality.nis.toFixed(2)}
                    {' / '}
                    {posEst.shadowQuality.result
                      ? `${posEst.shadowQuality.result.ageSpreadMs} ms`
                      : '—'}</span>
                  <span>Shadow K</span>
                  <span>{posEst.shadowQuality.estimate
                    ? `(${posEst.shadowQuality.estimate.x.toFixed(2)}, `
                      + `${posEst.shadowQuality.estimate.y.toFixed(2)}) m`
                    : '—'}</span>
                  <span>Config</span>
                  <span>{posEst.shadowQuality.configVersion} · {firmwareInfo?.rangingMode.toUpperCase() ?? 'UNKNOWN'}</span>
                </div>
              </div>
            </div>

            {/* Canvas */}
            <div className="position-map-container">
              <PositionMap
                anchorLayout={anchorLayout}
                distancesM={distancesM}
                trilatPoint={posEst.trilatPoint}
                kalmanPoint={posEst.kalmanPoint}
                registerFrameCallback={posEst.registerFrameCallback}
                showRangeCircles={showRangeCircles}
                showTrilateration={showTrilateration}
                showKalman={showKalman}
                showGrid={showGrid}
                gridSpacing={gridSpacing}
              />
            </div>
          </div>
        )}

        {/* ---- FILTER LAB ---- */}
        {activeTab === 'filterlab' && (
          <div id="filterlab-panel" role="tabpanel">
            <FilterTuningLab rangeHistory={rangeHistory} />
          </div>
        )}

        {/* ---- HEALTH ---- */}
        {activeTab === 'health' && (
          <div id="health-panel" role="tabpanel">
            <SystemHealthTab
              latestStats={latestStats}
              rangeHistory={rangeHistory}
              connectionMode={connectionMode}
              crcErrorCount={transportStats.crcErrorCount}
              uartGapCount={transportStats.uartGapCount}
              rangeGapCount={linkHealth.rangeGapCount}
              firmwareInfo={firmwareInfo}
              telemetryState={telemetryState}
            />
          </div>
        )}

        {/* ---- CALIBRATION ---- */}
        {activeTab === 'calibration' && (
          <div id="calibration-panel" role="tabpanel">
            <CalibrationWizard key={dataSessionId} latestRange={latestRange} isConnected={isConnected && isDataFresh} firmwareInfo={firmwareInfo} />
          </div>
        )}
      </main>

      {/* ===== CONNECTION DIALOG ===== */}
      <ConnectionDialog
        open={connectionDialogOpen}
        onClose={() => setConnectionDialogOpen(false)}
        ipAddress={ipAddress}
        setIpAddress={ip => {
          setIpAddress(ip);
          try { localStorage.setItem('uwb_last_ip', ip); } catch { /* private mode */ }
        }}
        connect={connect}
        connectWifi={connectWifi}
        isConnecting={isConnecting}
      />
    </div>
  );
}

export default App;
