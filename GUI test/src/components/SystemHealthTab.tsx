import { useMemo } from 'react';

import { Activity, Wifi, AlertTriangle, Clock, Thermometer, Info } from 'lucide-react';
import {
  SUPPORTED_ANCHOR_IDS,
  type StatsSample,
  type RangeSample,
  type FirmwareInfo,
  type TelemetryState,
} from '../lib/types';
import { computeAnchorUptime } from '../lib/health';
import { PIPELINE_CONFIG_VERSION } from '../lib/replayLog';

const TAG_ST_CALIBRATION_MISSING = 0x20;

interface Props {
  latestStats: StatsSample | null;
  rangeHistory: RangeSample[];
  connectionMode: 'usb' | 'wifi' | 'disconnected';
  crcErrorCount: number;
  uartGapCount: number;
  rangeGapCount: number;
  firmwareInfo: FirmwareInfo | null;
  telemetryState: TelemetryState;
}

/** Returns window duration in seconds from the clientTime of first/last samples in the ring buffer.
 *  Falls back to 0 if fewer than 2 samples. Does NOT assume 50 Hz. */
function sessionWindowSec(rangeHistory: RangeSample[]): number {
  if (rangeHistory.length < 2) return 0;
  const first = rangeHistory[0].clientTime;
  const last  = rangeHistory[rangeHistory.length - 1].clientTime;
  return Math.max(0, (last - first) / 1000);
}

function rangeFilterLabel(mode: number): string {
  if (mode === 1) return 'MEDIAN_GATE (C9 fast candidate)';
  if (mode === 2) return 'CV_KALMAN_V2 (experimental)';
  return 'LEGACY_KALMAN (rollback baseline)';
}

function legacyAdaptiveLabel(mode: FirmwareInfo['legacyAdaptiveMode']): string {
  if (mode === 'shadow') return 'SHADOW (output remains Legacy)';
  if (mode === 'active') return 'ACTIVE (candidate output)';
  if (mode === 'unknown') return 'UNKNOWN (reject for A/B)';
  return 'OFF (deployed baseline)';
}

function c9_2ModeLabel(mode: FirmwareInfo['c9_2MotionMode']): string {
  if (mode === 'shadow') return 'SHADOW (diagnostic only)';
  if (mode === 'active') return 'ACTIVE (candidate output)';
  if (mode === 'unknown') return 'UNKNOWN (schema 1 / not reported)';
  return 'OFF (rollback baseline)';
}

export function SystemHealthTab({ latestStats, rangeHistory, connectionMode, crcErrorCount, uartGapCount, rangeGapCount, firmwareInfo, telemetryState }: Props) {
  const anchorIds = useMemo(() => {
    const ids = new Set<number>(SUPPORTED_ANCHOR_IDS);
    for (const sample of rangeHistory) {
      for (const id of Object.keys(sample.anchorsById)) ids.add(Number(id));
    }
    return [...ids].sort((a, b) => a - b);
  }, [rangeHistory]);
  const uptimesById = useMemo(() => Object.fromEntries(
    anchorIds.map(id => [id, computeAnchorUptime(rangeHistory, id)]),
  ) as Record<number, number>, [anchorIds, rangeHistory]);

  const s = latestStats;
  const isWifi = connectionMode === 'wifi';
  const windowSec = sessionWindowSec(rangeHistory);

  const hasErrors = crcErrorCount > 0 || uartGapCount > 0 || rangeGapCount > 0
    || (s?.wifiRangeDropCount ?? 0) > 0;

  return (
    <div style={{ padding: '1rem', display: 'flex', flexDirection: 'column', gap: '1rem', overflowY: 'auto', height: '100%' }}>
      {/* Section: Link Quality */}
      <div className="glass-panel">
        <div className="anchor-header" style={{ borderBottom: 'none', paddingBottom: 0, marginBottom: '0.75rem' }}>
          <div className="anchor-title"><Wifi size={16} color="var(--accent-blue)" /> Link Quality</div>
          {hasErrors && <span style={{ color: 'var(--accent-orange)', fontSize: '0.75rem' }}>⚠ Errors detected</span>}
        </div>
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: '0.75rem' }}>
          <StatCard label="Telemetry" value={telemetryState.toUpperCase()} warning={telemetryState === 'stale'} unit="" />
          <StatCard label="Range Gap Count" value={rangeHistory.length === 0 ? '—' : rangeGapCount} warning={rangeGapCount > 0} unit="pkts" />
          <StatCard
            label="UART Gap Count"
            value={isWifi ? (rangeHistory.length === 0 ? '—' : uartGapCount) : 'WiFi only'}
            warning={isWifi && uartGapCount > 0}
            unit={isWifi ? 'pkts' : ''}
          />
          <StatCard
            label="CRC Errors"
            value={connectionMode === 'disconnected' ? '—' : crcErrorCount}
            warning={crcErrorCount > 0}
            unit={connectionMode === 'disconnected' ? '' : 'errs'}
          />
        </div>
        {isWifi && (
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, minmax(0, 1fr))', gap: '0.75rem', marginTop: '0.75rem' }}>
            <StatCard
              label="WiFi Range TX"
              value={s?.wifiRangeHz ?? '—'}
              warning={(s?.wifiRangeHz ?? 50) < 45}
              unit="Hz"
            />
            <StatCard
              label="ESP Low-Heap Drops"
              value={s?.wifiRangeDropCount ?? '—'}
              warning={(s?.wifiRangeDropCount ?? 0) > 0}
              unit="frames"
            />
          </div>
        )}
        {isWifi && s?.wifiRangeIntervalMs !== undefined && (
          <p style={{ fontSize: '0.72rem', color: 'var(--text-muted)', marginTop: '0.5rem' }}>
            ESP32 target interval: {s.wifiRangeIntervalMs} ms. Chart history is intentionally decimated; calibration, CSV logging and the position estimator still receive every range frame.
          </p>
        )}
        {!isWifi && connectionMode !== 'disconnected' && (
          <p style={{ fontSize: '0.72rem', color: 'var(--text-muted)', marginTop: '0.5rem' }}>UART source-gap diagnostics are supplied by the ESP32 bridge; USB CRC is checked directly in the GUI parser.</p>
        )}
      </div>

      {/* Section: Ranging Performance */}
      <div className="glass-panel">
        <div className="anchor-header" style={{ borderBottom: 'none', paddingBottom: 0, marginBottom: '0.75rem' }}>
          <div className="anchor-title"><Activity size={16} color="var(--accent-green)" /> Ranging Performance</div>
        </div>
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: '0.75rem' }}>
          <StatCard label="Cycle Freq" value={s?.cycHz ?? '—'} unit="Hz" />
          <StatCard label="Ranging Ops" value={s?.opsHz ?? '—'} unit="/s" />
          <StatCard label="RX Timeout" value={s ? s.rxTimeout : '—'} warning={(s?.rxTimeout ?? 0) > 10} unit="" />
          <StatCard label="RX Error" value={s ? s.rxError : '—'} warning={(s?.rxError ?? 0) > 0} unit="" />
        </div>
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: '0.75rem', marginTop: '0.75rem' }}>
          <StatCard label="Poll Sent" value={s ? s.pollSent : '—'} unit="" />
          <StatCard label="Response OK" value={s ? s.responseOk : '—'} unit="" />
          <StatCard label="Cycle Overrun" value={s ? s.cycleOverrun : '—'} warning={(s?.cycleOverrun ?? 0) > 0} unit="" />
          <StatCard
            label="UART Overflow"
            value={s ? s.uartOverflow : '—'}
            warning={(s?.uartOverflow ?? 0) > 0}
            unit=""
          />
        </div>
      </div>

      {/* Section: Anchor Uptime */}
      <div className="glass-panel">
        <div className="anchor-header" style={{ borderBottom: 'none', paddingBottom: 0, marginBottom: '0.75rem' }}>
          <div className="anchor-title"><Clock size={16} color="var(--accent-orange)" /> Anchor Availability</div>
        </div>
        <div style={{ display: 'grid', gridTemplateColumns: `repeat(${anchorIds.length}, minmax(0, 1fr))`, gap: '0.75rem' }}>
          {anchorIds.map(id => {
            const uptime = uptimesById[id] ?? 0;
            const latestAnchor = rangeHistory.length > 0
              ? rangeHistory[rangeHistory.length - 1].anchorsById[id]
              : undefined;
            const calibrationMissing = latestAnchor?.status !== undefined
              && (latestAnchor.status & TAG_ST_CALIBRATION_MISSING) !== 0;
            return (
            <div key={id} style={{ textAlign: 'center' }}>
              <div style={{ fontSize: '0.75rem', color: 'var(--text-muted)', marginBottom: 4 }}>Anchor {id}</div>
              <div style={{ fontSize: '1.8rem', fontWeight: 700, color: uptime > 80 ? 'var(--accent-green)' : uptime > 50 ? 'var(--accent-orange)' : 'var(--accent-red)' }}>
                {rangeHistory.length === 0 ? '—' : `${uptime.toFixed(1)}%`}
              </div>
              <UptimeBar pct={uptime} />
              {calibrationMissing && (
                <div style={{ marginTop: 5, color: 'var(--accent-orange)', fontSize: '0.68rem', fontWeight: 600 }}>
                  Calibration required
                </div>
              )}
            </div>
            );
          })}
        </div>
        <p style={{ fontSize: '0.72rem', color: 'var(--text-muted)', marginTop: '0.5rem' }}>
          {rangeHistory.length < 2
            ? 'No data received yet.'
            : `Last ${windowSec.toFixed(1)}s window — ${rangeHistory.length} samples.`
          }
        </p>
      </div>

      {/* Section: Placeholders (12b) */}
      <div className="glass-panel" style={{ opacity: 0.6 }}>
        <div className="anchor-header" style={{ borderBottom: 'none', paddingBottom: 0, marginBottom: '0.75rem' }}>
          <div className="anchor-title"><Thermometer size={16} color="var(--text-muted)" /> DW1000 Temperature</div>
          <span style={{ fontSize: '0.7rem', color: 'var(--text-muted)' }}>Unavailable</span>
        </div>
        <PlaceholderNote text="Firmware currently reads temperature once at boot (printf only). Needs periodic telemetry packet to display here." />
      </div>

      <div className="glass-panel" style={{ opacity: firmwareInfo ? 1 : 0.6 }}>
        <div className="anchor-header" style={{ borderBottom: 'none', paddingBottom: 0, marginBottom: '0.75rem' }}>
          <div className="anchor-title"><Info size={16} color="var(--accent-blue)" /> Firmware &amp; Ranging Profile</div>
          <span style={{ fontSize: '0.7rem', color: firmwareInfo ? 'var(--accent-green)' : 'var(--text-muted)' }}>
            {firmwareInfo?.calibrationProfile ?? 'Unavailable'}
          </span>
        </div>
        {firmwareInfo ? (
          <div style={{ display: 'grid', gap: 6, fontSize: '0.78rem' }}>
            <div>Ranging mode: <strong>{firmwareInfo.rangingMode.toUpperCase()}</strong></div>
            <div>PHY: <strong>{firmwareInfo.phyProfile}</strong> · SPI runtime: <strong>{firmwareInfo.spiClockMhz > 0 ? `${firmwareInfo.spiClockMhz} MHz` : 'unknown'}</strong></div>
            <div>Calibration: <strong>{firmwareInfo.calibrationProfile}</strong> · HW antenna delay: <strong>{firmwareInfo.hardwareAntennaDelay ? 'on' : 'off'}</strong></div>
            <div>Range filter: <strong>{rangeFilterLabel(firmwareInfo.rangeFilterMode)}</strong> · DS calibrated mask: <strong>0x{firmwareInfo.dsCalibratedMask.toString(16).padStart(2, '0')}</strong></div>
            <div>Adaptive Legacy: <strong>{legacyAdaptiveLabel(firmwareInfo.legacyAdaptiveMode)}</strong></div>
            <div>C9.2 motion controller: <strong>{c9_2ModeLabel(firmwareInfo.c9_2MotionMode)}</strong> · aggregate: <strong>{firmwareInfo.c9_2MotionMode === 'off' ? 'DISABLED' : firmwareInfo.c9_2GlobalMotionState.toUpperCase()}</strong></div>
            <div>GUI pipeline config: <strong>{PIPELINE_CONFIG_VERSION}</strong> · Range rollback: <strong>set UWB_LEGACY_ADAPTIVE_MODE to OFF</strong></div>
            <div style={{ fontFamily: 'var(--font-mono)' }}>
              {Object.entries(firmwareInfo.activeOffsetsM)
                .sort(([left], [right]) => Number(left) - Number(right))
                .map(([id, value]) => `A${id}: ${value.toFixed(6)} m`)
                .join(' · ')}
            </div>
          </div>
        ) : (
          <PlaceholderNote text="Waiting for the periodic TYPE 0x00 firmware-info packet." />
        )}
      </div>
    </div>
  );
}

function StatCard({ label, value, unit, warning = false }: { label: string; value: number | string; unit: string; warning?: boolean }) {
  return (
    <div style={{ background: 'rgba(0,0,0,0.15)', borderRadius: 8, padding: '0.6rem 0.8rem' }}>
      <div style={{ fontSize: '0.7rem', textTransform: 'uppercase', letterSpacing: '0.05em', color: 'var(--text-muted)', marginBottom: 4 }}>{label}</div>
      <div style={{ fontSize: '1.2rem', fontWeight: 700, color: warning ? 'var(--accent-orange)' : 'var(--text-main)', fontVariantNumeric: 'tabular-nums' }}>
        {value} <span style={{ fontSize: '0.8rem', color: 'var(--text-muted)' }}>{unit}</span>
      </div>
    </div>
  );
}

function UptimeBar({ pct }: { pct: number }) {
  const color = pct > 80 ? 'var(--accent-green)' : pct > 50 ? 'var(--accent-orange)' : 'var(--accent-red)';
  return (
    <div style={{ height: 4, background: 'rgba(255,255,255,0.1)', borderRadius: 2, marginTop: 6, overflow: 'hidden' }}>
      <div style={{ height: '100%', width: `${Math.max(0, Math.min(100, pct))}%`, background: color, borderRadius: 2, transition: 'width 0.3s ease' }} />
    </div>
  );
}

function PlaceholderNote({ text }: { text: string }) {
  return (
    <div style={{ display: 'flex', alignItems: 'flex-start', gap: '0.5rem', padding: '0.5rem', background: 'rgba(0,0,0,0.15)', borderRadius: 6 }}>
      <AlertTriangle size={14} color="var(--accent-orange)" style={{ flexShrink: 0, marginTop: 2 }} />
      <p style={{ fontSize: '0.75rem', color: 'var(--text-muted)', lineHeight: 1.5 }}>{text}</p>
    </div>
  );
}
