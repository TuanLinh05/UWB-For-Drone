import { useState, useRef, useCallback, useEffect } from 'react';
import type {
  AnchorSample,
  AnchorPosition,
  ChartPoint,
  ConnectionMode,
  FirmwareInfo,
  LinkHealth,
  RangeSample,
  StatsSample,
  TelemetryState,
} from '../lib/types';
import {
  TelemetryBinaryParser,
  type TelemetryBinaryDiagnostics,
  type TelemetryBinaryPacket,
} from '../lib/telemetryBinary';
import {
  canonicalizeRangeSample,
  decodeFirmwareInfo,
  missingTransportPackets,
} from '../lib/telemetryModel';
import {
  buildReplayCsv,
  createReplayLogContext,
  type ReplayLogContext,
} from '../lib/replayLog';

export {
  type AnchorSample,
  type ChartPoint,
  type ConnectionMode,
  type FirmwareInfo,
  type LinkHealth,
  type RangeSample,
  type StatsSample,
  type TelemetryState,
};

const RANGE_HISTORY_MAX = 600;
const CHART_HISTORY_MAX = 600;
/* Keep the UI chart at 25 Hz so Recharts does not compete with the 50 Hz
 * WebSocket stream. latestRange, CSV logging and the position callback still
 * receive every frame. */
const UI_HISTORY_MIN_INTERVAL_MS = 40;
const RANGE_STALE_AFTER_MS = 750;
const WATCHDOG_INTERVAL_MS = 200;
/** Display-only coasting for isolated UWB misses; canonical data stays invalid. */
const FILTERED_CHART_HOLD_MS = 100;

const EMPTY_BINARY_DIAGNOSTICS: TelemetryBinaryDiagnostics = {
  framesSeen: 0,
  framesDecoded: 0,
  crcErrors: 0,
  versionErrors: 0,
  lengthErrors: 0,
  unknownTypeFrames: 0,
  discardedBytes: 0,
};

function asFiniteNumber(value: unknown): number | null {
  const parsed = typeof value === 'number' ? value : Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function wifiAnchor(input: Record<string, unknown>): AnchorSample | null {
  const id = asFiniteNumber(input.id);
  const ageMs = asFiniteNumber(input.age_ms);
  if (id === null || ageMs === null) return null;
  return {
    id,
    valid: input.valid === true || input.valid === 1,
    ageMs,
    rawMm: asFiniteNumber(input.raw_mm),
    filtMm: asFiniteNumber(input.filt_mm),
    fppDbm: asFiniteNumber(input.fpp_dbm),
    ...(asFiniteNumber(input.status) === null ? {} : { status: asFiniteNumber(input.status) as number }),
  };
}

export function useUwbTelemetry(maxHistory: number = CHART_HISTORY_MAX) {
  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);
  const [connectionMode, setConnectionMode] = useState<ConnectionMode>('disconnected');
  const [telemetryState, setTelemetryState] = useState<TelemetryState>('disconnected');
  const [lastPacketAt, setLastPacketAt] = useState<number | null>(null);
  const [dataSessionId, setDataSessionId] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const [dataHistory, setDataHistory] = useState<ChartPoint[]>([]);
  const [rangeHistory, setRangeHistory] = useState<RangeSample[]>([]);
  const [latestRange, setLatestRange] = useState<RangeSample | null>(null);
  const [latestStats, setLatestStats] = useState<StatsSample | null>(null);
  const [firmwareInfo, setFirmwareInfo] = useState<FirmwareInfo | null>(null);
  const [isLogging, setIsLogging] = useState(false);

  const portRef = useRef<any>(null);
  const readerRef = useRef<any>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const keepReadingRef = useRef(false);
  const sessionIdRef = useRef(0);
  const connectedRef = useRef(false);
  const connectedAtRef = useRef<number | null>(null);
  const lastRangeAtRef = useRef<number | null>(null);
  const lastSourceSeqRef = useRef<number | null>(null);
  const lastMcuTimeRef = useRef<number | null>(null);
  const telemetryStateRef = useRef<TelemetryState>('disconnected');
  const fullLogRef = useRef<RangeSample[]>([]);
  const replayLogContextRef = useRef<ReplayLogContext | null>(null);
  const isLoggingRef = useRef(false);
  const connectionModeRef = useRef<ConnectionMode>('disconnected');
  const firmwareInfoRef = useRef<FirmwareInfo | null>(null);
  const linkHealthRef = useRef<LinkHealth>({ lastSeq: null, rangeGapCount: 0 });
  const latestStatsRef = useRef<StatsSample | null>(null);
  const usbDiagnosticsRef = useRef<TelemetryBinaryDiagnostics>(EMPTY_BINARY_DIAGNOSTICS);
  const chartFilterHoldRef = useRef<Record<number, number>>({});
  const lastHistoryAtRef = useRef<number | null>(null);

  /** External high-rate consumer (the position estimator). */
  const onRangeSampleRef = useRef<((sample: RangeSample) => void) | null>(null);

  const updateTelemetryState = useCallback((next: TelemetryState) => {
    telemetryStateRef.current = next;
    setTelemetryState(next);
  }, []);

  const stopLogging = useCallback(() => {
    isLoggingRef.current = false;
    setIsLogging(false);
  }, []);

  const prepareSession = useCallback(() => {
    linkHealthRef.current = { lastSeq: null, rangeGapCount: 0 };
    latestStatsRef.current = null;
    usbDiagnosticsRef.current = EMPTY_BINARY_DIAGNOSTICS;
    lastRangeAtRef.current = null;
    lastSourceSeqRef.current = null;
    lastMcuTimeRef.current = null;
    chartFilterHoldRef.current = {};
    lastHistoryAtRef.current = null;
    setLastPacketAt(null);
    setLatestRange(null);
    setLatestStats(null);
    setFirmwareInfo(null);
    firmwareInfoRef.current = null;
    setDataHistory([]);
    setRangeHistory([]);
    fullLogRef.current = [];
    replayLogContextRef.current = null;
    stopLogging();
    setDataSessionId(previous => previous + 1);
  }, [stopLogging]);

  const markDisconnected = useCallback((message?: string) => {
    connectedRef.current = false;
    connectedAtRef.current = null;
    lastRangeAtRef.current = null;
    setIsConnected(false);
    setIsConnecting(false);
    setConnectionMode('disconnected');
    connectionModeRef.current = 'disconnected';
    updateTelemetryState('disconnected');
    setLatestRange(null);
    setLatestStats(null);
    latestStatsRef.current = null;
    stopLogging();
    if (message) setError(message);
  }, [stopLogging, updateTelemetryState]);

  const handleFirmwarePacket = useCallback((packet: Extract<TelemetryBinaryPacket, { kind: 'info' }>) => {
    const info = decodeFirmwareInfo({
      schemaVersion: packet.schemaVersion,
      flags: packet.flags,
      rangingMode: packet.rangingMode,
      dsCalibratedMask: packet.dsCalibratedMask,
      rangeFilterMode: packet.filterMode,
      phyProfileId: packet.phyProfileId,
      spiClockMhz: packet.spiClockMhz,
      c9_2MotionMode: packet.c9_2MotionMode,
      c9_2GlobalMotionState: packet.c9_2GlobalMotionState,
      offsets: packet.activeOffsets,
    });
    if (info) {
      firmwareInfoRef.current = info;
      setFirmwareInfo(info);
      if (isLoggingRef.current && replayLogContextRef.current?.firmwareInfo === null) {
        replayLogContextRef.current = createReplayLogContext({
          startedAtMs: replayLogContextRef.current.startedAtMs,
          connectionMode: replayLogContextRef.current.connectionMode,
          firmwareInfo: info,
          anchorLayout: replayLogContextRef.current.anchorLayout,
          positionRangeSource: replayLogContextRef.current.positionRangeSource,
        });
      }
    }
  }, []);

  const handleRangeInput = useCallback((input: {
    seq: number;
    transportSeq?: number;
    timeMs: number;
    anchors: AnchorSample[];
  }) => {
    const receivedAt = Date.now();
    const sample = canonicalizeRangeSample({ ...input, clientTime: receivedAt });
    if (!sample) return;

    const sourceWentBackwards = lastSourceSeqRef.current !== null
      && ((sample.seq - lastSourceSeqRef.current) >>> 0) >= 0x8000_0000;
    const clockWentBackwards = lastMcuTimeRef.current !== null
      && ((sample.timeMs - lastMcuTimeRef.current) >>> 0) >= 0x8000_0000;
    if (sourceWentBackwards && clockWentBackwards) {
      // The STM32 rebooted while USB/WiFi remained open. A fresh data epoch
      // prevents histories, estimator time and calibration metadata from mixing.
      prepareSession();
    }
    lastSourceSeqRef.current = sample.seq;
    lastMcuTimeRef.current = sample.timeMs;

    const health = linkHealthRef.current;
    health.rangeGapCount += missingTransportPackets(health.lastSeq, sample.transportSeq);
    health.lastSeq = sample.transportSeq;

    lastRangeAtRef.current = receivedAt;
    setLastPacketAt(receivedAt);
    if (telemetryStateRef.current !== 'fresh') updateTelemetryState('fresh');
    setLatestRange(sample);

    /* Decimate render-only histories. This avoids cloning/rerendering two
     * 600-element arrays at 50 Hz, without starving calibration, CSV logging
     * or the position estimator of source frames. */
    const shouldRecordHistory = lastHistoryAtRef.current === null
      || receivedAt - lastHistoryAtRef.current >= UI_HISTORY_MIN_INTERVAL_MS;

    if (shouldRecordHistory) {
      lastHistoryAtRef.current = receivedAt;
      const byId = sample.anchorsById;
      const chartFilteredValue = (id: number): number | null => {
        const anchor = byId[id];
        if (anchor?.filtMm !== null && anchor?.filtMm !== undefined) {
          chartFilterHoldRef.current[id] = anchor.filtMm;
          return anchor.filtMm;
        }

        /* Bridge only a short, explicitly stale interval in the visual filtered
         * trace. Raw data, valid flags, uptime and solver inputs remain untouched.
         * A real outage longer than the hold window still creates a chart gap. */
        if (anchor && anchor.ageMs <= FILTERED_CHART_HOLD_MS) {
          return chartFilterHoldRef.current[id] ?? null;
        }
        return null;
      };

      const point: ChartPoint = {
        time: sample.clientTime,
        cyc: latestStatsRef.current?.cycHz ?? 0,
        ops: latestStatsRef.current?.opsHz ?? 0,
        a1Raw: byId[1]?.rawMm ?? byId[1]?.diagnosticRawMm ?? null,
        a1Filter: chartFilteredValue(1),
        a2Raw: byId[2]?.rawMm ?? byId[2]?.diagnosticRawMm ?? null,
        a2Filter: chartFilteredValue(2),
        a3Raw: byId[3]?.rawMm ?? byId[3]?.diagnosticRawMm ?? null,
        a3Filter: chartFilteredValue(3),
        a4Raw: byId[4]?.rawMm ?? byId[4]?.diagnosticRawMm ?? null,
        a4Filter: chartFilteredValue(4),
      };

      setDataHistory(previous => {
        const next = [...previous, point];
        return next.length > maxHistory ? next.slice(next.length - maxHistory) : next;
      });
      setRangeHistory(previous => {
        const next = [...previous, sample];
        return next.length > RANGE_HISTORY_MAX ? next.slice(next.length - RANGE_HISTORY_MAX) : next;
      });
    }

    if (isLoggingRef.current) fullLogRef.current.push(sample);
    onRangeSampleRef.current?.(sample);
  }, [maxHistory, prepareSession, updateTelemetryState]);

  const handleStatsSample = useCallback((stats: StatsSample) => {
    setLatestStats(stats);
    latestStatsRef.current = stats;
  }, []);

  const handleBinaryPacket = useCallback((packet: TelemetryBinaryPacket) => {
    if (packet.kind === 'range') {
      handleRangeInput({
        seq: packet.seq,
        transportSeq: packet.seq,
        timeMs: packet.timeMs,
        anchors: packet.anchors,
      });
    } else if (packet.kind === 'stats') {
      handleStatsSample(packet.stats);
    } else {
      handleFirmwarePacket(packet);
    }
  }, [handleFirmwarePacket, handleRangeInput, handleStatsSample]);

  const connect = useCallback(async () => {
    if (isConnecting || connectedRef.current) return;
    setIsConnecting(true);
    setError(null);
    const sessionId = ++sessionIdRef.current;

    try {
      if (!('serial' in navigator)) {
        throw new Error('Web Serial is not supported. Use Chrome or Edge over HTTPS/localhost.');
      }

      const port = await (navigator as any).serial.requestPort();
      if (sessionId !== sessionIdRef.current) return;
      await port.open({ baudRate: 115200 });
      if (sessionId !== sessionIdRef.current) {
        await port.close();
        return;
      }

      portRef.current = port;
      prepareSession();
      keepReadingRef.current = true;
      connectedRef.current = true;
      connectedAtRef.current = Date.now();
      setIsConnected(true);
      setIsConnecting(false);
      setConnectionMode('usb');
      connectionModeRef.current = 'usb';
      updateTelemetryState('waiting');

      const parser = new TelemetryBinaryParser();
      const reader = port.readable.getReader();
      readerRef.current = reader;

      try {
        while (keepReadingRef.current && sessionId === sessionIdRef.current) {
          const { value, done } = await reader.read();
          if (done) break;
          if (!(value instanceof Uint8Array)) continue;
          for (const packet of parser.push(value)) handleBinaryPacket(packet);
          usbDiagnosticsRef.current = parser.diagnostics;
        }
        if (keepReadingRef.current && sessionId === sessionIdRef.current) {
          markDisconnected('USB telemetry stream ended.');
        }
      } catch (readError) {
        if (keepReadingRef.current && sessionId === sessionIdRef.current) {
          const message = readError instanceof Error ? readError.message : 'Unknown serial read error';
          markDisconnected(`USB connection lost: ${message}`);
        }
      } finally {
        if (readerRef.current === reader) readerRef.current = null;
        try { reader.releaseLock(); } catch { /* already released during manual disconnect */ }
        if (sessionId === sessionIdRef.current && !connectedRef.current && portRef.current === port) {
          portRef.current = null;
          try { await port.close(); } catch { /* device already closed */ }
        }
      }
    } catch (connectError) {
      if (sessionId !== sessionIdRef.current) return;
      setIsConnecting(false);
      const message = connectError instanceof Error ? connectError.message : 'Failed to connect via USB';
      setError(message);
      updateTelemetryState('disconnected');
    }
  }, [handleBinaryPacket, isConnecting, markDisconnected, prepareSession, updateTelemetryState]);

  const connectWifi = useCallback((ipAddress: string) => {
    if (isConnecting || connectedRef.current) return;
    const host = ipAddress.trim();
    if (!host) {
      setError('Enter the ESP32 IP address.');
      return;
    }

    setIsConnecting(true);
    setError(null);
    const sessionId = ++sessionIdRef.current;

    try {
      const ws = new WebSocket(`ws://${host}:81`);
      wsRef.current = ws;

      ws.onopen = () => {
        if (sessionId !== sessionIdRef.current) return;
        prepareSession();
        connectedRef.current = true;
        connectedAtRef.current = Date.now();
        setIsConnected(true);
        setIsConnecting(false);
        setConnectionMode('wifi');
        connectionModeRef.current = 'wifi';
        updateTelemetryState('waiting');
      };

      ws.onmessage = event => {
        if (sessionId !== sessionIdRef.current || typeof event.data !== 'string') return;
        let message: Record<string, any>;
        try {
          message = JSON.parse(event.data) as Record<string, any>;
        } catch {
          return;
        }

        if (message.t === 'r') {
          const anchors = Array.isArray(message.anchors)
            ? message.anchors
                .map((anchor: unknown) => anchor && typeof anchor === 'object'
                  ? wifiAnchor(anchor as Record<string, unknown>)
                  : null)
                .filter((anchor: AnchorSample | null): anchor is AnchorSample => anchor !== null)
            : [];
          const sourceSeq = asFiniteNumber(message.source_seq ?? message.seq);
          const transportSeq = asFiniteNumber(message.ws_seq ?? message.seq);
          const timeMs = asFiniteNumber(message.time_ms);
          if (sourceSeq !== null && transportSeq !== null && timeMs !== null) {
            handleRangeInput({ seq: sourceSeq, transportSeq, timeMs, anchors });
          }
        } else if (message.t === 's') {
          const wifiRangeHz = asFiniteNumber(message.ws_range_hz);
          const wifiRangeDropCount = asFiniteNumber(message.ws_range_drop_count);
          const wifiRangeIntervalMs = asFiniteNumber(message.ws_range_interval_ms);
          handleStatsSample({
            pollSent: Number(message.poll_sent),
            responseOk: Number(message.response_ok),
            rxTimeout: Number(message.rx_timeout),
            rxError: Number(message.rx_error),
            cycleOverrun: Number(message.cycle_overrun),
            uartOverflow: Number(message.uart_overflow),
            cycHz: Number(message.cyc_hz),
            opsHz: Number(message.ops_hz),
            uartGapCount: Number(message.uart_gap_count ?? 0),
            crcErrorCount: Number(message.crc_error_count ?? 0),
            ...(wifiRangeHz === null ? {} : { wifiRangeHz }),
            ...(wifiRangeDropCount === null ? {} : { wifiRangeDropCount }),
            ...(wifiRangeIntervalMs === null ? {} : { wifiRangeIntervalMs }),
          });
        } else if (message.t === 'i') {
          const offsets = Array.isArray(message.offsets)
            ? message.offsets.map((offset: Record<string, unknown>) => ({
                id: Number(offset.id),
                activeOffsetUm: Number(offset.active_offset_um),
              }))
            : [];
          const info = decodeFirmwareInfo({
            schemaVersion: Number(message.schema),
            flags: Number(message.flags),
            rangingMode: Number(message.ranging_mode),
            dsCalibratedMask: Number(message.ds_calibrated_mask),
            rangeFilterMode: Number(message.filter_mode),
            phyProfileId: Number(message.phy_profile_id ?? 0),
            spiClockMhz: Number(message.spi_clock_mhz ?? 0),
            c9_2MotionMode: Number(message.c9_2_motion_mode ?? 0),
            c9_2GlobalMotionState: Number(message.c9_2_global_motion_state ?? 0),
            offsets,
            ...(typeof message.firmware_build_id === 'string'
              ? { firmwareBuildId: message.firmware_build_id }
              : {}),
          });
          if (info) {
            firmwareInfoRef.current = info;
            setFirmwareInfo(info);
            if (isLoggingRef.current && replayLogContextRef.current?.firmwareInfo === null) {
              replayLogContextRef.current = createReplayLogContext({
                startedAtMs: replayLogContextRef.current.startedAtMs,
                connectionMode: replayLogContextRef.current.connectionMode,
                firmwareInfo: info,
                anchorLayout: replayLogContextRef.current.anchorLayout,
                positionRangeSource: replayLogContextRef.current.positionRangeSource,
              });
            }
          }
        }
      };

      ws.onerror = () => {
        if (sessionId !== sessionIdRef.current) return;
        setError('WebSocket error. Check the ESP32 IP and bridge status.');
      };
      ws.onclose = () => {
        if (sessionId !== sessionIdRef.current) return;
        wsRef.current = null;
        markDisconnected(connectedRef.current ? 'WiFi connection closed.' : undefined);
      };
    } catch (connectError) {
      if (sessionId !== sessionIdRef.current) return;
      const message = connectError instanceof Error ? connectError.message : 'Failed to connect via WiFi';
      setError(message);
      setIsConnecting(false);
      updateTelemetryState('disconnected');
    }
  }, [handleRangeInput, handleStatsSample, isConnecting, markDisconnected, prepareSession, updateTelemetryState]);

  const disconnect = useCallback(async () => {
    ++sessionIdRef.current;
    keepReadingRef.current = false;
    const reader = readerRef.current;
    readerRef.current = null;
    try { await reader?.cancel(); } catch { /* device may already be gone */ }
    try { reader?.releaseLock(); } catch { /* read loop may own/release it */ }

    const port = portRef.current;
    portRef.current = null;
    try { await port?.close(); } catch { /* stream may already be closed */ }

    const ws = wsRef.current;
    wsRef.current = null;
    try { ws?.close(); } catch { /* already closed */ }
    markDisconnected();
  }, [markDisconnected]);

  useEffect(() => {
    const timer = window.setInterval(() => {
      if (!connectedRef.current) return;
      const baseline = lastRangeAtRef.current ?? connectedAtRef.current;
      if (baseline === null || Date.now() - baseline <= RANGE_STALE_AFTER_MS) return;
      if (telemetryStateRef.current === 'stale') return;
      updateTelemetryState('stale');
      setLatestRange(null);
      setLatestStats(null);
      latestStatsRef.current = null;
      // Keep history for diagnostics/export, but no live consumer can observe an
      // old range as if it were current. App resets the estimator on this state.
    }, WATCHDOG_INTERVAL_MS);
    return () => window.clearInterval(timer);
  }, [updateTelemetryState]);

  useEffect(() => () => {
    ++sessionIdRef.current;
    keepReadingRef.current = false;
    try { readerRef.current?.cancel(); } catch { /* unmount cleanup */ }
    try { wsRef.current?.close(); } catch { /* unmount cleanup */ }
  }, []);

  const startLog = useCallback((options?: {
    anchorLayout?: readonly AnchorPosition[];
    positionRangeSource?: 'raw' | 'filtered';
  }) => {
    fullLogRef.current = [];
    replayLogContextRef.current = createReplayLogContext({
      connectionMode: connectionModeRef.current,
      firmwareInfo: firmwareInfoRef.current,
      anchorLayout: options?.anchorLayout,
      positionRangeSource: options?.positionRangeSource,
    });
    isLoggingRef.current = true;
    setIsLogging(true);
  }, []);

  const exportCsv = useCallback(() => {
    const log = fullLogRef.current;
    if (log.length === 0) {
      window.alert('No logged data. Start logging and capture at least one packet.');
      return;
    }

    const context = replayLogContextRef.current ?? createReplayLogContext({
      connectionMode: connectionModeRef.current,
      firmwareInfo: firmwareInfoRef.current,
    });
    const blob = new Blob([buildReplayCsv(log, context)], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `uwb_log_${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}.csv`;
    document.body.appendChild(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
  }, []);

  return {
    isConnected,
    isConnecting,
    isDataFresh: telemetryState === 'fresh',
    dataSessionId,
    connectionMode,
    telemetryState,
    lastPacketAt,
    error,
    latestRange,
    latestStats,
    firmwareInfo,
    dataHistory,
    rangeHistory,
    connect,
    connectWifi,
    disconnect,
    isLogging,
    startLog,
    stopLog: stopLogging,
    exportCsv,
    linkHealth: linkHealthRef.current,
    usbDiagnostics: usbDiagnosticsRef.current,
    onRangeSampleRef,
  };
}
