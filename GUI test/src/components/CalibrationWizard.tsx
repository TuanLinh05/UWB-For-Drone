import { useState, useRef, useEffect, useMemo, useCallback } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';
import {
  ChevronRight, ChevronLeft, CheckCircle2, AlertTriangle, Download, Copy, Check, FlaskConical
} from 'lucide-react';
import {
  SUPPORTED_ANCHOR_IDS,
  type FirmwareInfo,
  type RangeSample,
} from '../lib/types';
import { analyzeFppBias } from '../lib/fppBiasAnalysis';
import { analyzeStaticCalibrationHoldout } from '../lib/staticCalibrationAnalysis';

// ---- Types from plan mục 13b ----
interface CalibrationPoint {
  trueDistanceM: number;
  samples: number[];
  fppSamples: number[];   // FPP (dBm) song song với samples — dùng cho FPP Bias Check
  meanMm: number;
  stdDevMm: number;
  offsetM: number;
}
interface AnchorCalibration {
  anchorId: number;
  points: CalibrationPoint[];
}

export type CalibrationFirmwareInfo = Pick<
  FirmwareInfo,
  'calibrationProfile' | 'rangingMode' | 'phyProfile' | 'activeOffsetsM' | 'firmwareBuildId'
>;

interface CalibrationResult {
  anchorId: number;
  currentOffsetM: number;
  deltaM: number;
  proposedOffsetM: number;
  measured: boolean;
}

function computeCalibrationPoint(trueDistanceM: number, samples: number[], fppSamples: number[]): CalibrationPoint {
  const meanMm = samples.reduce((a, b) => a + b, 0) / samples.length;
  const variance = samples.reduce((s, v) => s + (v - meanMm) ** 2, 0) / samples.length;
  return { trueDistanceM, samples, fppSamples, meanMm, stdDevMm: Math.sqrt(variance), offsetM: meanMm / 1000 - trueDistanceM };
}

function finalOffsetForAnchor(cal: AnchorCalibration): number {
  const offsets = cal.points.map(p => p.offsetM);
  return offsets.reduce((a, b) => a + b, 0) / offsets.length;
}

function activeOffsetForAnchor(firmwareInfo: CalibrationFirmwareInfo | null, anchorId: number): number | null {
  if (!firmwareInfo || !Object.prototype.hasOwnProperty.call(firmwareInfo.activeOffsetsM, anchorId)) return null;
  const value = firmwareInfo.activeOffsetsM[anchorId];
  return Number.isFinite(value) ? value : null;
}

function buildCalibrationResults(
  cals: AnchorCalibration[],
  firmwareInfo: CalibrationFirmwareInfo,
): CalibrationResult[] | null {
  const deltaById = new Map(cals.map(cal => [cal.anchorId, finalOffsetForAnchor(cal)]));
  const results: CalibrationResult[] = [];

  // SS arrays need all four current values for a safe replacement initializer.
  // DS emits only measured macros, so it never touches an unmeasured anchor.
  const anchorIds = firmwareInfo.calibrationProfile === 'ds'
    ? [...deltaById.keys()].sort((a, b) => a - b)
    : [...SUPPORTED_ANCHOR_IDS];
  for (const anchorId of anchorIds) {
    const currentOffsetM = activeOffsetForAnchor(firmwareInfo, anchorId);
    if (currentOffsetM === null) return null;
    const deltaM = deltaById.get(anchorId) ?? 0;
    /* DS raw samples are captured before the firmware applies any DS offset.
     * Therefore deltaM is already the complete replacement offset, rather
     * than an increment to the value reported by INFO telemetry. */
    const proposedOffsetM = firmwareInfo.calibrationProfile === 'ds' && deltaById.has(anchorId)
      ? deltaM
      : currentOffsetM + deltaM;
    results.push({
      anchorId,
      currentOffsetM,
      deltaM,
      proposedOffsetM,
      measured: deltaById.has(anchorId),
    });
  }

  return results;
}

function generateFirmwareSnippet(
  firmwareInfo: CalibrationFirmwareInfo,
  results: CalibrationResult[],
): string {
  const buildComment = firmwareInfo.firmwareBuildId
    ? ` Measured on firmware ${firmwareInfo.firmwareBuildId}.`
    : '';

  if (firmwareInfo.calibrationProfile === 'ds') {
    const measured = results.filter(result => result.measured);
    const macros = measured.map(result =>
      `#define UWB_DS_OFFSET_A${result.anchorId}_M   ${result.proposedOffsetM.toFixed(6)}`
      + `  /* replacement offset from DS raw delta ${result.deltaM.toFixed(6)} */`
    );
    const maskBits = measured.map(result => `UWB_DS_CAL_A${result.anchorId}_BIT`).join(' | ');
    return [
      `/* DS-TWR calibration.${buildComment} */`,
      '/* Replace ONLY the matching UWB_DS_OFFSET_A*_M macros in uwb_calibration.h.',
      ' * Leave every unlisted anchor unchanged. Validate each result before enabling its bit. */',
      ...macros,
      '',
      `/* After validation, OR these bits into the existing UWB_DS_CALIBRATED_MASK: ${maskBits} */`,
    ].join('\n');
  }

  const target = firmwareInfo.calibrationProfile === 'legacy'
    ? 'calibration_offset_m'
    : 'residual_offset_m';
  const calibratedMask = firmwareInfo.calibrationProfile === 'legacy'
    ? 'UWB_SS_LEGACY_CALIBRATED_MASK'
    : 'UWB_SS_RESIDUAL_CALIBRATED_MASK';
  const measuredMaskBits = results
    .filter(result => result.measured)
    .map(result => `UWB_SS_CAL_A${result.anchorId}_BIT`)
    .join(' | ');
  const values = results.map(result =>
    `  ${result.proposedOffsetM.toFixed(6)}  /* A${result.anchorId}: ${result.measured
      ? `current ${result.currentOffsetM.toFixed(6)} + delta ${result.deltaM.toFixed(6)}`
      : 'unchanged current value'} */`
  );

  return [
    `/* Replace the existing ${target} declaration in tag_ranging.c.${buildComment} */`,
    `volatile double ${target}[TAG_NUM_ANCHORS] = {`,
    values.join(',\n'),
    '};',
    '',
    `/* After validation, OR these bits into ${calibratedMask}: ${measuredMaskBits} */`,
  ].join('\n');
}

// ---- Props ----
interface Props {
  latestRange: RangeSample | null;
  isConnected: boolean;
  firmwareInfo: CalibrationFirmwareInfo | null;
}

type Step = 1 | 2 | 3 | 4 | 5 | 6 | 7;
const TOTAL_STEPS = 7;
const MIN_SAMPLES = 100;
const MIN_DISTANCE_M = 0.1;
const MAX_DISTANCE_M = 20;
const MAX_SAMPLES = 2000;  // Phase 5: safety cap — auto-stop after this many
const WARN_OFFSET_DIFF_M = 0.05;
const TAG_ST_DS_FALLBACK = 0x10;
const TAG_ST_CALIBRATION_MISSING = 0x20;
type SupportedAnchorId = typeof SUPPORTED_ANCHOR_IDS[number];

export function CalibrationWizard({ latestRange, isConnected, firmwareInfo }: Props) {

  const [step, setStep] = useState<Step>(1);
  const [selectedAnchorId, setSelectedAnchorId] = useState<SupportedAnchorId>(1);
  const [trueDistanceM, setTrueDistanceM] = useState('');
  const [collecting, setCollecting] = useState(false);
  const [liveSampleCount, setLiveSampleCount] = useState(0);
  const [livePreview, setLivePreview] = useState<number[]>([]);   // chỉ 200 mẫu gần nhất, cho chart preview
  const [calibrations, setCalibrations] = useState<AnchorCalibration[]>([]);
  const [currentAnchorPoints, setCurrentAnchorPoints] = useState<CalibrationPoint[]>([]);
  const [lastPoint, setLastPoint] = useState<CalibrationPoint | null>(null);
  const [snippet, setSnippet] = useState('');
  const [calibrationResults, setCalibrationResults] = useState<CalibrationResult[]>([]);
  const [resultFirmwareInfo, setResultFirmwareInfo] = useState<CalibrationFirmwareInfo | null>(null);
  const [copied, setCopied] = useState(false);

  const samplesRef = useRef<number[]>([]);
  const fppSamplesRef = useRef<number[]>([]);
  const collectingRef = useRef(false);
  const captureProfileKeyRef = useRef<string | null>(null);

  const selectedCurrentOffsetM = activeOffsetForAnchor(firmwareInfo, selectedAnchorId);
  const currentProfileKey = firmwareInfo && selectedCurrentOffsetM !== null
    ? [
        firmwareInfo.firmwareBuildId ?? 'unknown-build',
        firmwareInfo.calibrationProfile,
        firmwareInfo.rangingMode,
        firmwareInfo.phyProfile,
        selectedAnchorId,
        selectedCurrentOffsetM,
      ].join(':')
    : null;

  const profileBlockReason = useMemo(() => {
    if (!firmwareInfo) return 'Waiting for firmware calibration information.';
    if (firmwareInfo.calibrationProfile === 'unknown') return 'Firmware calibration profile is unknown.';
    if (firmwareInfo.rangingMode === 'unknown') return 'Firmware ranging mode is unknown.';
    if (firmwareInfo.calibrationProfile === 'ds' && firmwareInfo.rangingMode !== 'ds') {
      return 'DS calibration profile does not match the active ranging mode.';
    }
    if (firmwareInfo.calibrationProfile !== 'ds' && firmwareInfo.rangingMode !== 'ss') {
      return 'SS calibration profile does not match the active ranging mode.';
    }
    if (selectedCurrentOffsetM === null) {
      return `Firmware did not report the active offset for A${selectedAnchorId}.`;
    }
    return null;
  }, [firmwareInfo, selectedAnchorId, selectedCurrentOffsetM]);

  const resultBlockReason = useMemo(() => {
    if (profileBlockReason) return profileBlockReason;
    if (!firmwareInfo) return 'Waiting for firmware calibration information.';
    if (currentAnchorPoints.length === 0) return `A${selectedAnchorId} has no completed measurements.`;

    const measuredAnchorIds = new Set(calibrations.map(calibration => calibration.anchorId));
    measuredAnchorIds.add(selectedAnchorId);
    const requiredAnchorIds = firmwareInfo.calibrationProfile === 'ds'
      ? [...measuredAnchorIds]
      : [...SUPPORTED_ANCHOR_IDS];
    const missing = requiredAnchorIds.filter(anchorId => activeOffsetForAnchor(firmwareInfo, anchorId) === null);
    return missing.length > 0
      ? `Firmware did not report active offsets for ${missing.map(anchorId => `A${anchorId}`).join(', ')}.`
      : null;
  }, [profileBlockReason, firmwareInfo, currentAnchorPoints.length, calibrations, selectedAnchorId]);

  const finalizeCapture = useCallback(() => {
    collectingRef.current = false;
    captureProfileKeyRef.current = null;
    setCollecting(false);
    setLiveSampleCount(samplesRef.current.length);

    const distanceM = Number(trueDistanceM);
    if (!Number.isFinite(distanceM) || distanceM < MIN_DISTANCE_M || distanceM > MAX_DISTANCE_M) return;

    const samples = [...samplesRef.current];
    const fppSamples = [...fppSamplesRef.current];
    if (samples.length < MIN_SAMPLES) return;

    const point = computeCalibrationPoint(distanceM, samples, fppSamples);
    setLastPoint(point);
    setCurrentAnchorPoints(previous => [...previous, point]);
    setStep(4);
  }, [trueDistanceM]);

  // Watch latestRange for sample collection.
  // Không setState bản copy toàn bộ mảng mỗi mẫu (O(n²) nếu để collecting chạy
  // lâu) — samplesRef là nguồn dữ liệu thật; React chỉ cần biết SỐ LƯỢNG (rẻ)
  // và 1 cửa sổ preview cắt sẵn ở nguồn (luôn O(200), không phụ thuộc n), và
  // chỉ cập nhật mỗi 4 mẫu (~12.5Hz) — đủ mượt cho mắt người, đỡ tốn re-render.
  useEffect(() => {
    if (!collectingRef.current || !latestRange) return;
    type CalibrationAnchorSample = RangeSample['anchors'][number] & {
      diagnosticRawMm?: number | null;
      diagnosticFppDbm?: number | null;
    };
    const rangeWithIndex = latestRange as RangeSample & {
      anchorsById?: Partial<Record<number, CalibrationAnchorSample>>;
    };
    const anchor = rangeWithIndex.anchorsById?.[selectedAnchorId]
      ?? latestRange.anchors.find(sample => sample.id === selectedAnchorId) as CalibrationAnchorSample | undefined;
    if (!anchor) return;

    const status = anchor.status;
    const isDs = firmwareInfo?.rangingMode === 'ds';
    const isDsFallback = status !== undefined && (status & TAG_ST_DS_FALLBACK) !== 0;
    const isCalibrationDiagnostic = status !== undefined
      && (status & TAG_ST_CALIBRATION_MISSING) !== 0;
    const isFreshValidSample = anchor.valid && anchor.ageMs <= 500;

    // DS fallback is an SS measurement and must never contaminate a DS profile.
    // Missing-calibration diagnostics are deliberately invalid for production,
    // but their pre-offset raw value is the required bootstrap input for both
    // a new SS anchor (A4) and initial DS calibration.
    if (isDs && (status === undefined || isDsFallback)) return;
    if (!isFreshValidSample && !isCalibrationDiagnostic) return;

    const rawMm = isFreshValidSample ? anchor.rawMm : anchor.diagnosticRawMm;
    const fppDbm = isFreshValidSample ? anchor.fppDbm : anchor.diagnosticFppDbm;
    if (typeof rawMm !== 'number' || !Number.isFinite(rawMm)
      || typeof fppDbm !== 'number' || !Number.isFinite(fppDbm)) return;

    samplesRef.current.push(rawMm);
    fppSamplesRef.current.push(fppDbm);

    const n = samplesRef.current.length;

    // Phase 5: auto-stop safety cap
    if (n >= MAX_SAMPLES) {
      setLivePreview(samplesRef.current.slice(-200));
      finalizeCapture();
      return;
    }

    if (n % 4 === 0) {
      setLiveSampleCount(n);
      setLivePreview(samplesRef.current.slice(-200));
    }
  }, [latestRange, selectedAnchorId, firmwareInfo?.rangingMode, finalizeCapture]);

  // Phase 5: cleanup collection if component unmounts (tab switch away)
  useEffect(() => {
    return () => {
      if (collectingRef.current) {
        collectingRef.current = false;
      }
    };
  }, []);

  const startCollecting = () => {
    if (!isConnected || profileBlockReason) return;
    samplesRef.current = [];
    fppSamplesRef.current = [];
    setLiveSampleCount(0);
    setLivePreview([]);
    captureProfileKeyRef.current = currentProfileKey;
    collectingRef.current = true;
    setCollecting(true);
  };

  // P0-04: Cancel — always available when collecting, regardless of sample count
  const cancelCollecting = () => {
    collectingRef.current = false;
    setCollecting(false);
    samplesRef.current = [];
    fppSamplesRef.current = [];
    captureProfileKeyRef.current = null;
    setLiveSampleCount(0);
    setLivePreview([]);
  };

  // Never mix samples across a disconnect, firmware build, mode, profile, or active-offset change.
  useEffect(() => {
    if (!collectingRef.current) return;
    if (isConnected && !profileBlockReason && captureProfileKeyRef.current === currentProfileKey) return;

    collectingRef.current = false;
    captureProfileKeyRef.current = null;
    samplesRef.current = [];
    fppSamplesRef.current = [];
    setCollecting(false);
    setLiveSampleCount(0);
    setLivePreview([]);
  }, [isConnected, profileBlockReason, currentProfileKey]);


  const stopCollecting = finalizeCapture;

  const finishAnchor = () => {
    // Save current anchor calibration
    const existing = calibrations.find(c => c.anchorId === selectedAnchorId);
    if (existing) {
      setCalibrations(prev => prev.map(c => c.anchorId === selectedAnchorId
        ? { ...c, points: currentAnchorPoints }
        : c
      ));
    } else {
      setCalibrations(prev => [...prev, { anchorId: selectedAnchorId, points: currentAnchorPoints }]);
    }
    setStep(6);
  };

  const startNextAnchor = (nextId: SupportedAnchorId) => {
    setSelectedAnchorId(nextId);
    setCurrentAnchorPoints([]);
    setLastPoint(null);
    setTrueDistanceM('');
    setStep(1);
  };

  const generateResult = () => {
    if (!firmwareInfo || resultBlockReason || currentAnchorPoints.length === 0) return;

    const final = [...calibrations.filter(c => c.anchorId !== selectedAnchorId), { anchorId: selectedAnchorId, points: currentAnchorPoints }];
    const results = buildCalibrationResults(final, firmwareInfo);
    if (!results) return;

    setCalibrations(final);
    setCalibrationResults(results);
    setResultFirmwareInfo({ ...firmwareInfo, activeOffsetsM: { ...firmwareInfo.activeOffsetsM } });
    setSnippet(generateFirmwareSnippet(firmwareInfo, results));
    setStep(7);
  };

  const exportCalibCsv = () => {
    const header = 'anchor_id,true_distance_m,sample_index,raw_mm,fpp_dbm,mean_mm,residual_delta_m,current_offset_m,proposed_offset_m,profile,ranging_mode,phy_profile,firmware_build_id';
    const rows: string[] = [];
    calibrations.forEach(cal => {
      const result = calibrationResults.find(item => item.anchorId === cal.anchorId);
      cal.points.forEach(pt => {
        pt.samples.forEach((s, i) => {
          const fpp = pt.fppSamples[i] ?? 0;
          rows.push([
            cal.anchorId,
            pt.trueDistanceM,
            i,
            s,
            fpp.toFixed(2),
            pt.meanMm.toFixed(1),
            pt.offsetM.toFixed(6),
            result?.currentOffsetM.toFixed(6) ?? '',
            result?.proposedOffsetM.toFixed(6) ?? '',
            resultFirmwareInfo?.calibrationProfile ?? 'unknown',
            resultFirmwareInfo?.rangingMode ?? 'unknown',
            resultFirmwareInfo?.phyProfile ?? 'unknown',
            resultFirmwareInfo?.firmwareBuildId ?? '',
          ].join(','));
        });
      });
    });
    const csv = [header, ...rows].join('\n');
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `calibration_${new Date().toISOString().slice(0, 19).replace(/[:.]/g, '-')}.csv`;
    link.click();
    URL.revokeObjectURL(url);
  };

  const copySnippet = () => {
    navigator.clipboard.writeText(snippet).then(() => {
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    });
  };

  // Live chart data for step 3 — livePreview đã bị cắt sẵn còn 200 điểm ở nguồn
  const liveChartData = useMemo(() =>
    livePreview.map((v, i) => ({ i, v })),
    [livePreview]
  );

  // FPP Bias Check (Step 5) — thay cho tools/analyze_fpp_bias.py, chạy ngay
  // trong GUI trên dữ liệu đã thu của anchor đang calib.
  const fppBiasResult = useMemo(() =>
    analyzeFppBias(currentAnchorPoints.map(p => ({
      trueDistanceM: p.trueDistanceM,
      rawMm: p.samples,
      fppDbm: p.fppSamples,
    }))),
    [currentAnchorPoints]
  );

  const staticHoldoutResult = useMemo(() =>
    analyzeStaticCalibrationHoldout(currentAnchorPoints.map(point => ({
      trueDistanceM: point.trueDistanceM,
      rawMm: point.samples,
    }))),
    [currentAnchorPoints]
  );

  // Warning check for step 5
  const offsetWarning = useMemo(() => {
    if (currentAnchorPoints.length < 2) return false;
    const offsets = currentAnchorPoints.map(p => p.offsetM);
    const minO = Math.min(...offsets), maxO = Math.max(...offsets);
    return (maxO - minO) > WARN_OFFSET_DIFF_M;
  }, [currentAnchorPoints]);

  const d = Number(trueDistanceM);
  const distanceValid = Number.isFinite(d) && d >= MIN_DISTANCE_M && d <= MAX_DISTANCE_M;
  const currentDeltaM = currentAnchorPoints.length > 0
    ? finalOffsetForAnchor({ anchorId: selectedAnchorId, points: currentAnchorPoints })
    : null;
  const currentProposedOffsetM = selectedCurrentOffsetM !== null && currentDeltaM !== null
    ? selectedCurrentOffsetM + currentDeltaM
    : null;
  const profileLabel = firmwareInfo?.calibrationProfile === 'legacy'
    ? 'SS legacy software offset'
    : firmwareInfo?.calibrationProfile === 'residual-hw'
      ? 'SS hardware antenna delay + residual'
      : firmwareInfo?.calibrationProfile === 'ds'
        ? 'DS-TWR offset'
        : 'Unknown';
  const targetLabel = firmwareInfo?.calibrationProfile === 'legacy'
    ? 'calibration_offset_m'
    : firmwareInfo?.calibrationProfile === 'residual-hw'
      ? 'residual_offset_m'
      : firmwareInfo?.calibrationProfile === 'ds'
        ? `UWB_DS_OFFSET_A${selectedAnchorId}_M`
        : 'unknown target';

  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100%', padding: '1rem', gap: '0.75rem', overflowY: 'auto' }}>
      {/* Stepper header */}
      <StepperBar current={step} total={TOTAL_STEPS} />

      <div style={{
        display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(160px, 1fr))', gap: 8,
        padding: '10px 12px', border: '1px solid var(--color-border)', borderRadius: 8,
        background: 'var(--color-surface-2)', fontSize: '0.78rem', color: 'var(--color-text-muted)',
      }}>
        <div>Profile: <strong style={{ color: 'var(--color-text)' }}>{profileLabel}</strong></div>
        <div>Mode: <strong style={{ color: 'var(--color-text)' }}>{firmwareInfo?.rangingMode.toUpperCase() ?? '—'}</strong></div>
        <div>Target: <code style={{ color: 'var(--color-primary)' }}>{targetLabel}</code></div>
        <div>Current A{selectedAnchorId}: <strong style={{ color: 'var(--color-text)' }}>
          {selectedCurrentOffsetM === null ? '—' : `${selectedCurrentOffsetM.toFixed(6)} m`}
        </strong></div>
        {firmwareInfo?.firmwareBuildId && (
          <div style={{ gridColumn: '1 / -1' }}>Firmware build: <code>{firmwareInfo.firmwareBuildId}</code></div>
        )}
        {profileBlockReason && (
          <div style={{ gridColumn: '1 / -1', color: 'var(--color-danger)', display: 'flex', gap: 6, alignItems: 'center' }}>
            <AlertTriangle size={14} /> {profileBlockReason} Collection and result generation are locked.
          </div>
        )}
      </div>

      <div className="glass-panel" style={{ flex: 1 }}>
        {/* Step 1: Select anchor */}
        {step === 1 && (
          <StepPanel title="Step 1: Select Anchor to Calibrate" icon="🎯">
            <p style={hintStyle}>Choose the anchor you want to measure. Wizard will filter data automatically — other anchors can remain powered on.</p>
            <div style={{ display: 'flex', gap: '0.75rem', margin: '1rem 0' }}>
              {SUPPORTED_ANCHOR_IDS.map(id => (
                <button key={id} className={`btn ${selectedAnchorId === id ? 'btn--primary' : 'btn--secondary'}`}
                  style={{ flex: 1, padding: '0.75rem', fontSize: '1rem' }}
                  onClick={() => setSelectedAnchorId(id)}>
                  A{id}
                </button>
              ))}
            </div>
            <p style={{ fontSize: '0.75rem', color: 'var(--text-muted)' }}>
              <span style={{ color: 'var(--accent-green)' }}>✓</span> Wizard reads only A{selectedAnchorId} data regardless of which anchors are powered.
            </p>
            <NextBtn onClick={() => setStep(2)} />
          </StepPanel>
        )}

        {/* Step 2: Enter distance */}
        {step === 2 && (
          <StepPanel title="Step 2: Enter True Distance" icon="📏">
            <p style={hintStyle}>Place the TAG exactly at a known distance from Anchor {selectedAnchorId}. Measure with a tape/laser ruler.</p>
            <div className="input-group" style={{ margin: '1rem 0' }}>
              <label>True distance (meters):</label>
              <input className="form-input" type="number" step="0.01" min={MIN_DISTANCE_M} max={MAX_DISTANCE_M} placeholder="e.g. 2.00"
                value={trueDistanceM} onChange={e => setTrueDistanceM(e.target.value)}
                style={{ width: 120 }}
              />
              <span style={{ color: 'var(--text-muted)', fontSize: '0.85rem' }}>m</span>
            </div>
            {trueDistanceM !== '' && !distanceValid && (
              <p style={{ color: 'var(--color-danger)', fontSize: '0.78rem', marginBottom: 8 }}>
                Distance must be between {MIN_DISTANCE_M.toFixed(1)} m and {MAX_DISTANCE_M.toFixed(1)} m.
              </p>
            )}
            <div style={{ display: 'flex', gap: '0.5rem' }}>
              <BackBtn onClick={() => setStep(1)} />
              <NextBtn onClick={() => setStep(3)} disabled={!distanceValid} />
            </div>
          </StepPanel>
        )}

        {/* Step 3: Collect samples */}
        {step === 3 && (
          <StepPanel title="Step 3: Collect Samples" icon="📡">
            {!isConnected && (
              <div style={{ display: 'flex', gap: 8, padding: '8px 12px', background: 'rgba(239,68,68,0.1)', border: '1px solid rgba(239,68,68,0.3)', borderRadius: 8, marginBottom: 12, fontSize: '0.8125rem', color: '#fca5a5' }}>
                <AlertTriangle size={15} style={{ flexShrink: 0, marginTop: 1 }} />
                Not connected. Connect a device before collecting samples.
              </div>
            )}
            {isConnected && profileBlockReason && (
              <div style={{ display: 'flex', gap: 8, padding: '8px 12px', background: 'rgba(220,38,38,0.08)', border: '1px solid rgba(220,38,38,0.3)', borderRadius: 8, marginBottom: 12, fontSize: '0.8125rem', color: 'var(--color-danger)' }}>
                <AlertTriangle size={15} style={{ flexShrink: 0, marginTop: 1 }} />
                {profileBlockReason}
              </div>
            )}
            {firmwareInfo?.rangingMode === 'ds' && (
              <p style={{ ...hintStyle, color: 'var(--color-warning)' }}>
                DS capture rejects SS fallback packets.
              </p>
            )}
            <p style={{ ...hintStyle, color: 'var(--color-warning)' }}>
              An uncalibrated anchor is captured from its status-aware diagnostic raw value. It remains excluded from Position and production filtering until its calibration bit is enabled.
            </p>
            <p style={hintStyle}>Keep TAG still at {d.toFixed(2)}m from A{selectedAnchorId}. Collect ≥{MIN_SAMPLES} samples (~2s @ 50Hz). Auto-stops at {MAX_SAMPLES}.</p>
            <div style={{ display: 'flex', alignItems: 'center', gap: '1rem', margin: '0.75rem 0' }}>
              {!collecting ? (
                <button className="btn" style={{ background: 'var(--color-danger)', borderColor: 'var(--color-danger)', opacity: (!isConnected || profileBlockReason) ? 0.45 : 1 }}
                  onClick={startCollecting} disabled={!isConnected || Boolean(profileBlockReason)}>
                  ● Start Collecting
                </button>
              ) : (
                // P0-04: Finish (requires enough samples) AND Cancel (always available)
                <>
                  <button className="btn" style={{ background: 'var(--accent-green)', borderColor: 'var(--accent-green)' }}
                    onClick={stopCollecting} disabled={liveSampleCount < MIN_SAMPLES}>
                    ■ Finish ({liveSampleCount}/{MIN_SAMPLES})
                  </button>
                  <button className="btn btn-danger" style={{ marginTop: 0 }} onClick={cancelCollecting}>
                    ✕ Cancel
                  </button>
                </>
              )}
              <span style={{ color: liveSampleCount >= MIN_SAMPLES ? 'var(--accent-green)' : 'var(--text-muted)', fontSize: '0.85rem' }}>
                {liveSampleCount} / {MAX_SAMPLES} samples
              </span>
            </div>

            {livePreview.length > 0 && (
              <div style={{ height: 160, marginBottom: '0.75rem' }}>
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart data={liveChartData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="rgba(0,0,0,0.08)" />
                    <XAxis dataKey="i" hide />
                    <YAxis stroke="#94a3b8" unit=" mm" width={55} />
                    <Tooltip contentStyle={{ backgroundColor: 'var(--color-surface-1)', border: '1px solid var(--color-border)', borderRadius: 8 }} />
                    <Line type="monotone" dataKey="v" name="raw_mm" stroke="var(--accent-orange)" strokeWidth={1.5} dot={false} isAnimationActive={false} />
                  </LineChart>
                </ResponsiveContainer>
              </div>
            )}
            <BackBtn onClick={() => {
              if (collecting) { cancelCollecting(); }
              setStep(2);
            }} />
          </StepPanel>
        )}

        {/* Step 4: Results for this measurement */}
        {step === 4 && lastPoint && (
          <StepPanel title="Step 4: Measurement Result" icon="📊">
            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: '0.75rem', margin: '0.75rem 0' }}>
              <ResultCard label="True Distance" value={`${lastPoint.trueDistanceM.toFixed(3)} m`} />
              <ResultCard label="Mean Raw (mm)" value={`${lastPoint.meanMm.toFixed(1)} mm`} />
              <ResultCard label="Std Deviation" value={`± ${lastPoint.stdDevMm.toFixed(1)} mm`} accent={lastPoint.stdDevMm > 30 ? 'var(--accent-orange)' : 'var(--accent-green)'} />
              <ResultCard label="Residual Delta" value={`${(lastPoint.offsetM * 1000).toFixed(1)} mm`} accent={Math.abs(lastPoint.offsetM) > 0.05 ? 'var(--accent-orange)' : 'var(--text-main)'} />
              <ResultCard label="Current Active Offset" value={selectedCurrentOffsetM === null ? 'Unavailable' : `${selectedCurrentOffsetM.toFixed(6)} m`} />
              <ResultCard label="Point-Proposed Offset" value={selectedCurrentOffsetM === null ? 'Unavailable' : `${(selectedCurrentOffsetM + lastPoint.offsetM).toFixed(6)} m`} accent="var(--color-primary)" />
            </div>
            <div style={{ display: 'flex', gap: '0.5rem', flexWrap: 'wrap' }}>
              <button className="btn" style={{ background: 'var(--accent-blue)', borderColor: 'var(--accent-blue)', fontSize: '0.8rem' }}
                onClick={() => setStep(2)}>
                + Add Another Distance (same anchor)
              </button>
              <button className="btn" style={{ background: 'var(--accent-green)', borderColor: 'var(--accent-green)', fontSize: '0.8rem' }}
                onClick={() => setStep(5)}>
                View Summary →
              </button>
            </div>
          </StepPanel>
        )}

        {/* Step 5: Multi-point summary & warning */}
        {step === 5 && (
          <StepPanel title="Step 5: Multi-Point Summary" icon="📋">
            {offsetWarning && (
              <div style={{ display: 'flex', gap: '0.5rem', padding: '0.5rem 0.75rem', background: 'rgba(245,158,11,0.1)', border: '1px solid rgba(245,158,11,0.3)', borderRadius: 8, marginBottom: '0.75rem' }}>
                <AlertTriangle size={16} color="var(--accent-orange)" style={{ flexShrink: 0 }} />
                <span style={{ fontSize: '0.8rem', color: 'var(--accent-orange)' }}>
                  Offset varies more than 5cm between distances. This may indicate antenna delay needs re-tuning (not just residual offset).
                </span>
              </div>
            )}
            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(130px, 1fr))', gap: '0.5rem', marginBottom: '0.75rem' }}>
              <ResultCard label="Current Active" value={selectedCurrentOffsetM === null ? 'Unavailable' : `${selectedCurrentOffsetM.toFixed(6)} m`} />
              <ResultCard label="Mean Delta" value={currentDeltaM === null ? 'Unavailable' : `${currentDeltaM >= 0 ? '+' : ''}${currentDeltaM.toFixed(6)} m`} />
              <ResultCard label="Proposed" value={currentProposedOffsetM === null ? 'Unavailable' : `${currentProposedOffsetM.toFixed(6)} m`} accent="var(--color-primary)" />
            </div>
            <p style={hintStyle}>Proposed = current active offset + mean measured residual. Firmware subtracts this target offset from the uncorrected range.</p>
            <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.8rem', marginBottom: '0.75rem' }}>
              <thead>
                <tr style={{ color: 'var(--text-muted)', borderBottom: '1px solid rgba(255,255,255,0.1)' }}>
                  <th style={{ textAlign: 'left', padding: '4px 8px' }}>Distance (m)</th>
                  <th style={{ textAlign: 'right', padding: '4px 8px' }}>Mean (mm)</th>
                  <th style={{ textAlign: 'right', padding: '4px 8px' }}>Std (mm)</th>
                  <th style={{ textAlign: 'right', padding: '4px 8px' }}>Delta (mm)</th>
                  <th style={{ textAlign: 'right', padding: '4px 8px' }}>Samples</th>
                </tr>
              </thead>
              <tbody>
                {currentAnchorPoints.map((pt, i) => (
                  <tr key={i} style={{ borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                    <td style={{ padding: '4px 8px' }}>{pt.trueDistanceM.toFixed(3)}</td>
                    <td style={{ padding: '4px 8px', textAlign: 'right' }}>{pt.meanMm.toFixed(1)}</td>
                    <td style={{ padding: '4px 8px', textAlign: 'right', color: pt.stdDevMm > 30 ? 'var(--accent-orange)' : 'inherit' }}>{pt.stdDevMm.toFixed(1)}</td>
                    <td style={{ padding: '4px 8px', textAlign: 'right', color: Math.abs(pt.offsetM) > 0.05 ? 'var(--accent-orange)' : 'var(--accent-green)' }}>{(pt.offsetM * 1000).toFixed(1)}</td>
                    <td style={{ padding: '4px 8px', textAlign: 'right', color: 'var(--text-muted)' }}>{pt.samples.length}</td>
                  </tr>
                ))}
              </tbody>
            </table>
            <div style={{ display: 'flex', gap: '0.5rem', flexWrap: 'wrap', marginBottom: '1rem' }}>
              <BackBtn onClick={() => setStep(4)} />
              <NextBtn label="Done with A" onClick={finishAnchor} />
            </div>

            <div style={{ paddingTop: '0.85rem', borderTop: '1px solid rgba(255,255,255,0.08)', marginBottom: '1rem' }}>
              <div style={{ fontWeight: 600, fontSize: '0.85rem', marginBottom: '0.5rem', color: 'var(--accent-blue)', display: 'flex', alignItems: 'center', gap: '0.4rem' }}>
                <CheckCircle2 size={15} /> Held-out Static Accuracy
              </div>
              {staticHoldoutResult.verdict === 'insufficient-data' ? (
                <p style={{ fontSize: '0.78rem', color: 'var(--text-muted)' }}>{staticHoldoutResult.verdictText}</p>
              ) : (
                <>
                  <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(135px, 1fr))', gap: '0.5rem', marginBottom: '0.6rem' }}>
                    <ResultCard label="LODO p95 bias" value={`${staticHoldoutResult.p95AbsHoldoutBiasMm?.toFixed(1) ?? '—'} mm`} accent={staticHoldoutResult.verdict === 'pass' ? 'var(--accent-green)' : 'var(--accent-orange)'} />
                    <ResultCard label="Worst held-out" value={`${staticHoldoutResult.maxAbsHoldoutBiasMm?.toFixed(1) ?? '—'} mm`} />
                    <ResultCard label="Median raw std" value={`${staticHoldoutResult.medianWithinCaptureStdMm?.toFixed(1) ?? '—'} mm`} />
                  </div>
                  <div style={{
                    padding: '0.6rem 0.75rem', borderRadius: 8, fontSize: '0.78rem', lineHeight: 1.5,
                    background: staticHoldoutResult.verdict === 'pass' ? 'rgba(16,185,129,0.12)' : 'rgba(245,158,11,0.12)',
                    border: `1px solid ${staticHoldoutResult.verdict === 'pass' ? 'rgba(16,185,129,0.35)' : 'rgba(245,158,11,0.35)'}`,
                    color: staticHoldoutResult.verdict === 'pass' ? 'var(--accent-green)' : 'var(--accent-orange)',
                  }}>
                    {staticHoldoutResult.verdictText}
                  </div>
                </>
              )}
            </div>

            {/* FPP Bias Check — thay cho tools/analyze_fpp_bias.py, chạy ngay trên dữ liệu vừa thu */}
            <div style={{ paddingTop: '0.85rem', borderTop: '1px solid rgba(255,255,255,0.08)' }}>
              <div style={{ fontWeight: 600, fontSize: '0.85rem', marginBottom: '0.5rem', color: 'var(--accent-blue)', display: 'flex', alignItems: 'center', gap: '0.4rem' }}>
                <FlaskConical size={15} /> FPP Bias Check
              </div>

              {fppBiasResult.verdict === 'insufficient-data' ? (
                <p style={{ fontSize: '0.78rem', color: 'var(--text-muted)' }}>{fppBiasResult.verdictText}</p>
              ) : (
                <>
                  <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.78rem', marginBottom: '0.6rem' }}>
                    <thead>
                      <tr style={{ color: 'var(--text-muted)', borderBottom: '1px solid rgba(255,255,255,0.1)' }}>
                        <th style={{ textAlign: 'left', padding: '4px 8px' }}>Distance (m)</th>
                        <th style={{ textAlign: 'right', padding: '4px 8px' }}>Mean FPP (dBm)</th>
                        <th style={{ textAlign: 'right', padding: '4px 8px' }}>Offset (mm)</th>
                        <th style={{ textAlign: 'right', padding: '4px 8px' }}>n</th>
                      </tr>
                    </thead>
                    <tbody>
                      {fppBiasResult.perDistance.map((p, i) => (
                        <tr key={i} style={{ borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                          <td style={{ padding: '4px 8px' }}>{p.trueDistanceM.toFixed(3)}</td>
                          <td style={{ padding: '4px 8px', textAlign: 'right' }}>{p.meanFppDbm.toFixed(2)}</td>
                          <td style={{ padding: '4px 8px', textAlign: 'right' }}>{(p.offsetM * 1000).toFixed(1)}</td>
                          <td style={{ padding: '4px 8px', textAlign: 'right', color: 'var(--text-muted)' }}>{p.n}</td>
                        </tr>
                      ))}
                    </tbody>
                  </table>

                  <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: '0.4rem 1rem', marginBottom: '0.6rem', fontSize: '0.78rem', color: 'var(--text-muted)' }}>
                    <div>r (across-distance): <b style={{ color: 'var(--text-main)' }}>{fppBiasResult.rAcrossDistance?.toFixed(3) ?? '—'}</b></div>
                    <div>r (within-capture): <b style={{ color: 'var(--text-main)' }}>{fppBiasResult.rWithinCapture?.toFixed(3) ?? '—'}</b></div>
                    {fppBiasResult.withinSlopeMmPerDb !== null && (
                      <div style={{ gridColumn: '1 / -1' }}>
                        Hồi quy: residual_mm ≈ {fppBiasResult.withinSlopeMmPerDb.toFixed(2)} × (fpp − mean_fpp_cự_ly)
                      </div>
                    )}
                  </div>

                  <div style={{
                    padding: '0.6rem 0.75rem', borderRadius: 8, fontSize: '0.8rem', lineHeight: 1.5,
                    background: fppBiasResult.verdict === 'confirmed' ? 'rgba(16,185,129,0.12)'
                              : fppBiasResult.verdict === 'inconclusive' ? 'rgba(245,158,11,0.12)'
                              : 'rgba(148,163,184,0.12)',
                    border: `1px solid ${fppBiasResult.verdict === 'confirmed' ? 'rgba(16,185,129,0.35)'
                              : fppBiasResult.verdict === 'inconclusive' ? 'rgba(245,158,11,0.35)'
                              : 'rgba(148,163,184,0.3)'}`,
                    color: fppBiasResult.verdict === 'confirmed' ? 'var(--accent-green)'
                         : fppBiasResult.verdict === 'inconclusive' ? 'var(--accent-orange)'
                         : 'var(--text-muted)',
                  }}>
                    {fppBiasResult.verdictText}
                  </div>
                </>
              )}
            </div>
          </StepPanel>
        )}

        {/* Step 6: Next anchor or finish */}
        {step === 6 && (
          <StepPanel title="Step 6: Continue or Finish" icon="🔁">
            <p style={hintStyle}>Calibrated anchors: {[...new Set(calibrations.map(c => c.anchorId))].map(id => `A${id}`).join(', ')}</p>
            {resultBlockReason && (
              <div style={{ display: 'flex', gap: 8, padding: '8px 12px', background: 'rgba(220,38,38,0.08)', border: '1px solid rgba(220,38,38,0.3)', borderRadius: 8, color: 'var(--color-danger)', fontSize: '0.8rem' }}>
                <AlertTriangle size={15} style={{ flexShrink: 0 }} /> {resultBlockReason} Result generation is locked.
              </div>
            )}
            <div style={{ display: 'flex', gap: '0.75rem', margin: '1rem 0', flexWrap: 'wrap' }}>
              {SUPPORTED_ANCHOR_IDS.filter(id => {
                const done = calibrations.map(c => c.anchorId);
                return id !== selectedAnchorId && !done.includes(id);
              }).map(id => (
                <button key={id} className="btn" style={{ flex: 1, background: 'var(--accent-blue)', borderColor: 'var(--accent-blue)' }}
                  onClick={() => startNextAnchor(id)}>
                  Calibrate A{id} next
                </button>
              ))}
              <button className="btn" style={{ flex: 1, background: 'var(--color-success)', borderColor: 'var(--color-success)', opacity: resultBlockReason ? 0.45 : 1 }}
                onClick={generateResult} disabled={Boolean(resultBlockReason)}>
                <CheckCircle2 size={16} /> Generate Result
              </button>
            </div>
          </StepPanel>
        )}

        {/* Step 7: Final result */}
        {step === 7 && (
          <StepPanel title="Step 7: Calibration Complete!" icon="✅">
            <p style={hintStyle}>
              {resultFirmwareInfo?.calibrationProfile === 'ds'
                ? <>Replace the listed macros in <code>uwb_calibration.h</code>, validate them, then enable only the measured anchor bits.</>
                : <>Replace the matching array declaration in <code>tag_ranging.c</code>, validate it, then enable the measured anchor bits in the SS calibration mask. Unmeasured anchors retain the offsets reported by firmware.</>}
            </p>
            <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.78rem', marginBottom: '0.75rem' }}>
              <thead>
                <tr style={{ color: 'var(--color-text-muted)', borderBottom: '1px solid var(--color-border)' }}>
                  <th style={{ textAlign: 'left', padding: '5px 8px' }}>Anchor</th>
                  <th style={{ textAlign: 'right', padding: '5px 8px' }}>Current (m)</th>
                  <th style={{ textAlign: 'right', padding: '5px 8px' }}>Delta (m)</th>
                  <th style={{ textAlign: 'right', padding: '5px 8px' }}>Proposed (m)</th>
                </tr>
              </thead>
              <tbody>
                {calibrationResults.map(result => (
                  <tr key={result.anchorId} style={{ borderBottom: '1px solid var(--color-border-subtle)', opacity: result.measured ? 1 : 0.65 }}>
                    <td style={{ padding: '5px 8px' }}>A{result.anchorId}{result.measured ? '' : ' (unchanged)'}</td>
                    <td style={{ padding: '5px 8px', textAlign: 'right' }}>{result.currentOffsetM.toFixed(6)}</td>
                    <td style={{ padding: '5px 8px', textAlign: 'right' }}>{result.measured && result.deltaM >= 0 ? '+' : ''}{result.deltaM.toFixed(6)}</td>
                    <td style={{ padding: '5px 8px', textAlign: 'right', fontWeight: result.measured ? 700 : 400 }}>{result.proposedOffsetM.toFixed(6)}</td>
                  </tr>
                ))}
              </tbody>
            </table>
            <pre style={{ fontFamily: 'var(--font-mono)', fontSize: '0.78rem', background: 'var(--color-surface-2)', border: '1px solid var(--color-border)', padding: '0.75rem', borderRadius: 8, marginBottom: '0.75rem', color: 'var(--color-success)', whiteSpace: 'pre-wrap', overflowX: 'auto' }}>
              {snippet}
            </pre>
            <div style={{ display: 'flex', gap: '0.5rem', flexWrap: 'wrap' }}>
              <button className="btn" style={{ background: 'var(--accent-blue)', borderColor: 'var(--accent-blue)' }} onClick={copySnippet}>
                {copied ? <><Check size={14} /> Copied!</> : <><Copy size={14} /> Copy Code</>}
              </button>
              <button className="btn" style={{ background: 'var(--accent-orange)', borderColor: 'var(--accent-orange)' }} onClick={exportCalibCsv}>
                <Download size={14} /> Export Calib CSV
              </button>
              <button className="btn btn-danger" onClick={() => { setStep(1); setCalibrations([]); setCurrentAnchorPoints([]); setLastPoint(null); setSnippet(''); setCalibrationResults([]); setResultFirmwareInfo(null); }}>
                Start Over
              </button>
            </div>
          </StepPanel>
        )}
      </div>
    </div>
  );
}

// ---- Sub-components ----

function StepperBar({ current, total }: { current: number; total: number }) {
  return (
    <div style={{ display: 'flex', gap: 4 }}>
      {Array.from({ length: total }, (_, i) => i + 1).map(n => (
        <div key={n} style={{
          flex: 1, height: 4, borderRadius: 2,
          background: n < current ? 'var(--accent-green)' : n === current ? 'var(--accent-blue)' : 'rgba(255,255,255,0.1)',
          transition: 'background 0.3s',
        }} />
      ))}
    </div>
  );
}

function StepPanel({ title, icon, children }: { title: string; icon: string; children: React.ReactNode }) {
  return (
    <div style={{ padding: '0.25rem' }}>
      <h3 style={{ fontSize: '1.1rem', fontWeight: 600, marginBottom: '0.75rem', display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
        <span>{icon}</span> {title}
      </h3>
      {children}
    </div>
  );
}

function ResultCard({ label, value, accent }: { label: string; value: string; accent?: string }) {
  return (
    <div style={{ background: 'rgba(0,0,0,0.15)', borderRadius: 8, padding: '0.6rem 0.8rem' }}>
      <div style={{ fontSize: '0.7rem', color: 'var(--text-muted)', marginBottom: 2 }}>{label}</div>
      <div style={{ fontSize: '1.1rem', fontWeight: 700, color: accent ?? 'var(--text-main)' }}>{value}</div>
    </div>
  );
}

function NextBtn({ onClick, disabled, label }: { onClick: () => void; disabled?: boolean; label?: string }) {
  return (
    <button className="btn" style={{ background: 'var(--accent-blue)', borderColor: 'var(--accent-blue)', marginTop: '0.5rem' }}
      onClick={onClick} disabled={disabled}>
      {label ?? 'Next'} <ChevronRight size={16} />
    </button>
  );
}

function BackBtn({ onClick }: { onClick: () => void }) {
  return (
    <button className="btn btn-danger" style={{ marginTop: '0.5rem' }} onClick={onClick}>
      <ChevronLeft size={16} /> Back
    </button>
  );
}

const hintStyle: React.CSSProperties = { fontSize: '0.82rem', color: 'var(--text-muted)', marginBottom: '0.5rem', lineHeight: 1.5 };
