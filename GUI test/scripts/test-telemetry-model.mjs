import assert from 'node:assert/strict';
import { mkdtemp, readFile, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import ts from 'typescript';

const scriptDir = dirname(fileURLToPath(import.meta.url));
const tempDir = await mkdtemp(join(tmpdir(), 'uwb-telemetry-model-'));

try {
  const sourcePath = join(scriptDir, '..', 'src', 'lib', 'telemetryModel.ts');
  const outputPath = join(tempDir, 'telemetryModel.mjs');
  const source = await readFile(sourcePath, 'utf8');
  const transpiled = ts.transpileModule(source, {
    fileName: sourcePath,
    reportDiagnostics: true,
    compilerOptions: { target: ts.ScriptTarget.ES2020, module: ts.ModuleKind.ES2020 },
  });
  assert.equal(transpiled.diagnostics?.length ?? 0, 0);
  await writeFile(outputPath, transpiled.outputText, 'utf8');

  const {
    canonicalizeRangeSample,
    decodeFirmwareInfo,
    missingTransportPackets,
  } = await import(pathToFileURL(outputPath).href);

  const sample = canonicalizeRangeSample({
    seq: 100,
    transportSeq: 8,
    timeMs: 1234,
    clientTime: 9999,
    anchors: [
      { id: 3, valid: false, status: 0x20, ageMs: 0xffff, rawMm: 3456, filtMm: 3400, fppDbm: -82 },
      { id: 2, valid: false, status: 0x40, ageMs: 25, rawMm: 4321, filtMm: 1200, fppDbm: -79 },
      { id: 4, valid: true, status: 0, ageMs: 10, rawMm: 4567, filtMm: 4500, fppDbm: -74 },
      { id: 1, valid: true, status: 0, ageMs: 0, rawMm: 1234, filtMm: 1200, fppDbm: -70 },
    ],
  });
  assert.ok(sample);
  assert.deepEqual(sample.anchors.map(anchor => anchor.id), [1, 2, 3, 4], 'stable iteration order is by ID');
  assert.equal(sample.anchorsById[1].rawMm, 1234);
  assert.equal(sample.anchorsById[4].filtMm, 4500, 'anchor 4 is canonicalized by ID');
  assert.equal(sample.anchorsById[3].rawMm, null, 'invalid production range is null');
  assert.equal(sample.anchorsById[3].filtMm, null, 'invalid filtered range is null');
  assert.equal(sample.anchorsById[3].diagnosticRawMm, 3456, 'missing-calibration bootstrap data is explicitly diagnostic');
  assert.equal(sample.anchorsById[2].rawMm, null, 'conditioner reject is not canonical data');
  assert.equal(sample.anchorsById[2].diagnosticRawMm, 4321, 'conditioner reject raw evidence remains replayable');
  assert.equal(sample.anchorsById[2].diagnosticFppDbm, -79);

  const duplicate = canonicalizeRangeSample({
    seq: 1,
    timeMs: 1,
    anchors: [
      { id: 2, valid: true, ageMs: 0, rawMm: 1, filtMm: 1, fppDbm: -70 },
      { id: 2, valid: true, ageMs: 0, rawMm: 2, filtMm: 2, fppDbm: -70 },
    ],
  });
  assert.equal(duplicate, null, 'duplicate IDs reject the whole ambiguous sample');

  assert.equal(missingTransportPackets(7, 8), 0);
  assert.equal(missingTransportPackets(7, 10), 2);
  assert.equal(missingTransportPackets(0xffff_ffff, 0), 0, 'uint32 sequence wrap is contiguous');
  assert.equal(missingTransportPackets(10, 9), 1, 'out-of-order is one continuity fault');

  const legacy = decodeFirmwareInfo({
    schemaVersion: 1,
    flags: 0x02,
    rangingMode: 0,
    dsCalibratedMask: 0,
    rangeFilterMode: 0,
    phyProfileId: 2,
    spiClockMhz: 16,
    offsets: [{ id: 1, activeOffsetUm: 156_728_437 }],
    receivedAt: 10,
  });
  assert.equal(legacy?.calibrationProfile, 'legacy');
  assert.equal(legacy?.phyProfile, 'fast-256');
  assert.equal(legacy?.spiClockMhz, 16);
  assert.equal(legacy?.activeOffsetsM[1], 156.728437);
  assert.equal(legacy?.legacyAdaptiveMode, 'off', 'older INFO flags decode as safe OFF');
  assert.equal(legacy?.c9_2MotionMode, 'unknown', 'schema 1 does not claim a C9.2 mode');

  const shadow = decodeFirmwareInfo({
    schemaVersion: 1,
    flags: 0x0a, // legacy offset + Adaptive Legacy SHADOW in INFO bits 3..4
    rangingMode: 0,
    dsCalibratedMask: 0,
    rangeFilterMode: 0,
    offsets: [{ id: 1, activeOffsetUm: 156_728_437 }],
  });
  assert.equal(shadow?.legacyAdaptiveMode, 'shadow');

  const active = decodeFirmwareInfo({
    schemaVersion: 1,
    flags: 0x14, // DS build + Adaptive Legacy ACTIVE in INFO bits 3..4
    rangingMode: 1,
    dsCalibratedMask: 0x0f,
    rangeFilterMode: 0,
    offsets: [{ id: 1, activeOffsetUm: 154_000_000 }],
  });
  assert.equal(active?.legacyAdaptiveMode, 'active');

  const residual = decodeFirmwareInfo({
    schemaVersion: 1,
    flags: 0x01,
    rangingMode: 0,
    dsCalibratedMask: 0,
    rangeFilterMode: 0,
    offsets: [{ id: 1, activeOffsetUm: -12_500 }],
  });
  assert.equal(residual?.calibrationProfile, 'residual-hw');

  const mismatched = decodeFirmwareInfo({
    schemaVersion: 1,
    flags: 0,
    rangingMode: 0,
    dsCalibratedMask: 0,
    rangeFilterMode: 0,
    offsets: [{ id: 1, activeOffsetUm: 0 }],
  });
  assert.equal(mismatched?.calibrationProfile, 'unknown', 'unsafe profile combinations fail closed');

  const doubleCompensation = decodeFirmwareInfo({
    schemaVersion: 1,
    flags: 0x03,
    rangingMode: 0,
    dsCalibratedMask: 0,
    rangeFilterMode: 0,
    offsets: [{ id: 1, activeOffsetUm: 156_000_000 }],
  });
  assert.equal(doubleCompensation?.calibrationProfile, 'unknown', 'HW + legacy double compensation is blocked');

  const c9Motion = decodeFirmwareInfo({
    schemaVersion: 2,
    flags: 0x02,
    rangingMode: 0,
    dsCalibratedMask: 0,
    rangeFilterMode: 0,
    phyProfileId: 2,
    spiClockMhz: 16,
    c9_2MotionMode: 1,
    c9_2GlobalMotionState: 3,
    offsets: [{ id: 1, activeOffsetUm: 156_728_437 }],
  });
  assert.equal(c9Motion?.c9_2MotionMode, 'shadow');
  assert.equal(c9Motion?.c9_2GlobalMotionState, 'fast');

  console.log('telemetryModel: canonical IDs, invalid data, sequences and profiles passed');
} finally {
  await rm(tempDir, { recursive: true, force: true });
}
