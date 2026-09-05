import assert from 'node:assert/strict';
import { mkdtemp, readFile, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import ts from 'typescript';

const scriptDir = dirname(fileURLToPath(import.meta.url));
const tempDir = await mkdtemp(join(tmpdir(), 'uwb-telemetry-binary-'));

function concat(...parts) {
  const output = new Uint8Array(parts.reduce((length, part) => length + part.length, 0));
  let offset = 0;
  for (const part of parts) {
    output.set(part, offset);
    offset += part.length;
  }
  return output;
}

try {
  const parserSourcePath = join(scriptDir, '..', 'src', 'lib', 'telemetryBinary.ts');
  const bundledParser = join(tempDir, 'telemetryBinary.mjs');
  const source = await readFile(parserSourcePath, 'utf8');
  const transpiled = ts.transpileModule(source, {
    fileName: parserSourcePath,
    reportDiagnostics: true,
    compilerOptions: {
      target: ts.ScriptTarget.ES2020,
      module: ts.ModuleKind.ES2020,
    },
  });
  assert.equal(transpiled.diagnostics?.length ?? 0, 0, 'parser has no transpile diagnostics');
  await writeFile(bundledParser, transpiled.outputText, 'utf8');

  const { TelemetryBinaryParser, telemetryCrc16Ccitt } = await import(pathToFileURL(bundledParser).href);

  function frame(type, seq, timeMs, payload, version = 1) {
    const output = new Uint8Array(14 + payload.length + 2);
    const view = new DataView(output.buffer);
    output[0] = 0xaa;
    output[1] = 0x55;
    output[2] = version;
    output[3] = type;
    view.setUint16(4, payload.length, true);
    view.setUint32(6, seq, true);
    view.setUint32(10, timeMs, true);
    output.set(payload, 14);
    view.setUint16(14 + payload.length, telemetryCrc16Ccitt(output.subarray(2, 14 + payload.length)), true);
    return output;
  }

  function rangePayload() {
    const payload = new Uint8Array(1 + 2 * 16);
    const view = new DataView(payload.buffer);
    payload[0] = 2;

    view.setUint16(1, 7, true);
    payload[3] = 1;
    payload[4] = 0x12;
    view.setUint16(5, 25, true);
    view.setInt32(7, -1234, true);
    view.setInt32(11, 5678, true);
    view.setInt16(15, -8333, true);

    view.setUint16(17, 42, true);
    payload[19] = 0;
    payload[20] = 0x80;
    view.setUint16(21, 0xffff, true);
    view.setInt32(23, 2_147_483_647, true);
    view.setInt32(27, -2_147_483_648, true);
    view.setInt16(31, -9_999, true);
    return payload;
  }

  function statsPayload() {
    const payload = new Uint8Array(28);
    const view = new DataView(payload.buffer);
    [4_000_000_001, 4_000_000_002, 3, 4, 5, 6].forEach((value, index) => {
      view.setUint32(index * 4, value, true);
    });
    view.setUint16(24, 50, true);
    view.setUint16(26, 145, true);
    return payload;
  }

  function infoPayload() {
    const payload = new Uint8Array(10 + 2 * 6);
    const view = new DataView(payload.buffer);
    payload.set([2, 0x05, 1, 2, 0x03, 2, 2, 16, 1, 3]);
    view.setUint16(10, 1, true);
    view.setInt32(12, -12_345, true);
    view.setUint16(16, 9, true);
    view.setInt32(18, 67_890, true);
    return payload;
  }

  assert.equal(
    telemetryCrc16Ccitt(new TextEncoder().encode('123456789')),
    0x29b1,
    'CRC-16/CCITT-FALSE check value',
  );

  const range = frame(0x01, 0xffff_fffe, 123_456, rangePayload());
  const stats = frame(0x02, 900, 124_000, statsPayload());
  const info = frame(0x00, 901, 124_001, infoPayload());
  const parser = new TelemetryBinaryParser();
  const stream = concat(new Uint8Array([0x00, 0xaa, 0x01, 0xaa]), range, stats, info);
  const decoded = [];
  const chunkSizes = [1, 2, 7, 3, 19, 1, 31, 4, 2, 99];
  let streamOffset = 0;
  for (const size of chunkSizes) {
    if (streamOffset >= stream.length) break;
    decoded.push(...parser.push(stream.subarray(streamOffset, streamOffset + size)));
    streamOffset += size;
  }
  decoded.push(...parser.push(stream.subarray(streamOffset)));

  assert.equal(decoded.length, 3, 'decodes split and back-to-back frames');
  assert.equal(decoded[0].kind, 'range');
  assert.equal(decoded[0].seq, 0xffff_fffe);
  assert.deepEqual(decoded[0].anchors[0], {
    id: 7,
    valid: true,
    status: 0x12,
    ageMs: 25,
    rawMm: -1234,
    filtMm: 5678,
    fppDbm: -83.33,
  });
  assert.equal(decoded[0].anchors[1].ageMs, 0xffff);
  assert.equal(decoded[0].anchors[1].filtMm, -2_147_483_648);
  assert.equal(decoded[1].kind, 'stats');
  assert.equal(decoded[1].stats.pollSent, 4_000_000_001);
  assert.equal(decoded[1].stats.opsHz, 145);
  assert.equal(decoded[2].kind, 'info');
  assert.equal(decoded[2].rangingMode, 1);
  assert.equal(decoded[2].dsCalibratedMask, 0x03);
  assert.equal(decoded[2].phyProfileId, 2);
  assert.equal(decoded[2].spiClockMhz, 16);
  assert.equal(decoded[2].schemaVersion, 2);
  assert.equal(decoded[2].c9_2MotionMode, 1);
  assert.equal(decoded[2].c9_2GlobalMotionState, 3);
  assert.deepEqual(decoded[2].activeOffsets, [
    { id: 1, activeOffsetUm: -12_345 },
    { id: 9, activeOffsetUm: 67_890 },
  ]);
  assert.equal(parser.bufferedByteCount, 0);

  const corrupt = range.slice();
  corrupt[20] ^= 0x40;
  const recoveryParser = new TelemetryBinaryParser();
  const recovered = recoveryParser.push(concat(corrupt, new Uint8Array([0x13, 0xaa]), stats));
  assert.equal(recovered.length, 1, 'resynchronises after a bad CRC');
  assert.equal(recovered[0].kind, 'stats');
  assert.equal(recoveryParser.diagnostics.crcErrors, 1);

  const validationParser = new TelemetryBinaryParser();
  const unsupportedVersion = frame(0x01, 1, 1, rangePayload(), 2);
  const invalidDeclaredLength = frame(0x01, 2, 2, new Uint8Array([0, 0]));
  const invalidRecordCount = frame(0x01, 3, 3, new Uint8Array(17));
  const unknownType = frame(0x7f, 4, 4, new Uint8Array([0xaa, 0x55, 0x00]));
  const validated = validationParser.push(
    concat(unsupportedVersion, invalidDeclaredLength, invalidRecordCount, unknownType, range),
  );
  assert.equal(validated.length, 1, 'rejects malformed frames and finds the following valid frame');
  assert.equal(validationParser.diagnostics.versionErrors, 1);
  assert.ok(validationParser.diagnostics.lengthErrors >= 2);
  assert.equal(validationParser.diagnostics.unknownTypeFrames, 1);

  console.log('telemetryBinary: all streaming, CRC, validation and resync tests passed');
} finally {
  await rm(tempDir, { recursive: true, force: true });
}
