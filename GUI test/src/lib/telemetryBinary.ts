import type { AnchorSample, StatsSample } from './types';

/** Wire constants shared with STM32_UWB/TAG/Core/Inc/telemetry.h. */
export const TELEMETRY_BINARY_VERSION = 1;
export const TELEMETRY_BINARY_INFO_TYPE = 0x00;
export const TELEMETRY_BINARY_RANGE_TYPE = 0x01;
export const TELEMETRY_BINARY_STATS_TYPE = 0x02;

const SOF_1 = 0xaa;
const SOF_2 = 0x55;
const HEADER_LENGTH = 14; // SOF(2) + VER..TIME(12)
const CRC_LENGTH = 2;
const RANGE_RECORD_LENGTH = 16;
const STATS_PAYLOAD_LENGTH = 28;
const INFO_V1_HEADER_LENGTH = 8;
const INFO_V2_HEADER_LENGTH = 10;
const INFO_OFFSET_RECORD_LENGTH = 6;
const DEFAULT_MAX_PAYLOAD_LENGTH = 80; // STM32 TELEM_MAX_PKT(96) - header(14) - CRC(2)

export interface BinaryRangePacket {
  kind: 'range';
  version: number;
  seq: number;
  timeMs: number;
  anchors: AnchorSample[];
}

export interface BinaryStatsPacket {
  kind: 'stats';
  version: number;
  seq: number;
  timeMs: number;
  stats: StatsSample;
}

export interface BinaryInfoAnchorOffset {
  id: number;
  activeOffsetUm: number;
}

export interface BinaryInfoPacket {
  kind: 'info';
  version: number;
  seq: number;
  timeMs: number;
  schemaVersion: number;
  flags: number;
  /** Firmware value: 0 = SS-TWR, 1 = DS-TWR. */
  rangingMode: number;
  anchorCount: number;
  dsCalibratedMask: number;
  filterMode: number;
  phyProfileId: number;
  spiClockMhz: number;
  c9_2MotionMode?: number;
  c9_2GlobalMotionState?: number;
  activeOffsets: BinaryInfoAnchorOffset[];
}

export type TelemetryBinaryPacket = BinaryInfoPacket | BinaryRangePacket | BinaryStatsPacket;

export interface TelemetryBinaryDiagnostics {
  /** Frames whose SOF and complete declared length were observed. */
  framesSeen: number;
  /** Valid range/stats frames returned to the caller. */
  framesDecoded: number;
  crcErrors: number;
  versionErrors: number;
  lengthErrors: number;
  unknownTypeFrames: number;
  /** Bytes skipped while finding the next valid frame boundary. */
  discardedBytes: number;
}

export interface TelemetryBinaryParserOptions {
  /** Reject larger payloads before allocating/waiting for the complete frame. */
  maxPayloadLength?: number;
}

function emptyDiagnostics(): TelemetryBinaryDiagnostics {
  return {
    framesSeen: 0,
    framesDecoded: 0,
    crcErrors: 0,
    versionErrors: 0,
    lengthErrors: 0,
    unknownTypeFrames: 0,
    discardedBytes: 0,
  };
}

/** CRC-16/CCITT-FALSE: poly 0x1021, init 0xffff, no reflection/xorout. */
export function telemetryCrc16Ccitt(data: Uint8Array): number {
  let crc = 0xffff;
  for (const byte of data) {
    crc ^= byte << 8;
    for (let bit = 0; bit < 8; bit++) {
      crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
    }
  }
  return crc;
}

function copyChunk(chunk: Uint8Array | ArrayBuffer): Uint8Array {
  if (chunk instanceof Uint8Array) return chunk;
  return new Uint8Array(chunk);
}

function findSof(data: Uint8Array, from: number): number {
  for (let i = from; i + 1 < data.length; i++) {
    if (data[i] === SOF_1 && data[i + 1] === SOF_2) return i;
  }
  return -1;
}

function u32(view: DataView, offset: number): number {
  return view.getUint32(offset, true);
}

function decodeRange(
  view: DataView,
  payloadOffset: number,
  payloadLength: number,
  version: number,
  seq: number,
  timeMs: number,
): BinaryRangePacket | null {
  if (payloadLength < 1) return null;

  const count = view.getUint8(payloadOffset);
  if (payloadLength !== 1 + count * RANGE_RECORD_LENGTH) return null;

  const anchors: AnchorSample[] = [];
  let offset = payloadOffset + 1;
  for (let i = 0; i < count; i++) {
    anchors.push({
      id: view.getUint16(offset, true),
      valid: view.getUint8(offset + 2) !== 0,
      status: view.getUint8(offset + 3),
      ageMs: view.getUint16(offset + 4, true),
      rawMm: view.getInt32(offset + 6, true),
      filtMm: view.getInt32(offset + 10, true),
      fppDbm: view.getInt16(offset + 14, true) / 100,
    });
    offset += RANGE_RECORD_LENGTH;
  }

  return { kind: 'range', version, seq, timeMs, anchors };
}

function decodeInfo(
  view: DataView,
  payloadOffset: number,
  payloadLength: number,
  version: number,
  seq: number,
  timeMs: number,
): BinaryInfoPacket | null {
  if (payloadLength < INFO_V1_HEADER_LENGTH) return null;

  const schemaVersion = view.getUint8(payloadOffset);
  const infoHeaderLength = schemaVersion === 1
    ? INFO_V1_HEADER_LENGTH
    : schemaVersion === 2
      ? INFO_V2_HEADER_LENGTH
      : 0;
  if (infoHeaderLength === 0 || payloadLength < infoHeaderLength) return null;

  const anchorCount = view.getUint8(payloadOffset + 3);
  if (payloadLength !== infoHeaderLength + anchorCount * INFO_OFFSET_RECORD_LENGTH) return null;

  const activeOffsets: BinaryInfoAnchorOffset[] = [];
  let offset = payloadOffset + infoHeaderLength;
  for (let i = 0; i < anchorCount; i++) {
    activeOffsets.push({
      id: view.getUint16(offset, true),
      activeOffsetUm: view.getInt32(offset + 2, true),
    });
    offset += INFO_OFFSET_RECORD_LENGTH;
  }

  return {
    kind: 'info',
    version,
    seq,
    timeMs,
    schemaVersion,
    flags: view.getUint8(payloadOffset + 1),
    rangingMode: view.getUint8(payloadOffset + 2),
    anchorCount,
    dsCalibratedMask: view.getUint8(payloadOffset + 4),
    filterMode: view.getUint8(payloadOffset + 5),
    phyProfileId: view.getUint8(payloadOffset + 6),
    spiClockMhz: view.getUint8(payloadOffset + 7),
    ...(schemaVersion >= 2
      ? {
          c9_2MotionMode: view.getUint8(payloadOffset + 8),
          c9_2GlobalMotionState: view.getUint8(payloadOffset + 9),
        }
      : {}),
    activeOffsets,
  };
}

function decodeStats(
  view: DataView,
  payloadOffset: number,
  payloadLength: number,
  version: number,
  seq: number,
  timeMs: number,
): BinaryStatsPacket | null {
  if (payloadLength !== STATS_PAYLOAD_LENGTH) return null;

  return {
    kind: 'stats',
    version,
    seq,
    timeMs,
    stats: {
      pollSent: u32(view, payloadOffset),
      responseOk: u32(view, payloadOffset + 4),
      rxTimeout: u32(view, payloadOffset + 8),
      rxError: u32(view, payloadOffset + 12),
      cycleOverrun: u32(view, payloadOffset + 16),
      uartOverflow: u32(view, payloadOffset + 20),
      cycHz: view.getUint16(payloadOffset + 24, true),
      opsHz: view.getUint16(payloadOffset + 26, true),
    },
  };
}

/**
 * Stateful streaming decoder for the STM32 telemetry protocol.
 *
 * `push()` accepts arbitrary Web Serial chunks: a frame may be split across
 * calls, several frames may arrive in one call, and noise/corrupt frames are
 * skipped until the next AA 55 marker. Diagnostics remain cumulative until
 * `reset()` or `resetDiagnostics()` is called.
 */
export class TelemetryBinaryParser {
  private pending = new Uint8Array(0);
  private readonly maxPayloadLength: number;
  private counters = emptyDiagnostics();

  constructor(options: TelemetryBinaryParserOptions = {}) {
    this.maxPayloadLength = options.maxPayloadLength ?? DEFAULT_MAX_PAYLOAD_LENGTH;
    if (!Number.isInteger(this.maxPayloadLength) || this.maxPayloadLength < 0 || this.maxPayloadLength > 0xffff) {
      throw new RangeError('maxPayloadLength must be an integer between 0 and 65535');
    }
  }

  get diagnostics(): Readonly<TelemetryBinaryDiagnostics> {
    return { ...this.counters };
  }

  get bufferedByteCount(): number {
    return this.pending.length;
  }

  reset(): void {
    this.pending = new Uint8Array(0);
    this.counters = emptyDiagnostics();
  }

  resetDiagnostics(): void {
    this.counters = emptyDiagnostics();
  }

  push(chunk: Uint8Array | ArrayBuffer): TelemetryBinaryPacket[] {
    const bytes = copyChunk(chunk);
    if (bytes.length === 0) return [];

    const joined = new Uint8Array(this.pending.length + bytes.length);
    joined.set(this.pending);
    joined.set(bytes, this.pending.length);
    this.pending = joined;

    const packets: TelemetryBinaryPacket[] = [];
    let cursor = 0;

    while (true) {
      const sof = findSof(this.pending, cursor);
      if (sof < 0) {
        // Keep a final AA because it may be the first half of a split SOF.
        const keep = this.pending.length > cursor && this.pending[this.pending.length - 1] === SOF_1 ? 1 : 0;
        this.counters.discardedBytes += this.pending.length - cursor - keep;
        this.pending = keep ? this.pending.slice(-1) : new Uint8Array(0);
        return packets;
      }

      this.counters.discardedBytes += sof - cursor;
      if (this.pending.length - sof < HEADER_LENGTH) {
        this.pending = this.pending.slice(sof);
        return packets;
      }

      const header = new DataView(this.pending.buffer, this.pending.byteOffset + sof, this.pending.length - sof);
      const version = header.getUint8(2);
      const type = header.getUint8(3);
      const payloadLength = header.getUint16(4, true);

      if (version !== TELEMETRY_BINARY_VERSION) {
        this.counters.versionErrors++;
        this.counters.discardedBytes++;
        cursor = sof + 1;
        continue;
      }

      if (!this.isPlausibleLength(type, payloadLength)) {
        this.counters.lengthErrors++;
        this.counters.discardedBytes++;
        cursor = sof + 1;
        continue;
      }

      const frameLength = HEADER_LENGTH + payloadLength + CRC_LENGTH;
      if (this.pending.length - sof < frameLength) {
        this.pending = this.pending.slice(sof);
        return packets;
      }

      this.counters.framesSeen++;
      const receivedCrc = header.getUint16(HEADER_LENGTH + payloadLength, true);
      const crcData = this.pending.subarray(sof + 2, sof + HEADER_LENGTH + payloadLength);
      if (receivedCrc !== telemetryCrc16Ccitt(crcData)) {
        this.counters.crcErrors++;
        this.counters.discardedBytes++;
        cursor = sof + 1;
        continue;
      }

      const seq = header.getUint32(6, true);
      const timeMs = header.getUint32(10, true);
      const payloadOffset = HEADER_LENGTH;
      let packet: TelemetryBinaryPacket | null = null;

      if (type === TELEMETRY_BINARY_INFO_TYPE) {
        packet = decodeInfo(header, payloadOffset, payloadLength, version, seq, timeMs);
      } else if (type === TELEMETRY_BINARY_RANGE_TYPE) {
        packet = decodeRange(header, payloadOffset, payloadLength, version, seq, timeMs);
      } else if (type === TELEMETRY_BINARY_STATS_TYPE) {
        packet = decodeStats(header, payloadOffset, payloadLength, version, seq, timeMs);
      } else {
        this.counters.unknownTypeFrames++;
      }

      if (packet) {
        packets.push(packet);
        this.counters.framesDecoded++;
      } else if (
        type === TELEMETRY_BINARY_INFO_TYPE
        || type === TELEMETRY_BINARY_RANGE_TYPE
        || type === TELEMETRY_BINARY_STATS_TYPE
      ) {
        // Payload-internal length/count mismatch; CRC was valid but decoding is unsafe.
        this.counters.lengthErrors++;
      }

      cursor = sof + frameLength;
      if (cursor === this.pending.length) {
        this.pending = new Uint8Array(0);
        return packets;
      }
    }
  }

  private isPlausibleLength(type: number, payloadLength: number): boolean {
    if (payloadLength > this.maxPayloadLength) return false;
    if (type === TELEMETRY_BINARY_INFO_TYPE) {
      /* Schema is inside payload. CRC + decodeInfo validate the exact layout
       * once the complete frame is available; this precheck must admit both
       * the old 8-byte and new 10-byte INFO headers. */
      return payloadLength >= INFO_V1_HEADER_LENGTH;
    }
    if (type === TELEMETRY_BINARY_RANGE_TYPE) {
      return payloadLength >= 1 && (payloadLength - 1) % RANGE_RECORD_LENGTH === 0;
    }
    if (type === TELEMETRY_BINARY_STATS_TYPE) return payloadLength === STATS_PAYLOAD_LENGTH;
    return true;
  }
}
