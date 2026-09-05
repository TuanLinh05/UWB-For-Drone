import { createHash } from 'node:crypto';
import { createServer } from 'node:http';

function textFrame(value) {
  const payload = Buffer.from(JSON.stringify(value), 'utf8');
  if (payload.length < 126) {
    return Buffer.concat([Buffer.from([0x81, payload.length]), payload]);
  }
  const header = Buffer.alloc(4);
  header[0] = 0x81;
  header[1] = 126;
  header.writeUInt16BE(payload.length, 2);
  return Buffer.concat([header, payload]);
}

const server = createServer();

server.on('upgrade', (request, socket) => {
  const accept = createHash('sha1')
    .update(`${request.headers['sec-websocket-key']}258EAFA5-E914-47DA-95CA-C5AB0DC85B11`)
    .digest('base64');

  socket.write([
    'HTTP/1.1 101 Switching Protocols',
    'Upgrade: websocket',
    'Connection: Upgrade',
    `Sec-WebSocket-Accept: ${accept}`,
    '',
    '',
  ].join('\r\n'));

  const send = value => socket.write(textFrame(value));
  send({
    t: 'i',
    schema: 1,
    flags: 2,
    ranging_mode: 0,
    ds_calibrated_mask: 0,
    filter_mode: 0,
    offsets: [
      { id: 1, active_offset_um: 156_728_437 },
      { id: 2, active_offset_um: 156_461_800 },
      { id: 3, active_offset_um: 155_995_800 },
      { id: 4, active_offset_um: 0 },
    ],
  });
  send({
    t: 's',
    poll_sent: 200,
    response_ok: 159,
    rx_timeout: 41,
    rx_error: 0,
    cycle_overrun: 0,
    uart_overflow: 0,
    cyc_hz: 50,
    ops_hz: 159,
    uart_gap_count: 0,
    crc_error_count: 0,
  });

  let sequence = 0;
  const timer = setInterval(() => {
    sequence += 1;
    const a1Lost = sequence % 5 === 0;
    const a3Lost = sequence % 12 >= 7 && sequence % 12 <= 10;
    const a3Age = a3Lost ? (sequence % 12 - 6) * 20 : 5;
    const a4Diagnostic = sequence % 4 !== 0;

    send({
      t: 'r',
      source_seq: sequence,
      ws_seq: sequence,
      time_ms: sequence * 20,
      anchors: [
        a1Lost
          ? { id: 1, valid: false, status: 1, age_ms: 20, raw_mm: 0, filt_mm: 410, fpp_dbm: 0 }
          : { id: 1, valid: true, status: 0, age_ms: 5, raw_mm: 420, filt_mm: 410, fpp_dbm: -76.2 },
        { id: 2, valid: true, status: 0, age_ms: 5, raw_mm: 620, filt_mm: 610, fpp_dbm: -78.4 },
        a3Lost
          ? { id: 3, valid: false, status: 1, age_ms: a3Age, raw_mm: 0, filt_mm: 805, fpp_dbm: 0 }
          : { id: 3, valid: true, status: 0, age_ms: 5, raw_mm: 820, filt_mm: 805, fpp_dbm: -80.1 },
        a4Diagnostic
          ? {
              id: 4,
              valid: false,
              status: 0x20,
              age_ms: 65535,
              raw_mm: 158_500 + (sequence % 5),
              filt_mm: 0,
              fpp_dbm: -72.3,
            }
          : { id: 4, valid: false, status: 1, age_ms: 65535, raw_mm: 0, filt_mm: 0, fpp_dbm: 0 },
      ],
    });
  }, 20);

  socket.on('close', () => clearInterval(timer));
  socket.on('error', () => clearInterval(timer));
});

server.listen(81, '127.0.0.1', () => {
  console.log('Mock intermittent telemetry: ws://127.0.0.1:81');
});
