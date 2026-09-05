import { useRef, useEffect, useCallback } from 'react';
import type { AnchorPosition } from '../lib/types';
import type { RingBuffer2D } from '../lib/ringBuffer2d';

// ============================================================
// Color palette
// ============================================================
const ANCHOR_COLOR      = 'rgb(59, 130, 246)';
const ANCHOR_GLOW_RGB   = '59,130,246';
const TRILAT_COLOR      = 'rgb(245, 158, 11)';
const KALMAN_COLOR      = 'rgb(16, 185, 129)';
const DRONE_GLOW_RGB    = '16,185,129';
const RANGE_CIRCLE_COLOR = 'rgba(59, 130, 246, 0.15)';
const LEGEND_BG         = 'rgba(255, 255, 255, 0.85)';
const LEGEND_BORDER     = 'rgba(0, 0, 0, 0.1)';
const TEXT_COLOR        = 'rgb(15, 23, 42)';



const MARGIN = 52;

interface Props {
  anchorLayout: AnchorPosition[];
  /** Khoảng cách hiện tại mỗi anchor (mét), null nếu chưa có data */
  distancesM: Readonly<Record<number, number>> | null;
  trilatPoint: { x: number; y: number } | null;
  kalmanPoint: { x: number; y: number; vx: number; vy: number } | null;
  registerFrameCallback: (cb: ((trilat: RingBuffer2D, kalman: RingBuffer2D) => void) | null) => void;
  showRangeCircles: boolean;
  showTrilateration: boolean;
  showKalman: boolean;
  showGrid: boolean;
  gridSpacing: number;
}


/** World-space bounds + view transform.
 *  P1-01: must store minX/minY so toScreen handles translated layouts and negative coords. */
interface Scale {
  scale: number;
  offsetX: number;
  offsetY: number;
  cssW: number;
  cssH: number;
  simW: number;
  simH: number;
  minX: number;
  minY: number;
}

/** Compute uniform scale. Padding in world-meters on each side. */
function computeScale(cssW: number, cssH: number, minX: number, maxX: number, minY: number, maxY: number): Scale {
  const PAD = 2; // meters padding on each side
  const worldW = Math.max(maxX - minX + PAD * 2, 6);
  const worldH = Math.max(maxY - minY + PAD * 2, 6);
  const drawW = cssW - 2 * MARGIN;
  const drawH = cssH - 2 * MARGIN;
  const scale = Math.min(drawW / worldW, drawH / worldH);
  const offsetX = MARGIN + (drawW - worldW * scale) / 2;
  const offsetY = MARGIN + (drawH - worldH * scale) / 2;
  return { scale, offsetX, offsetY, cssW, cssH, simW: worldW, simH: worldH, minX: minX - PAD, minY: minY - PAD };
}

/** P1-01 fixed: use (x - minX) and (y - minY) so layout shifted from origin renders correctly. */
function toScreen(x: number, y: number, sc: Scale): [number, number] {
  return [
    sc.offsetX + (x - sc.minX) * sc.scale,
    sc.cssH - sc.offsetY - (y - sc.minY) * sc.scale,  // flip Y
  ];
}

function metersToPx(m: number, sc: Scale) {
  return m * sc.scale;
}



// ---- Drawing helpers ----

function drawGlow(ctx: CanvasRenderingContext2D, x: number, y: number, radius: number, rgb: string, alpha = 0.35) {
  const g = ctx.createRadialGradient(x, y, 0, x, y, radius);
  g.addColorStop(0, `rgba(${rgb}, ${alpha})`);
  g.addColorStop(1, `rgba(${rgb}, 0)`);
  ctx.fillStyle = g;
  ctx.beginPath();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  ctx.fill();
}

function drawGrid(ctx: CanvasRenderingContext2D, sc: Scale, gridSpacing: number) {
  // Label spacing — reduce label density for fine grids
  const labelStep = gridSpacing < 1 ? 1 : gridSpacing * 2;
  // Grid covers world bounds (minX..minX+simW, minY..minY+simH)
  const worldXStart = sc.minX;
  const worldXEnd = sc.minX + sc.simW;
  const worldYStart = sc.minY;
  const worldYEnd = sc.minY + sc.simH;

  const startX = Math.ceil(worldXStart / gridSpacing) * gridSpacing;
  for (let wx = startX; wx <= worldXEnd + gridSpacing * 0.01; wx += gridSpacing) {
    const [x1, y1] = toScreen(wx, worldYStart, sc);
    const [x2, y2] = toScreen(wx, worldYEnd, sc);
    
    const isAxis = Math.abs(wx) < 1e-4;
    ctx.strokeStyle = isAxis ? 'rgba(0,100,255,0.4)' : 'rgba(0,0,0,0.15)';
    ctx.lineWidth = isAxis ? 1.5 : 1.0;
    ctx.setLineDash(isAxis ? [] : [2, 4]);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    
    const rounded = Math.round(wx * 10) / 10;
    if (Math.abs(rounded % labelStep) < gridSpacing * 0.1) {
      ctx.fillStyle = isAxis ? 'rgba(0,100,255,0.8)' : 'rgba(0,0,0,0.55)';
      ctx.font = isAxis ? 'bold 10px Consolas, monospace' : '10px Consolas, monospace';
      ctx.textAlign = 'center';
      ctx.fillText(Number.isInteger(rounded) ? String(rounded) : rounded.toFixed(1), x1, y1 + 14);
      
      // Draw Y axis label at the top
      if (isAxis) {
        ctx.fillText("Y", x2, y2 - 6);
      }
    }
  }

  const startY = Math.ceil(worldYStart / gridSpacing) * gridSpacing;
  for (let wy = startY; wy <= worldYEnd + gridSpacing * 0.01; wy += gridSpacing) {
    const [x1, y1] = toScreen(worldXStart, wy, sc);
    const [x2, y2] = toScreen(worldXEnd, wy, sc);
    
    const isAxis = Math.abs(wy) < 1e-4;
    ctx.strokeStyle = isAxis ? 'rgba(0,100,255,0.4)' : 'rgba(0,0,0,0.15)';
    ctx.lineWidth = isAxis ? 1.5 : 1.0;
    ctx.setLineDash(isAxis ? [] : [2, 4]);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    
    const rounded = Math.round(wy * 10) / 10;
    if (Math.abs(rounded % labelStep) < gridSpacing * 0.1) {
      ctx.fillStyle = isAxis ? 'rgba(0,100,255,0.8)' : 'rgba(0,0,0,0.55)';
      ctx.font = isAxis ? 'bold 10px Consolas, monospace' : '10px Consolas, monospace';
      ctx.textAlign = 'right';
      // Avoid drawing "0" twice, wx=0 already drew it
      if (!isAxis) {
        ctx.fillText(Number.isInteger(rounded) ? String(rounded) : rounded.toFixed(1), x1 - 6, y1 + 4);
      }
      
      // Draw X axis label at the right
      if (isAxis) {
        ctx.textAlign = 'left';
        ctx.fillText("X", x2 + 6, y2 + 4);
      }
    }
  }
  ctx.setLineDash([]);
}


function drawRangeCircles(
  ctx: CanvasRenderingContext2D,
  anchors: AnchorPosition[],
  distancesByAnchorId: Readonly<Record<number, number>>,
  sc: Scale,
) {
  ctx.strokeStyle = RANGE_CIRCLE_COLOR;
  ctx.lineWidth = 1.2;
  ctx.setLineDash([5, 5]);
  anchors.forEach(a => {
    const distanceM = distancesByAnchorId[a.id];
    if (!Number.isFinite(distanceM) || distanceM <= 0) return;
    const r = metersToPx(distanceM, sc);
    if (r < 1 || r > 9999) return;
    const [cx, cy] = toScreen(a.x, a.y, sc);
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.stroke();
  });
  ctx.setLineDash([]);
}

function drawTrilatTrail(ctx: CanvasRenderingContext2D, buf: RingBuffer2D, sc: Scale) {
  buf.forEach((x, y, age, total) => {
    const alpha = Math.min(1, (age / total) * 0.85 + 0.1);
    const [sx, sy] = toScreen(x, y, sc);
    ctx.fillStyle = `rgba(255,183,77,${alpha})`;
    ctx.beginPath(); ctx.arc(sx, sy, 2.5, 0, Math.PI * 2); ctx.fill();
  });
}

function drawKalmanTrail(ctx: CanvasRenderingContext2D, buf: RingBuffer2D, sc: Scale) {
  let prevX = 0, prevY = 0, first = true;
  buf.forEach((x, y, age, total) => {
    const alpha = Math.min(1, (age / total) * 0.85 + 0.1);
    const [sx, sy] = toScreen(x, y, sc);
    if (!first) {
      ctx.strokeStyle = `rgba(233,69,96,${alpha})`;
      ctx.lineWidth = 2;
      ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(prevX, prevY); ctx.lineTo(sx, sy); ctx.stroke();
    }
    prevX = sx; prevY = sy; first = false;
  });
}

function drawAnchors(ctx: CanvasRenderingContext2D, anchors: AnchorPosition[], sc: Scale) {
  anchors.forEach(a => {
    const [x, y] = toScreen(a.x, a.y, sc);

    // Glow
    drawGlow(ctx, x, y, 18, ANCHOR_GLOW_RGB, 0.16);

    // Diamond icon (rotate 45°)
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(Math.PI / 4);
    ctx.fillStyle = ANCHOR_COLOR;
    ctx.fillRect(-5, -5, 10, 10);
    ctx.strokeStyle = 'rgba(0,0,0,0.2)';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(-5, -5, 10, 10);
    ctx.restore();

    // Label
    ctx.fillStyle = ANCHOR_COLOR;
    ctx.font = 'bold 11px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(`A${a.id}`, x, y - 20);

    // Coordinate text
    ctx.fillStyle = 'rgba(0,0,0,0.5)';
    ctx.font = '9px Consolas, monospace';
    ctx.fillText(`(${a.x.toFixed(1)}, ${a.y.toFixed(1)})`, x, y + 22);
  });
}

function drawDrone(ctx: CanvasRenderingContext2D, px: number, py: number) {
  const outerR = 14, innerR = 6, armLen = 10, motorR = 3;

  // Outer glow
  drawGlow(ctx, px, py, outerR * 1.5, DRONE_GLOW_RGB, 0.22);

  // Arms at 45° angles
  ctx.strokeStyle = 'rgba(0,0,0,0.3)';
  ctx.lineWidth = 2;
  [45, 135, 225, 315].forEach(deg => {
    const rad = deg * Math.PI / 180;
    const dx = Math.cos(rad) * armLen;
    const dy = Math.sin(rad) * armLen;
    ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px + dx, py + dy); ctx.stroke();
    // Motor circle
    ctx.fillStyle = 'rgba(0,0,0,0.5)';
    ctx.beginPath(); ctx.arc(px + dx, py + dy, motorR, 0, Math.PI * 2); ctx.fill();
  });

  // Center body
  ctx.fillStyle = 'white';
  ctx.beginPath(); ctx.arc(px, py, innerR, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = 'rgba(0,0,0,0.2)';
  ctx.lineWidth = 1;
  ctx.stroke();

  // Center accent dot (Kalman color)
  ctx.fillStyle = KALMAN_COLOR;
  ctx.beginPath(); ctx.arc(px, py, 2.5, 0, Math.PI * 2); ctx.fill();
}

function drawLegend(ctx: CanvasRenderingContext2D, showTrilat: boolean, showKalman: boolean) {
  const x = 10, y = 10;
  const lineH = 20, boxW = 175, boxH = (3 + (showTrilat ? 1 : 0) + (showKalman ? 1 : 0)) * lineH + 16;

  // Box background
  ctx.save();
  ctx.shadowColor = 'rgba(0,0,0,0.1)';
  ctx.shadowBlur = 10;
  ctx.shadowOffsetY = 4;
  ctx.fillStyle = LEGEND_BG;
  roundRect(ctx, x, y, boxW, boxH, 8);
  ctx.fill();
  ctx.restore();
  ctx.strokeStyle = LEGEND_BORDER;
  ctx.lineWidth = 1;
  roundRect(ctx, x, y, boxW, boxH, 8);
  ctx.stroke();

  let cy = y + 12;
  ctx.fillStyle = TEXT_COLOR;
  ctx.font = 'bold 11px Inter, sans-serif';
  ctx.textAlign = 'left';
  ctx.fillText('📡 LEGEND', x + 12, cy + 10); cy += lineH + 2;

  if (showTrilat) {
    ctx.fillStyle = TRILAT_COLOR;
    ctx.beginPath(); ctx.arc(x + 19, cy + 8, 4, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = TEXT_COLOR; ctx.font = '10px Inter, sans-serif';
    ctx.fillText('Trilateration', x + 30, cy + 12); cy += lineH;
  }
  if (showKalman) {
    ctx.strokeStyle = KALMAN_COLOR; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(x + 10, cy + 8); ctx.lineTo(x + 28, cy + 8); ctx.stroke();
    ctx.fillStyle = TEXT_COLOR; ctx.font = '10px Inter, sans-serif';
    ctx.fillText('Kalman Filter', x + 32, cy + 12); cy += lineH;
  }
  // Anchor
  ctx.fillStyle = ANCHOR_COLOR;
  ctx.beginPath(); ctx.arc(x + 19, cy + 8, 4, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = TEXT_COLOR; ctx.font = '10px Inter, sans-serif';
  ctx.fillText('Anchor', x + 30, cy + 12); cy += lineH;
  // Drone
  ctx.fillStyle = '#64748b';
  ctx.beginPath(); ctx.arc(x + 19, cy + 8, 4, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = TEXT_COLOR; ctx.font = '10px Inter, sans-serif';
  ctx.fillText('Drone (TAG)', x + 30, cy + 12);
}

function roundRect(ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number, r: number) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.lineTo(x + w - r, y); ctx.arcTo(x + w, y, x + w, y + r, r);
  ctx.lineTo(x + w, y + h - r); ctx.arcTo(x + w, y + h, x + w - r, y + h, r);
  ctx.lineTo(x + r, y + h); ctx.arcTo(x, y + h, x, y + h - r, r);
  ctx.lineTo(x, y + r); ctx.arcTo(x, y, x + r, y, r);
  ctx.closePath();
}

// ============================================================
// Component
// ============================================================
export function PositionMap({
  anchorLayout, distancesM, trilatPoint, kalmanPoint,
  registerFrameCallback, showRangeCircles, showTrilateration, showKalman, showGrid, gridSpacing
}: Props) {

  const canvasRef    = useRef<HTMLCanvasElement>(null);
  const scaleRef = useRef<Scale>({
    scale: 50, offsetX: 60, offsetY: 60, cssW: 800, cssH: 600,
    simW: 10, simH: 10, minX: -2, minY: -2,
  });
  const trilatBufRef = useRef<RingBuffer2D | null>(null);
  const kalmanBufRef = useRef<RingBuffer2D | null>(null);

  // Store latest props in ref so draw callback (bypass React) can access them
  const propsRef = useRef({ anchorLayout, distancesM, trilatPoint, kalmanPoint, showRangeCircles, showTrilateration, showKalman, showGrid, gridSpacing });
  useEffect(() => {
    propsRef.current = { anchorLayout, distancesM, trilatPoint, kalmanPoint, showRangeCircles, showTrilateration, showKalman, showGrid, gridSpacing };
  });


  // Compute world bounds from anchor layout (with uniform padding)
  const computeWorldBounds = useCallback((layout: AnchorPosition[]) => {
    if (!layout.length) return { minX: -2, maxX: 8, minY: -2, maxY: 8 };
    const xs = layout.map(a => a.x);
    const ys = layout.map(a => a.y);
    return {
      minX: Math.min(...xs),
      maxX: Math.max(...xs),
      minY: Math.min(...ys),
      maxY: Math.max(...ys),
    };
  }, []);


  const redraw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const sc = scaleRef.current;
    const p = propsRef.current;

    ctx.clearRect(0, 0, sc.cssW, sc.cssH);
    // Let the transparent background show the glassmorphism panel underneath


    if (p.showGrid) drawGrid(ctx, sc, p.gridSpacing);
    if (p.showRangeCircles && p.distancesM) drawRangeCircles(ctx, p.anchorLayout, p.distancesM, sc);


    if (p.showTrilateration && trilatBufRef.current && trilatBufRef.current.length > 0)
      drawTrilatTrail(ctx, trilatBufRef.current, sc);
    if (p.showKalman && kalmanBufRef.current && kalmanBufRef.current.length > 0)
      drawKalmanTrail(ctx, kalmanBufRef.current, sc);

    drawAnchors(ctx, p.anchorLayout, sc);

    if (p.kalmanPoint) {
      const [px, py] = toScreen(p.kalmanPoint.x, p.kalmanPoint.y, sc);
      drawDrone(ctx, px, py);
    } else if (p.trilatPoint) {
      const [px, py] = toScreen(p.trilatPoint.x, p.trilatPoint.y, sc);
      drawDrone(ctx, px, py);
    }

    drawLegend(ctx, p.showTrilateration, p.showKalman);
  }, []);

  // Register frame callback (direct draw — bypass React)
  useEffect(() => {
    registerFrameCallback((trilat, kalman) => {
      trilatBufRef.current = trilat;
      kalmanBufRef.current = kalman;
      redraw();
    });
    return () => registerFrameCallback(null);
  }, [registerFrameCallback, redraw]);

  // Redraw when props change (resize, anchor layout, show flags, etc.)
  useEffect(() => { redraw(); });

  // ResizeObserver — sync canvas resolution to actual CSS size + devicePixelRatio
  useEffect(() => {
    const canvas = canvasRef.current;
    const container = canvas?.parentElement;
    if (!canvas || !container) return;

    const resize = () => {
      const dpr = window.devicePixelRatio || 1;
      const cssW = container.clientWidth;
      const cssH = container.clientHeight;
      if (cssW < 10 || cssH < 10) return;
      canvas.width  = Math.round(cssW * dpr);
      canvas.height = Math.round(cssH * dpr);
      canvas.style.width  = `${cssW}px`;
      canvas.style.height = `${cssH}px`;
      canvas.getContext('2d')?.setTransform(dpr, 0, 0, dpr, 0, 0);

      const { minX, maxX, minY, maxY } = computeWorldBounds(propsRef.current.anchorLayout);
      scaleRef.current = computeScale(cssW, cssH, minX, maxX, minY, maxY);
      redraw();
    };

    const ro = new ResizeObserver(resize);
    ro.observe(container);
    resize();
    return () => ro.disconnect();
  }, [computeWorldBounds, redraw]);

  // Update world bounds when anchor layout changes
  useEffect(() => {
    const { minX, maxX, minY, maxY } = computeWorldBounds(anchorLayout);
    const sc = scaleRef.current;
    scaleRef.current = computeScale(sc.cssW, sc.cssH, minX, maxX, minY, maxY);
    redraw();
  }, [anchorLayout, computeWorldBounds, redraw]);


  return <canvas ref={canvasRef} style={{ display: 'block', width: '100%', height: '100%', borderRadius: 8 }} />;
}
