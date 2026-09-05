export const MAX_TRAIL = 800;

/**
 * Ring buffer lưu tối đa MAX_TRAIL điểm 2D (x, y) theo kiểu vòng tròn.
 * KHÔNG cấp phát mảng mới khi push() — tránh GC pressure khi chạy 50Hz.
 */
export class RingBuffer2D {
  private xs = new Float64Array(MAX_TRAIL);
  private ys = new Float64Array(MAX_TRAIL);
  private head = 0;
  private _count = 0;

  push(x: number, y: number) {
    this.xs[this.head] = x;
    this.ys[this.head] = y;
    this.head = (this.head + 1) % MAX_TRAIL;
    if (this._count < MAX_TRAIL) this._count++;
  }

  /** Duyệt cũ → mới, ageIndex=0 là điểm cũ nhất, ageIndex=count-1 là mới nhất. */
  forEach(cb: (x: number, y: number, ageIndex: number, total: number) => void) {
    const start = this._count < MAX_TRAIL ? 0 : this.head;
    for (let i = 0; i < this._count; i++) {
      const idx = (start + i) % MAX_TRAIL;
      cb(this.xs[idx], this.ys[idx], i, this._count);
    }
  }

  get length() { return this._count; }
  clear() { this.head = 0; this._count = 0; }
}
