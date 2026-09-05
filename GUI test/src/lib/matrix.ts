export class Matrix {
  readonly rows: number;
  readonly cols: number;
  private data: number[][];

  constructor(rows: number, cols: number) {
    this.rows = rows; this.cols = cols;
    this.data = Array.from({ length: rows }, () => new Array(cols).fill(0));
  }

  get(r: number, c: number) { return this.data[r][c]; }
  set(r: number, c: number, v: number) { this.data[r][c] = v; }

  static identity(n: number): Matrix {
    const m = new Matrix(n, n);
    for (let i = 0; i < n; i++) m.set(i, i, 1);
    return m;
  }
  static fromVector(v: number[]): Matrix {
    const m = new Matrix(v.length, 1);
    v.forEach((val, i) => m.set(i, 0, val));
    return m;
  }
  static fromArray(rows: number[][]): Matrix {
    const m = new Matrix(rows.length, rows[0].length);
    rows.forEach((row, r) => row.forEach((val, c) => m.set(r, c, val)));
    return m;
  }

  transpose(): Matrix {
    const r = new Matrix(this.cols, this.rows);
    for (let i = 0; i < this.rows; i++)
      for (let j = 0; j < this.cols; j++) r.set(j, i, this.get(i, j));
    return r;
  }
  add(o: Matrix): Matrix {
    const r = new Matrix(this.rows, this.cols);
    for (let i = 0; i < this.rows; i++)
      for (let j = 0; j < this.cols; j++) r.set(i, j, this.get(i, j) + o.get(i, j));
    return r;
  }
  sub(o: Matrix): Matrix {
    const r = new Matrix(this.rows, this.cols);
    for (let i = 0; i < this.rows; i++)
      for (let j = 0; j < this.cols; j++) r.set(i, j, this.get(i, j) - o.get(i, j));
    return r;
  }
  mul(o: Matrix): Matrix {
    const r = new Matrix(this.rows, o.cols);
    for (let i = 0; i < this.rows; i++)
      for (let j = 0; j < o.cols; j++) {
        let sum = 0;
        for (let k = 0; k < this.cols; k++) sum += this.get(i, k) * o.get(k, j);
        r.set(i, j, sum);
      }
    return r;
  }
  scale(s: number): Matrix {
    const r = new Matrix(this.rows, this.cols);
    for (let i = 0; i < this.rows; i++)
      for (let j = 0; j < this.cols; j++) r.set(i, j, this.get(i, j) * s);
    return r;
  }

  inverse2x2(): Matrix {
    const a = this.get(0,0), b = this.get(0,1), c = this.get(1,0), d = this.get(1,1);
    const det = a*d - b*c;
    if (Math.abs(det) < 1e-12) throw new Error('Matrix singular (det ≈ 0)');
    const inv = 1/det;
    const r = new Matrix(2,2);
    r.set(0,0, d*inv); r.set(0,1, -b*inv); r.set(1,0, -c*inv); r.set(1,1, a*inv);
    return r;
  }
}
