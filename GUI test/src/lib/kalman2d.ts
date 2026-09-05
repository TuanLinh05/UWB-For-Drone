import { Matrix } from './matrix';

export class KalmanFilter2D {
  private x: Matrix;
  private P: Matrix;
  private F = Matrix.identity(4);
  private readonly H = Matrix.fromArray([[1,0,0,0],[0,1,0,0]]);
  private Q = Matrix.identity(4);
  private R = Matrix.identity(2);
  private readonly I4 = Matrix.identity(4);

  dt: number;
  private _q: number;
  private _r: number;
  private initialized = false;

  constructor(dt = 0.02, processNoise = 0.1, measurementNoise = 0.5) {
    this.dt = dt; this._q = processNoise; this._r = measurementNoise;
    this.x = Matrix.fromVector([0,0,0,0]);
    this.P = Matrix.identity(4).scale(100);
    this.rebuildF(); this.rebuildQ(); this.rebuildR();
  }

  get processNoise() { return this._q; }
  set processNoise(v: number) {
    if (!Number.isFinite(v) || v < 0) throw new RangeError('processNoise must be finite and non-negative');
    this._q = v;
    this.rebuildQ();
  }
  get measurementNoise() { return this._r; }
  set measurementNoise(v: number) {
    if (!Number.isFinite(v) || v <= 0) throw new RangeError('measurementNoise must be finite and positive');
    this._r = v;
    this.rebuildR();
  }
  get isInitialized() { return this.initialized; }

  initialize(x: number, y: number, vx = 0, vy = 0) {
    if (![x, y, vx, vy].every(Number.isFinite)) {
      throw new RangeError('Kalman state must contain only finite values');
    }
    this.x = Matrix.fromVector([x, y, vx, vy]);
    this.P = Matrix.identity(4).scale(10);
    this.initialized = true;
  }
  reset() {
    this.x = Matrix.fromVector([0,0,0,0]);
    this.P = Matrix.identity(4).scale(100);
    this.initialized = false;
  }

  /** Update the sample interval and both time-dependent dynamics matrices. */
  setDeltaTime(dt: number) {
    if (!Number.isFinite(dt) || dt <= 0) throw new RangeError('dt must be finite and positive');
    this.dt = dt;
    this.rebuildDynamics();
  }

  update(measX: number, measY: number, dt?: number) {
    if (!Number.isFinite(measX) || !Number.isFinite(measY)) {
      throw new RangeError('Kalman measurement must contain only finite values');
    }
    if (dt !== undefined) this.setDeltaTime(dt);
    else this.rebuildDynamics();

    // Starting at the first real position avoids the long, artificial flight
    // from (0, 0) that occurs when the first measurement is treated as a
    // normal correction of an uninitialized state.
    if (!this.initialized) {
      this.initialize(measX, measY);
      return this.getState();
    }

    const xPred = this.F.mul(this.x);
    const PPred = this.F.mul(this.P).mul(this.F.transpose()).add(this.Q);
    const z = Matrix.fromVector([measX, measY]);
    const y = z.sub(this.H.mul(xPred));
    const S = this.H.mul(PPred).mul(this.H.transpose()).add(this.R);
    const K = PPred.mul(this.H.transpose()).mul(S.inverse2x2());
    this.x = xPred.add(K.mul(y));
    this.P = this.I4.sub(K.mul(this.H)).mul(PPred);
    return this.getState();
  }

  getState() {
    return { x: this.x.get(0,0), y: this.x.get(1,0), vx: this.x.get(2,0), vy: this.x.get(3,0) };
  }

  private rebuildF() {
    const dt = this.dt;
    this.F = Matrix.fromArray([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]);
  }
  private rebuildQ() {
    const dt = this.dt, dt2 = dt*dt, dt3 = dt2*dt/2, dt4 = dt2*dt2/4, q = this._q;
    this.Q = Matrix.fromArray([
      [dt4*q, 0,     dt3*q, 0    ],
      [0,     dt4*q, 0,     dt3*q],
      [dt3*q, 0,     dt2*q, 0    ],
      [0,     dt3*q, 0,     dt2*q],
    ]);
  }
  private rebuildR() {
    const r = this._r;
    this.R = Matrix.fromArray([[r,0],[0,r]]);
  }

  private rebuildDynamics() {
    this.rebuildF();
    this.rebuildQ();
  }
}
