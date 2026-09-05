import assert from 'node:assert/strict';
import { mkdtemp, readFile, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import ts from 'typescript';

const scriptDir = dirname(fileURLToPath(import.meta.url));
const tempDir = await mkdtemp(join(tmpdir(), 'uwb-anchor-state-'));

try {
  const sourcePath = join(scriptDir, '..', 'src', 'lib', 'anchorState.ts');
  const outputPath = join(tempDir, 'anchorState.mjs');
  const source = await readFile(sourcePath, 'utf8');
  const transpiled = ts.transpileModule(source, {
    fileName: sourcePath,
    reportDiagnostics: true,
    compilerOptions: { target: ts.ScriptTarget.ES2020, module: ts.ModuleKind.ES2020 },
  });
  assert.equal(transpiled.diagnostics?.length ?? 0, 0);
  await writeFile(outputPath, transpiled.outputText, 'utf8');

  const {
    deriveAnchorLinkState,
    updateCalibrationMissingIds,
  } = await import(pathToFileURL(outputPath).href);

  assert.deepEqual(
    deriveAnchorLinkState(true, 20, false),
    { isOnline: true, isStale: false, isOffline: false },
    'one missed 20ms slot must not flap ONLINE to STALE/OFFLINE',
  );
  assert.deepEqual(
    deriveAnchorLinkState(true, 80, false),
    { isOnline: false, isStale: true, isOffline: false },
    'a sustained interruption becomes STALE',
  );
  assert.deepEqual(
    deriveAnchorLinkState(true, 800, false),
    { isOnline: false, isStale: false, isOffline: true },
    'a long interruption becomes OFFLINE',
  );

  const missing = updateCalibrationMissingIds(
    new Set(),
    [1, 2, 3, 4],
    {
      4: {
        id: 4,
        valid: false,
        status: 0x20,
        ageMs: 0xffff,
        rawMm: null,
        filtMm: null,
        fppDbm: null,
        diagnosticRawMm: 158500,
      },
    },
  );
  assert.equal(missing.has(4), true);

  const retainedThroughTimeout = updateCalibrationMissingIds(
    missing,
    [1, 2, 3, 4],
    {
      4: {
        id: 4,
        valid: false,
        status: 0x01,
        ageMs: 0xffff,
        rawMm: null,
        filtMm: null,
        fppDbm: null,
      },
    },
  );
  assert.equal(
    retainedThroughTimeout.has(4),
    true,
    'timeout must not erase CALIBRATION REQUIRED',
  );

  const clearedByValidSample = updateCalibrationMissingIds(
    retainedThroughTimeout,
    [1, 2, 3, 4],
    {
      4: {
        id: 4,
        valid: true,
        status: 0,
        ageMs: 5,
        rawMm: 1000,
        filtMm: 1000,
        fppDbm: -70,
      },
    },
  );
  assert.equal(clearedByValidSample.has(4), false);

  console.log('anchorState: debounce and calibration latch tests passed');
} finally {
  await rm(tempDir, { recursive: true, force: true });
}
