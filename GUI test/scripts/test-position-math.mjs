import assert from 'node:assert/strict';
import { mkdtemp, readFile, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import ts from 'typescript';

const scriptDir = dirname(fileURLToPath(import.meta.url));
const sourceDir = join(scriptDir, '..', 'src', 'lib');
const tempDir = await mkdtemp(join(tmpdir(), 'uwb-position-math-'));
const moduleNames = ['matrix', 'kalman2d', 'positionTiming', 'trilateration', 'positionMath.test'];

try {
  for (const moduleName of moduleNames) {
    const sourcePath = join(sourceDir, `${moduleName}.ts`);
    const source = await readFile(sourcePath, 'utf8');
    const transpiled = ts.transpileModule(source, {
      fileName: sourcePath,
      reportDiagnostics: true,
      compilerOptions: { target: ts.ScriptTarget.ES2020, module: ts.ModuleKind.ES2020 },
    });
    assert.equal(transpiled.diagnostics?.length ?? 0, 0, `${moduleName} transpiles cleanly`);
    const runnable = transpiled.outputText.replace(
      /(from\s+['"]\.\/[^'"]+)(['"])/g,
      '$1.mjs$2',
    );
    await writeFile(join(tempDir, `${moduleName}.mjs`), runnable, 'utf8');
  }

  const tests = await import(pathToFileURL(join(tempDir, 'positionMath.test.mjs')).href);
  tests.runPositionMathSelfTests();
  console.log('positionMath: least-squares, geometry guards, MCU timing and Kalman dynamics passed');
} finally {
  await rm(tempDir, { recursive: true, force: true });
}
