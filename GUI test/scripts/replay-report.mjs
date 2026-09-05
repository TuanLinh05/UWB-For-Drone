import { readFile, mkdir, mkdtemp, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import ts from 'typescript';

const args = process.argv.slice(2);
const inputArg = args.find(arg => !arg.startsWith('--'));
const outputFlagIndex = args.indexOf('--out');
const outputArg = outputFlagIndex >= 0 ? args[outputFlagIndex + 1] : undefined;

if (!inputArg) {
  console.error('Usage: npm run replay:report -- <uwb_log.csv> [--out report.json]');
  process.exitCode = 2;
} else {
  const scriptDir = dirname(fileURLToPath(import.meta.url));
  const sourceDir = join(scriptDir, '..', 'src', 'lib');
  const tempDir = await mkdtemp(join(tmpdir(), 'uwb-replay-report-'));
  try {
    const moduleNames = [
      'replayDataset',
      'replayRunner',
      'adaptiveLegacyRangeFilter',
      'motionAdaptiveRangeFilter',
      'position/config',
      'position/adaptiveKalman2d',
      'position/geometry2d',
      'position/leaveOneOut',
      'position/matrix2',
      'position/mcuClock',
      'position/observationBuilder',
      'position/positionPipeline',
      'position/replayVariant',
      'position/robustSolver2d',
    ];
    for (const moduleName of moduleNames) {
      const sourcePath = join(sourceDir, `${moduleName}.ts`);
      const source = await readFile(sourcePath, 'utf8');
      const transpiled = ts.transpileModule(source, {
        fileName: sourcePath,
        reportDiagnostics: true,
        compilerOptions: { target: ts.ScriptTarget.ES2020, module: ts.ModuleKind.ES2020 },
      });
      if ((transpiled.diagnostics?.length ?? 0) > 0) {
        throw new Error(`Unable to transpile ${moduleName}.ts for the replay CLI.`);
      }
      const runnable = transpiled.outputText.replace(
        /(from\s+['"]\.\/[^'"]+)(['"])/g,
        '$1.mjs$2',
      );
      const outputPath = join(tempDir, `${moduleName}.mjs`);
      await mkdir(dirname(outputPath), { recursive: true });
      await writeFile(outputPath, runnable, 'utf8');
    }

    const datasetModule = await import(pathToFileURL(join(tempDir, 'replayDataset.mjs')).href);
    const runnerModule = await import(pathToFileURL(join(tempDir, 'replayRunner.mjs')).href);
    const positionVariantModule = await import(
      pathToFileURL(join(tempDir, 'position', 'replayVariant.mjs')).href
    );
    const csv = await readFile(resolve(inputArg), 'utf8');
    const parsed = datasetModule.parseReplayCsv(csv);
    if (!parsed.ok) {
      console.error(JSON.stringify({ ok: false, errors: parsed.errors }, null, 2));
      process.exitCode = 1;
    } else {
      const report = runnerModule.runReplayVariants(parsed.dataset, [
        runnerModule.recordedRawVariant,
        runnerModule.recordedFilteredVariant,
        runnerModule.c9MedianGateReplayVariant,
        runnerModule.adaptiveLegacyReplayVariant,
        runnerModule.c9_2MotionAdaptiveReplayVariant,
        positionVariantModule.createC6RobustReplayVariant(parsed.dataset),
        positionVariantModule.createC8AdaptiveReplayVariant(parsed.dataset),
      ]);
      const output = `${JSON.stringify({ ok: true, report }, null, 2)}\n`;
      if (outputArg) await writeFile(resolve(outputArg), output, 'utf8');
      else process.stdout.write(output);
    }
  } finally {
    await rm(tempDir, { recursive: true, force: true });
  }
}
