import { execFileSync } from 'node:child_process';
import { mkdirSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const scriptDirectory = dirname(fileURLToPath(import.meta.url));
const workspaceRoot = resolve(scriptDirectory, '..', '..');
const tagRoot = join(workspaceRoot, 'STM32_UWB', 'TAG');
const includeDirectory = join(tagRoot, 'Core', 'Inc');
const source = join(tagRoot, 'Core', 'Src', 'range_filter.c');
const test = join(workspaceRoot, 'STM32_UWB', 'HostTests', 'range_filter_test.c');
const outputDirectory = join(tmpdir(), `uwb-range-filter-${process.pid}`);

mkdirSync(outputDirectory, { recursive: true });
try {
  for (const mode of [1, 2]) {
    const executable = join(outputDirectory, `range_filter_mode_${mode}.exe`);
    execFileSync('gcc', [
      '-std=c11',
      '-Wall',
      '-Wextra',
      '-Werror',
      '-pedantic',
      `-DUWB_RANGE_FILTER_MODE=${mode}`,
      /* Alternate C9 range-filter profiles are tested in isolation. The
       * deployed C9.1/C9.2 controllers intentionally require Legacy mode. */
      '-DUWB_LEGACY_ADAPTIVE_MODE=0',
      '-DUWB_C9_2_MOTION_MODE=0',
      `-I${includeDirectory}`,
      source,
      test,
      '-lm',
      '-o',
      executable,
    ], { stdio: 'inherit', windowsHide: true });
    execFileSync(executable, [], { stdio: 'inherit', windowsHide: true });
  }
} finally {
  rmSync(outputDirectory, { recursive: true, force: true });
}
