import { execFileSync } from 'node:child_process';
import { mkdirSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const scriptDirectory = dirname(fileURLToPath(import.meta.url));
const workspaceRoot = resolve(scriptDirectory, '..', '..');
const includeDirectory = join(workspaceRoot, 'STM32_UWB', 'TAG', 'Core', 'Inc');
const test = join(workspaceRoot, 'STM32_UWB', 'HostTests', 'legacy_adaptive_tracking_test.c');
const outputDirectory = join(tmpdir(), `uwb-legacy-adaptive-${process.pid}`);
const executable = join(outputDirectory, 'legacy_adaptive_tracking_test.exe');

mkdirSync(outputDirectory, { recursive: true });
try {
  execFileSync('gcc', [
    '-std=c11',
    '-Wall',
    '-Wextra',
    '-Werror',
    '-pedantic',
    `-I${includeDirectory}`,
    test,
    '-lm',
    '-o',
    executable,
  ], { stdio: 'inherit', windowsHide: true });
  execFileSync(executable, [], { stdio: 'inherit', windowsHide: true });
} finally {
  rmSync(outputDirectory, { recursive: true, force: true });
}
