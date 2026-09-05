import { execFileSync } from 'node:child_process';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

/*
 * The CI/desktop environment does not always have the ARM GCC toolchain, but
 * host GCC can still preprocess and syntax-check the complete TAG translation
 * unit. Run every Adaptive Legacy and C9.2 compile-time branch here: OFF
 * must preserve the deployed path, while SHADOW and ACTIVE must remain
 * warning-clean.  The production-active combination is deliberately omitted:
 * uwb_calibration.h rejects two adaptive controllers owning output at once.
 *
 * CMSIS maps a 32-bit MCU register to a host pointer in two inline helpers;
 * suppress only that architecture-size diagnostic. This is not a substitute
 * for the final STM32CubeIDE build, which must still be done before flashing.
 */
const scriptDirectory = dirname(fileURLToPath(import.meta.url));
const workspaceRoot = resolve(scriptDirectory, '..', '..');
const tagRoot = join(workspaceRoot, 'STM32_UWB', 'TAG');
const includeDirectories = [
  join(tagRoot, 'Core', 'Inc'),
  join(tagRoot, 'Drivers', 'STM32F1xx_HAL_Driver', 'Inc', 'Legacy'),
  join(tagRoot, 'Drivers', 'STM32F1xx_HAL_Driver', 'Inc'),
  join(tagRoot, 'Drivers', 'CMSIS', 'Device', 'ST', 'STM32F1xx', 'Include'),
  join(tagRoot, 'Drivers', 'CMSIS', 'Include'),
];
const sources = [
  join(tagRoot, 'Core', 'Src', 'tag_ranging.c'),
  join(tagRoot, 'Core', 'Src', 'telemetry.c'),
];

const configurations = [
  { legacyMode: 0, motionMode: 0, label: 'legacy-off/c9.2-off' },
  { legacyMode: 1, motionMode: 0, label: 'legacy-shadow/c9.2-off' },
  { legacyMode: 2, motionMode: 0, label: 'legacy-active/c9.2-off' },
  { legacyMode: 0, motionMode: 1, label: 'legacy-off/c9.2-shadow' },
  { legacyMode: 1, motionMode: 1, label: 'legacy-shadow/c9.2-shadow' },
  { legacyMode: 0, motionMode: 2, label: 'legacy-off/c9.2-active' },
  { legacyMode: 1, motionMode: 2, label: 'legacy-shadow/c9.2-active' },
];

for (const configuration of configurations) {
  for (const source of sources) {
    execFileSync('gcc', [
      '-std=gnu11',
      '-fsyntax-only',
      '-Wall',
      '-Wextra',
      '-Werror',
      '-Wno-int-to-pointer-cast',
      '-DDEBUG',
      '-DUSE_HAL_DRIVER',
      '-DSTM32F103xB',
      `-DUWB_LEGACY_ADAPTIVE_MODE=${configuration.legacyMode}`,
      `-DUWB_C9_2_MOTION_MODE=${configuration.motionMode}`,
      ...includeDirectories.flatMap(directory => [`-I${directory}`]),
      source,
    ], { stdio: 'inherit', windowsHide: true });
  }
}

console.log('TAG adaptive syntax: C9.1 and C9.2 OFF/SHADOW/ACTIVE configurations passed');
