import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    include: ['src/**/*.vitest.test.ts'],
    environment: 'node',
    coverage: { enabled: false },
  },
});
