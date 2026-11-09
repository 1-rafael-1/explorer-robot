// A tiny assertion and reporting harness for the headless pages.
//
// The page starts with `data-result="running"`; this module flips it to `pass`
// or `fail` and writes the details into `#results`. The CI runner reads the
// dumped DOM, so a page that throws during import never reports a pass.

const cases = [];
const failures = [];
let started = false;

globalThis.addEventListener?.('error', (event) => {
  failures.push(`uncaught: ${event.message ?? event.error ?? 'error'}`);
  document.documentElement.dataset.result = 'fail';
});
globalThis.addEventListener?.('unhandledrejection', (event) => {
  failures.push(`unhandled rejection: ${event.reason ?? 'rejection'}`);
  document.documentElement.dataset.result = 'fail';
});

/** Registers one named test. */
export function test(name, body) {
  cases.push({ name, body });
}

/** Throws unless `condition` is truthy. */
export function assert(condition, message = 'assertion failed') {
  if (!condition) throw new Error(message);
}

/** Throws unless `actual` deep-equals `expected` (JSON comparison). */
export function assertEqual(actual, expected, message = 'values differ') {
  const left = JSON.stringify(actual);
  const right = JSON.stringify(expected);
  if (left !== right) throw new Error(`${message}: expected ${right}, got ${left}`);
}

/** Throws unless `actual` is within `tolerance` of `expected`. */
export function assertClose(actual, expected, tolerance, message = 'values differ') {
  if (!(Math.abs(actual - expected) <= tolerance)) {
    throw new Error(`${message}: expected ${expected} ± ${tolerance}, got ${actual}`);
  }
}

/** Runs every registered test and reports through the DOM. */
export async function run() {
  if (started) return;
  started = true;
  const lines = [];
  for (const item of cases) {
    try {
      await item.body();
      lines.push(`PASS ${item.name}`);
    } catch (error) {
      failures.push(`${item.name}: ${error?.stack ?? error}`);
      lines.push(`FAIL ${item.name}: ${error?.message ?? error}`);
    }
  }
  const failed = failures.length > 0 || lines.some((line) => line.startsWith('FAIL'));
  const pre = document.getElementById('results');
  if (pre) pre.textContent = [...lines, ...failures.map((failure) => `  ${failure}`)].join('\n');
  const status = document.getElementById('status');
  if (status) status.textContent = failed ? `FAIL ${failures.length} of ${cases.length}` : `PASS ${cases.length}`;
  const output = document.getElementById('test-results');
  if (output) output.textContent = JSON.stringify({ failed, total: cases.length, failures, lines });
  document.documentElement.dataset.result = failed ? 'fail' : 'pass';
  document.title = failed ? `FAIL ${failures.length}/${cases.length}` : `PASS ${cases.length}`;
}

/** Runs when the module graph settles, so every page reports exactly once. */
export function autorun() {
  globalThis.setTimeout(() => {
    run().catch((error) => {
      failures.push(`runner: ${error?.stack ?? error}`);
      document.documentElement.dataset.result = 'fail';
      document.title = 'FAIL runner';
    });
  }, 0);
}
