// A deterministic timer seam for the transport tests: no wall-clock sleeps.

/** Creates a fake timer set with `advance(ms)`. */
export function createFakeClock() {
  let now = 0;
  let nextId = 1;
  const timers = new Map();

  function setTimeoutFn(fn, ms = 0) {
    const id = nextId;
    nextId += 1;
    timers.set(id, { time: now + Math.max(0, ms), fn, interval: null });
    return id;
  }

  function setIntervalFn(fn, ms = 1) {
    const period = Math.max(1, ms);
    const id = nextId;
    nextId += 1;
    timers.set(id, { time: now + period, fn, interval: period });
    return id;
  }

  function clear(id) {
    timers.delete(id);
  }

  function advance(ms) {
    const end = now + Math.max(0, ms);
    for (;;) {
      let due = null;
      for (const [id, timer] of timers) {
        if (timer.time > end) continue;
        if (!due || timer.time < due.timer.time || (timer.time === due.timer.time && id < due.id)) {
          due = { id, timer };
        }
      }
      if (!due) break;
      now = due.timer.time;
      if (due.timer.interval != null) due.timer.time += due.timer.interval;
      else timers.delete(due.id);
      due.timer.fn();
    }
    now = end;
  }

  return {
    now: () => now,
    setTimeout: setTimeoutFn,
    clearTimeout: clear,
    setInterval: setIntervalFn,
    clearInterval: clear,
    advance,
    get pending() {
      return timers.size;
    },
  };
}
