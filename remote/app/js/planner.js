// The point-and-go planner: it owns the Target Point and the two-leg state
// machine, with the command sender and the clock injected (ADR-0043).
//
// Ticket 05 owns the preview and the rotate leg. The drive leg, busy handling,
// unknown outcomes and the contextual Stop follow in ticket 06; the `afterRotate`
// seam and the in-flight wait are written so those additions do not rewrite this.
//
// There is no distance cap, scan clamp, short-leg refusal, beyond-range warning,
// age gate or fixed completion timeout: the robot owns motion safety, and the
// wait ends on the completion Event, link loss, a Boot Id change, Stop or the
// Activity leaving running.

import { planLeg, pointInFootprint } from './geometry.js';
import { ROTATE_SPEED, MANUAL_DRIVE_ID, centidegrees } from './protocol.js';
import { activityBusy, controlEnabled, emptyDrive, latestPose } from './store.js';

const RAD = Math.PI / 180;

/** Formats an angle with one decimal, or an em dash when it is unknown. */
function angleText(value) {
  return Number.isFinite(value) ? `${Number(value).toFixed(1)}°` : '—';
}

/** Returns the key the store uses to dedupe an Event. */
function eventKey(event) {
  return event.boot_id == null ? `notice:${event.ts}` : `${event.boot_id}:${event.seq}`;
}

/** Creates the planner over a store, a command sender and a clock. */
export function createPlanner({ store, send = () => 0, now = () => Date.now(), announce = () => {} } = {}) {
  // The one in-flight rotate wait, or `null`. There is never more than one.
  let waiting = null;

  function setDrive(drive) {
    store.dispatch({ type: '@drive', drive });
  }

  /** Commits a Target Point and draws its preview from the current Pose. */
  function preview(target) {
    if (waiting) {
      announce('Wait for the robot to finish before choosing another point.', 'warn');
      return false;
    }
    if (!target || !Number.isFinite(target.x) || !Number.isFinite(target.y)) {
      announce('Tap a point on the radar or enter a bearing and distance.', 'warn');
      return false;
    }
    const pose = latestPose(store.state);
    const leg = planLeg(pose, target);
    setDrive({
      plan: { ...leg, target: { x: target.x, y: target.y }, pose: { x: pose.x, y: pose.y, yaw: pose.yaw } },
      stage: 'idle',
      report: null,
      sequence: null,
    });
    return true;
  }

  /**
   * Handles a radar tap. The drawn robot footprint is the only minimum: a tap
   * inside it produces no command, while a deliberately short leg outside it is
   * allowed (ADR-0043).
   */
  function pick(point, { inside_robot = false, scale = 1 } = {}) {
    if (waiting) {
      announce('Wait for the robot to finish before choosing another point.', 'warn');
      return false;
    }
    if (!point || !Number.isFinite(point.x) || !Number.isFinite(point.y)) {
      announce('Tap a point outside the robot footprint.', 'warn');
      return false;
    }
    const pose = latestPose(store.state);
    if (inside_robot || pointInFootprint(pose, point, scale)) {
      announce('Tap a point outside the robot footprint.', 'warn');
      return false;
    }
    return preview(point);
  }

  /** Creates the same Target Point from a bearing relative to the current heading. */
  function numeric({ bearing_deg, distance_cm }) {
    if (!Number.isFinite(bearing_deg) || !Number.isFinite(distance_cm)) {
      announce('Enter a bearing in degrees and a distance in centimetres.', 'warn');
      return false;
    }
    const pose = latestPose(store.state);
    const angle = (pose.yaw + bearing_deg) * RAD;
    return preview({
      x: pose.x + distance_cm * Math.cos(angle),
      y: pose.y + distance_cm * Math.sin(angle),
    });
  }

  /** Clears the Target Point and its preview. */
  function cancel() {
    if (waiting) {
      announce('Wait for the robot to finish before cancelling the target.', 'warn');
      return;
    }
    setDrive(emptyDrive());
  }

  /** Clears the last outcome report, keeping the Target Point for a re-issue. */
  function dismiss() {
    const drive = store.state.drive;
    if (!drive?.report) return;
    setDrive({ ...drive, report: null });
  }

  /** Builds the report for a rotate completion Event. */
  function completionReport(w, event) {
    const value = event.v ?? {};
    const status = value.status ?? 'ok';
    const target = Number.isFinite(value.target_deg) ? value.target_deg : w.angle;
    const error = Number.isFinite(value.error_deg) ? value.error_deg : 0;
    const actual = target + error;
    const reason = value.detail || (status === 'ok' ? 'done' : status);
    const title =
      status === 'ok'
        ? 'Rotation complete'
        : status === 'cancelled'
          ? 'Rotation cancelled'
          : 'Rotation failed';
    const kind = status === 'ok' ? 'success' : status === 'cancelled' ? 'warn' : 'bad';
    return {
      kind,
      title,
      text: `Target ${angleText(target)}, actual ${angleText(actual)} (${reason}).`,
      status,
      target_deg: target,
      actual_deg: actual,
      reason,
    };
  }

  /** Builds the report for a rejected, indeterminate or timed-out ACK. */
  function resultReport(result) {
    const unknown = result.outcome === 'indeterminate' || result.outcome === 'timeout';
    const reason = result.detail || result.status || result.outcome || 'rejected';
    if (unknown) {
      return {
        kind: 'unknown',
        title: 'Outcome unknown',
        text: `The rotation ${result.outcome === 'timeout' ? 'timed out' : 'outcome is unknown'}; re-issue when ready.`,
        status: result.status ?? null,
        reason,
      };
    }
    return {
      kind: 'bad',
      title: 'Rotation rejected',
      text: result.detail || `The robot rejected the rotation (${result.status ?? 'rejected'}).`,
      status: result.status ?? null,
      reason,
    };
  }

  /**
   * Ends the in-flight wait, renders the outcome and resolves `go()`. A
   * successful rotate passes through `afterRotate`, the seam ticket 06 fills
   * with the drive leg.
   */
  function endWait(w, report, continueAfterRotate = false) {
    if (!w || waiting !== w) return;
    waiting = null;
    w.unsubscribe();
    const plan = store.state.drive?.plan ?? w.plan;
    setDrive({ plan, stage: 'idle', report, sequence: w.seq });
    const outcome = continueAfterRotate ? afterRotate(w, report) : report;
    Promise.resolve(outcome).then((value) => w.resolve(value));
  }

  function endForReason(w, reason, detail) {
    const target = `Target ${angleText(w.angle)} was not confirmed.`;
    if (reason === 'link') {
      endWait(w, {
        kind: 'warn',
        title: 'Link lost',
        text: `${target} The rotation outcome is unknown; the robot stops on link loss.`,
        reason: 'link lost',
      });
    } else if (reason === 'boot') {
      endWait(w, {
        kind: 'unknown',
        title: 'Outcome unknown',
        text: `${target} The robot restarted (new Boot Id). Check it before re-issuing.`,
        reason: 'Boot Id changed',
      });
    } else if (reason === 'stop') {
      endWait(w, {
        kind: 'warn',
        title: 'Stop not confirmed',
        text: `${target} The robot stops on link loss.`,
        reason: 'stop requested',
      });
    } else {
      endWait(w, {
        kind: 'warn',
        title: 'Rotation ended',
        text: `${target} ${detail ?? 'The Activity left running before the rotation completed.'}`,
        reason: 'activity ended',
      });
    }
  }

  /** Finds the completion Event for this rotate, tolerating a null attribution. */
  function completionFor(w, state) {
    return (
      state.events.find((event) => {
        if (w.startKeys.has(eventKey(event))) return false;
        if (event.name !== 'rotate_completed') return false;
        const seq = event.v?.command_seq;
        if (seq != null) return seq === w.seq;
        const target = event.v?.target_deg;
        return target == null || Math.abs(target - w.angle) < 0.01;
      }) ?? null
    );
  }

  /** The store subscription's predicate: never a timer or a poll. */
  function evaluate(w) {
    if (!waiting || waiting !== w || !w.ready) return;
    const state = store.state;

    if (state.bootId !== w.bootId) {
      endForReason(w, 'boot');
      return;
    }
    if (state.link !== 'open' || state.connection.state !== 'connected') {
      endForReason(w, 'link');
      return;
    }
    if (w.stopRequested) {
      endForReason(w, 'stop');
      return;
    }

    const activity = state.streams.activity?.v;
    if (activity && activity.id === MANUAL_DRIVE_ID && activity.stage === 'running') {
      w.sawRunning = true;
    }

    const result = state.commands[w.seq];
    if (result) {
      const accepted = result.outcome === 'acked' && (result.status == null || result.status === 'ok');
      if (!accepted) {
        endWait(w, resultReport(result));
        return;
      }
      w.acked = true;
    }

    const completion = completionFor(w, state);
    if (completion && w.acked) {
      const report = completionReport(w, completion);
      endWait(w, report, report.status === 'ok');
      return;
    }

    if (w.sawRunning && (!activity || activity.id !== MANUAL_DRIVE_ID || activity.stage !== 'running')) {
      endForReason(w, 'activity');
    }
  }

  /** The seam ticket 06 fills: the drive leg follows an ok (or skipped) rotate. */
  function afterRotate(w, report) {
    // Ticket 05 stops after the rotate. Ticket 06 sends `drive_distance` here
    // only for an `ok` completion and reports a busy second claim without retry.
    return report;
  }

  function go() {
    if (waiting) {
      announce('A rotation is already in progress.', 'warn');
      return waiting.promise;
    }
    const state = store.state;
    const committed = state.drive?.plan?.target;
    if (!committed) {
      announce('Choose a target point first.', 'warn');
      return Promise.resolve(null);
    }
    if (!controlEnabled(state)) {
      announce('Control is unavailable while the link or robot state is unknown.', 'warn');
      return Promise.resolve(null);
    }
    if (activityBusy(state)) {
      announce('Another Activity is running.', 'warn');
      return Promise.resolve(null);
    }

    const pose = latestPose(state);
    const leg = planLeg(pose, committed);
    const plan = {
      ...leg,
      target: { x: committed.x, y: committed.y },
      pose: { x: pose.x, y: pose.y, yaw: pose.yaw },
    };

    if (!leg.needs_rotate) {
      const report = {
        kind: 'info',
        title: 'No rotation needed',
        text: 'The bearing rounds to a zero centidegree, so no rotate is sent.',
        status: 'ok',
        target_deg: 0,
        actual_deg: 0,
        reason: 'already aligned',
      };
      setDrive({ plan, stage: 'idle', report, sequence: null });
      announce('No rotation needed.', 'info');
      return Promise.resolve(afterRotate({ plan, angle: 0, seq: null }, report));
    }

    // The sent value is the rounded centidegree, the Relay's own precision.
    const sentAngle = centidegrees(leg.angle_deg) / 100;
    const w = {
      plan,
      angle: sentAngle,
      bootId: state.bootId,
      seq: null,
      ready: false,
      acked: false,
      sawRunning: false,
      stopRequested: false,
      startKeys: new Set(state.events.map(eventKey)),
      resolve: null,
      unsubscribe: null,
    };
    w.promise = new Promise((resolve) => {
      w.resolve = resolve;
    });

    setDrive({ plan, stage: 'rotate', report: null, sequence: null });
    waiting = w;
    w.unsubscribe = store.subscribe(() => evaluate(w));

    // Subscribe before sending so even a sender that dispatches synchronously
    // cannot deliver the ACK or Event before the wait is armed.
    const seq = send('rotate_by', { angle_deg: sentAngle, speed: ROTATE_SPEED });
    w.seq = seq;
    w.ready = true;
    setDrive({ plan, stage: 'rotate', report: null, sequence: seq });
    evaluate(w);
    return w.promise;
  }

  /**
   * Ticket 05 only ends the wait: the contextual `stop_motion`/`stop_procedure`
   * command and its ACK copy follow in ticket 06.
   */
  function stop() {
    if (!waiting) {
      announce('Stop is not available yet.', 'warn');
      return;
    }
    waiting.stopRequested = true;
    evaluate(waiting);
  }

  return {
    preview,
    pick,
    numeric,
    cancel,
    dismiss,
    go,
    stop,
    now,
    send,
  };
}
