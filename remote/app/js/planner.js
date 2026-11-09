// The point-and-go planner: the Target Point, the two-leg state machine, the
// completion wait and the context-sensitive Stop, with the command sender and
// the clock injected (ADR-0043).
//
// Go sends `rotate_by` only when the bearing rounds to a non-zero centidegree,
// then `drive_distance`. That is two commands and two claims: another operator
// can take the robot between the legs, and the drive then comes back `busy` and
// is reported without a retry. An ACK is acceptance, never the outcome; the
// completion Event is. No Volatile Command is auto-retried, so an
// `indeterminate` or `timeout` ACK is unknown and the operator re-issues.
//
// There is no distance cap, scan clamp, short-leg refusal, beyond-range warning,
// age gate or fixed completion timeout: the robot owns motion safety, and the
// wait ends on the completion Event, link loss, a Boot Id change, Stop or the
// Activity leaving running. A null-attribution completion is matched by leg,
// target and the one in-flight plan.

import { RAD, planLeg, pointInFootprint } from './geometry.js';
import { DRIVE_SPEED, BOOTING_ID, MANUAL_DRIVE_ID, ROTATE_SPEED, centidegrees } from './protocol.js';
import {
  activityBusy,
  controlEnabled,
  driveState,
  emptyDrive,
  eventKey,
  latestPose,
  liveActivityId,
  outcomeUnknown,
} from './store.js';

/** Formats an angle with one decimal, or an em dash when it is unknown. */
function angleText(value) {
  return Number.isFinite(value) ? `${Number(value).toFixed(1)}°` : '—';
}

/** Creates the planner over a store, a command sender and a clock. */
export function createPlanner({ store, send = () => 0, now = () => Date.now(), announce = () => {} } = {}) {
  // The one in-flight plan wait, or `null`. It spans the rotate and the drive.
  let waiting = null;
  // The one in-flight Stop wait, or `null`. Stop is idempotent but not stacked.
  let stopWaiting = null;

  function setDrive(drive) {
    store.dispatch({ type: '@drive', drive });
  }

  /** Whether a plan or a stop is in flight, so a new intent must wait. */
  function busy() {
    return waiting != null || stopWaiting != null;
  }

  /** Commits a Target Point and draws its preview from the current Pose. */
  function preview(target) {
    if (busy()) {
      announce('Wait for the robot to finish before choosing another point.', 'warn');
      return false;
    }
    if (!target || !Number.isFinite(target.x) || !Number.isFinite(target.y)) {
      announce('Tap a point on the radar or enter a bearing and distance.', 'warn');
      return false;
    }
    const pose = latestPose(store.state);
    const leg = planLeg(pose, target);
    setDrive(
      driveState(
        {
          ...leg,
          target: { x: target.x, y: target.y },
          pose: { x: pose.x, y: pose.y, yaw: pose.yaw },
        },
        'idle',
      ),
    );
    return true;
  }

  /**
   * Handles a radar tap. The drawn robot footprint is the only minimum: a tap
   * inside it produces no command, while a deliberately short leg outside it is
   * allowed (ADR-0043).
   */
  function pick(point, { inside_robot = false, scale = 1 } = {}) {
    if (busy()) {
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
    if (busy()) {
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

  /** The status→tone cascade shared by both completion reports. */
  function completionTone(status) {
    return status === 'ok' ? 'success' : status === 'cancelled' ? 'warn' : 'bad';
  }

  /** The status→title cascade for a rotate or drive completion. */
  function completionTitle(leg, status) {
    const subject = leg === 'drive' ? 'Drive' : 'Rotation';
    if (status === 'ok') return `${subject} complete`;
    if (status === 'cancelled') return leg === 'drive' ? 'Drive stopped' : 'Rotation cancelled';
    return `${subject} failed`;
  }

  /** Builds the report for a rotate completion Event. */
  function completionReport(w, event) {
    const value = event.v ?? {};
    const status = value.status ?? 'ok';
    const target = Number.isFinite(value.target_deg) ? value.target_deg : w.angle;
    const error = Number.isFinite(value.error_deg) ? value.error_deg : 0;
    const actual = target + error;
    const reason = value.detail || (status === 'ok' ? 'done' : status);
    return {
      kind: completionTone(status),
      title: completionTitle('rotate', status),
      text: `Target ${angleText(target)}, actual ${angleText(actual)} (${reason}).`,
      status,
      target_deg: target,
      actual_deg: actual,
      reason,
    };
  }

  /** Builds the report for a drive completion Event: target, actual and reason. */
  function driveCompletionReport(w, event) {
    const value = event.v ?? {};
    const status = value.status ?? 'ok';
    const target = Number.isFinite(value.target_cm) ? value.target_cm : w.target_cm;
    const actual = Number.isFinite(value.actual_cm) ? value.actual_cm : null;
    const reason = value.detail || (status === 'ok' ? 'done' : status);
    const actualText = actual == null ? '—' : `${actual} cm`;
    return {
      kind: completionTone(status),
      title: completionTitle('drive', status),
      text: `Target ${target} cm, actual ${actualText} (${reason}).`,
      status,
      target_cm: target,
      actual_cm: actual,
      reason,
    };
  }

  /** Builds the report for a rejected, indeterminate or timed-out ACK. */
  function resultReport(w, result) {
    const leg = w.leg === 'drive' ? 'drive' : 'rotation';
    const Leg = w.leg === 'drive' ? 'Drive' : 'Rotation';
    const unknown = result.outcome === 'indeterminate' || result.outcome === 'timeout';
    const reason = result.detail || result.status || result.outcome || 'rejected';
    if (unknown) {
      return {
        kind: 'unknown',
        title: 'Outcome unknown',
        text: `The ${leg} ${result.outcome === 'timeout' ? 'timed out' : 'outcome is unknown'}; re-issue when ready.`,
        status: result.status ?? null,
        reason,
      };
    }
    // A busy second claim is exactly what the two-claim design allows: report
    // it and never retry.
    if (w.leg === 'drive' && result.status === 'busy') {
      return {
        kind: 'busy',
        title: 'Rotation done, drive not sent',
        text:
          result.detail ||
          'The robot was busy when the drive was offered, so the drive was not accepted. No retry was sent.',
        status: 'busy',
        reason,
      };
    }
    return {
      kind: 'bad',
      title: `${Leg} rejected`,
      text: result.detail || `The robot rejected the ${leg} (${result.status ?? 'rejected'}).`,
      status: result.status ?? null,
      reason,
    };
  }

  /** Builds the "Stopped" report: shown only on an acked/ok Stop ACK. */
  function stoppedReport() {
    return {
      kind: 'success',
      title: 'Stopped',
      text: 'The robot acknowledged the stop.',
      status: 'ok',
      reason: 'stopped',
    };
  }

  /** Builds the honest fallback: the app cannot claim a stop without the ACK. */
  function notConfirmedReport(result) {
    const reason =
      typeof result === 'string'
        ? result
        : result == null
          ? 'not acknowledged'
          : result.detail || result.status || result.outcome || 'not acknowledged';
    return {
      kind: 'warn',
      title: 'Stop not confirmed',
      text: 'Not confirmed — the robot stops on link loss.',
      status: typeof result === 'string' ? null : (result?.status ?? null),
      reason,
    };
  }

  /** Builds the honest fallback when an Event gap or overflow taints the leg. */
  function droppedReport(w) {
    const drive = w.leg === 'drive';
    const subject = drive ? `Target ${w.target_cm} cm` : `Target ${angleText(w.angle)}`;
    return {
      kind: 'unknown',
      title: 'Outcome unknown',
      text: `${subject} was not confirmed. Events were dropped; re-issue when ready.`,
      reason: 'events dropped',
    };
  }

  /** Ends the in-flight plan wait, renders the outcome and resolves `go()`. */
  function endWait(w, report) {
    if (!w || waiting !== w) return;
    waiting = null;
    w.unsubscribe?.();
    // A Boot Id reset clears the store's plan; keep that truth instead of
    // resurrecting a target from a robot epoch that no longer exists.
    const plan = store.state.drive?.plan ?? null;
    setDrive(driveState(plan, 'idle', report, w.seq));
    w.resolve(report);
  }

  function endForReason(w, reason, detail) {
    const drive = w.leg === 'drive';
    const subject = drive ? `Target ${w.target_cm} cm` : `Target ${angleText(w.angle)}`;
    if (reason === 'link') {
      endWait(w, {
        kind: 'warn',
        title: 'Link lost',
        text: `${subject} was not confirmed. The ${drive ? 'drive' : 'rotation'} outcome is unknown; the robot stops on link loss.`,
        reason: 'link lost',
      });
    } else if (reason === 'boot') {
      endWait(w, {
        kind: 'unknown',
        title: 'Outcome unknown',
        text: `${subject} was not confirmed. The robot restarted (new Boot Id). Check it before re-issuing.`,
        reason: 'Boot Id changed',
      });
    } else {
      endWait(w, {
        kind: 'warn',
        title: drive ? 'Drive ended' : 'Rotation ended',
        text: `${subject} ${detail ?? `The Activity left running before the ${drive ? 'drive' : 'rotation'} completed.`}`,
        reason: 'activity ended',
      });
    }
  }

  /**
   * Finds the completion Event for this leg. A null attribution is matched by
   * leg, target and the one in-flight plan; a completion from the other leg is
   * never accepted.
   */
  function completionFor(w, state) {
    const name = w.leg === 'drive' ? 'drive_distance_completed' : 'rotate_completed';
    return (
      state.events.find((event) => {
        if (w.startKeys.has(eventKey(event))) return false;
        if (event.name !== name) return false;
        const seq = event.v?.command_seq;
        if (seq != null) return seq === w.seq;
        if (w.leg === 'drive') {
          const target = event.v?.target_cm;
          return target != null && Math.abs(target - w.target_cm) <= 1;
        }
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

    const activity = state.streams.activity?.v;
    if (activity && activity.id === MANUAL_DRIVE_ID && activity.stage === 'running') {
      w.sawRunning = true;
    }

    const result = w.seq == null ? null : state.commands[w.seq];
    if (result) {
      const accepted = result.outcome === 'acked' && (result.status == null || result.status === 'ok');
      if (!accepted) {
        endWait(w, resultReport(w, result));
        return;
      }
      w.acked = true;
    }

    const completion = completionFor(w, state);
    if (completion && w.acked) {
      const report =
        w.leg === 'drive' ? driveCompletionReport(w, completion) : completionReport(w, completion);
      if (w.leg === 'rotate') {
        if (afterRotate(w, report)) return;
        endWait(w, report);
        return;
      }
      endWait(w, report);
      return;
    }

    // A gap or overflow after this leg started, with no matching completion,
    // means the Events that would prove the outcome were dropped. Never success,
    // and never a retry.
    if (outcomeUnknown(state, w.startedAt) && !completion) {
      endWait(w, droppedReport(w));
      return;
    }

    if (w.sawRunning && (!activity || activity.id !== MANUAL_DRIVE_ID || activity.stage !== 'running')) {
      endForReason(w, 'activity');
    }
  }

  /**
   * Continues a successful rotation into the drive leg. A failed or cancelled
   * rotation never sends a drive; the rotation's own report ends the wait.
   */
  function afterRotate(w, report) {
    if (report.status !== 'ok') return false;
    startDrive(w);
    return true;
  }

  /** Sends `drive_distance` as the second claim and waits for its completion. */
  function startDrive(w) {
    if (!waiting || waiting !== w) return;
    const state = store.state;
    if (!controlEnabled(state)) {
      endWait(w, {
        kind: 'warn',
        title: 'Rotation done, drive not sent',
        text: 'The link was lost before the drive could be sent. The robot stops on link loss.',
        reason: 'link lost',
      });
      return;
    }

    // The drive is a fresh claim: its own ACK, its own completion.
    w.leg = 'drive';
    w.acked = false;
    w.sawRunning = false;
    w.seq = null;
    w.ready = false;
    w.target_cm = Math.round(w.plan.distance_cm);
    w.startedAt = now();
    w.startKeys = new Set(state.events.map(eventKey));

    setDrive(driveState(w.plan, 'drive'));
    // Subscribe already exists; arm the leg before the sender can answer.
    const seq = send('drive_distance', { distance_cm: w.target_cm, speed: DRIVE_SPEED });
    w.seq = seq;
    w.ready = true;
    setDrive(driveState(w.plan, 'drive', null, seq));
    evaluate(w);
  }

  function makeWait(state, plan, angle) {
    const w = {
      plan,
      leg: 'rotate',
      angle,
      target_cm: Math.round(plan.distance_cm),
      bootId: state.bootId,
      seq: null,
      ready: false,
      acked: false,
      sawRunning: false,
      startedAt: now(),
      startKeys: new Set(state.events.map(eventKey)),
      resolve: null,
      unsubscribe: null,
    };
    w.promise = new Promise((resolve) => {
      w.resolve = resolve;
    });
    return w;
  }

  function go() {
    if (waiting) {
      announce('A drive is already in progress.', 'warn');
      return waiting.promise;
    }
    if (stopWaiting) {
      announce('Wait for the stop to be confirmed.', 'warn');
      return Promise.resolve(null);
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

    // A bearing that rounds to a zero centidegree skips the rotate and sends
    // the drive directly (ADR-0043).
    if (!leg.needs_rotate) {
      const w = makeWait(state, plan, 0);
      waiting = w;
      w.unsubscribe = store.subscribe(() => evaluate(w));
      w.ready = true;
      startDrive(w);
      return w.promise;
    }

    // The sent value is the rounded centidegree, the Relay's own precision.
    const sentAngle = centidegrees(leg.angle_deg) / 100;
    const w = makeWait(state, plan, sentAngle);
    setDrive(driveState(plan, 'rotate'));
    waiting = w;
    w.unsubscribe = store.subscribe(() => evaluate(w));

    // Subscribe before sending so even a sender that dispatches synchronously
    // cannot deliver the ACK or Event before the wait is armed.
    const seq = send('rotate_by', { angle_deg: sentAngle, speed: ROTATE_SPEED });
    w.seq = seq;
    w.ready = true;
    setDrive(driveState(plan, 'rotate', null, seq));
    evaluate(w);
    return w.promise;
  }

  /** The contextual stop command: motion for a Manual Drive, procedure for a claim. */
  function stopCommandFor(state, motion) {
    if (motion) return { name: 'stop_motion', args: { brake: true } };
    const id = liveActivityId(state);
    // Booting is Activity-only, not a startable Procedure; Stop is idempotent.
    if (id != null && id !== BOOTING_ID && id !== MANUAL_DRIVE_ID) {
      return { name: 'stop_procedure', args: { id } };
    }
    return { name: 'stop_motion', args: { brake: true } };
  }

  /** Resolves the Stop wait with the ACK's verdict, never before it. */
  function finishStop(entry, report) {
    if (!stopWaiting || stopWaiting !== entry) return;
    stopWaiting = null;
    entry.unsubscribe?.();
    // Respect a Boot Id reset that cleared the plan while the stop was in flight.
    const plan = store.state.drive?.plan ?? null;
    setDrive(driveState(plan, 'idle', report, entry.seq));
    entry.resolve(report);
    // A Stop during a plan ends that plan's wait too.
    entry.motion?.resolve(report);
  }

  function evaluateStop(entry) {
    if (!stopWaiting || stopWaiting !== entry || !entry.ready) return;
    const state = store.state;
    if (state.bootId !== entry.bootId || state.link !== 'open' || state.connection.state !== 'connected') {
      finishStop(entry, notConfirmedReport('link lost'));
      return;
    }
    const result = entry.seq == null ? null : state.commands[entry.seq];
    if (!result) return;
    const stopped = result.outcome === 'acked' && (result.status == null || result.status === 'ok');
    finishStop(entry, stopped ? stoppedReport() : notConfirmedReport(result));
  }

  /**
   * Stops what is actually running (ADR-0043). Stop ends a motion wait, then
   * sends the contextual command and waits for its `command_result`; "Stopped"
   * appears only on an acked/ok ACK.
   */
  function stop() {
    const state = store.state;
    if (!controlEnabled(state)) {
      announce('Stop is unavailable while the link or robot state is unknown.', 'warn');
      return null;
    }
    if (stopWaiting) {
      announce('A stop is already in progress.', 'info');
      return stopWaiting.promise;
    }

    const motion = waiting;
    const context = stopCommandFor(state, motion);
    const plan = state.drive?.plan ?? motion?.plan ?? null;
    // Keep the leg's stage so the store still sees the app's own claim while
    // the stop is in flight; `stopping` is the view's extra signal.
    const stage = motion ? motion.leg : 'idle';

    if (motion) {
      waiting = null;
      motion.unsubscribe?.();
    }

    const entry = {
      motion,
      bootId: state.bootId,
      seq: null,
      ready: false,
      resolve: null,
      unsubscribe: null,
    };
    entry.promise = new Promise((resolve) => {
      entry.resolve = resolve;
    });
    stopWaiting = entry;

    setDrive({ ...driveState(plan, stage), stopping: true });
    entry.unsubscribe = store.subscribe(() => evaluateStop(entry));
    const seq = send(context.name, context.args);
    entry.seq = seq;
    entry.ready = true;
    setDrive({ ...driveState(plan, stage, null, seq), stopping: true });
    evaluateStop(entry);
    return entry.promise;
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
