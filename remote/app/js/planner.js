// The point-and-go planner: it owns the Target Point and the two-leg state
// machine, with the command sender and the clock injected (ADR-0043).
//
// Ticket 05/06 fills the legs; this module already holds the preview and numeric
// entry points so the shell can show the same Target Point either way.

import { planLeg } from './geometry.js';
import { emptyDrive, latestPose } from './store.js';

/** Creates the planner over a store, a command sender and a clock. */
export function createPlanner({ store, send = () => 0, now = () => Date.now(), announce = () => {} } = {}) {
  function setDrive(drive) {
    store.dispatch({ type: '@drive', drive });
  }

  function preview(target) {
    const pose = latestPose(store.state);
    const leg = planLeg(pose, target);
    setDrive({
      plan: { ...leg, target, pose: { x: pose.x, y: pose.y, yaw: pose.yaw } },
      stage: 'idle',
      report: null,
      sequence: null,
    });
  }

  function numeric({ bearing_deg, distance_cm }) {
    const pose = latestPose(store.state);
    const angle = (pose.yaw + bearing_deg) * (Math.PI / 180);
    preview({
      x: pose.x + distance_cm * Math.cos(angle),
      y: pose.y + distance_cm * Math.sin(angle),
    });
  }

  function cancel() {
    setDrive(emptyDrive());
  }

  function dismiss() {
    setDrive(emptyDrive());
  }

  function go() {
    announce('Point-and-go is not available yet.', 'warn');
  }

  function stop() {
    announce('Stop is not available yet.', 'warn');
  }

  return {
    preview,
    numeric,
    cancel,
    dismiss,
    go,
    stop,
    now,
    send,
  };
}
