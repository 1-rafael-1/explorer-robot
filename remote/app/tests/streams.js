// Rendering tests for the Perception row: per-sensor labels, the aggregate
// fallback for an older robot, and the both-drop case.

import { renderStreams } from '../js/views/streams.js';
import { assert, assertEqual, autorun, test } from './harness.js';

/** A Relay-shaped state with one perception value. */
function state(perception) {
  return {
    clockOffset: 0,
    streams: {
      perception: { v: perception, ts: 1000 },
      motion: { v: { left: 0, right: 0 }, ts: 1000 },
      power: { v: { level: 100, voltage_mv: 12000 }, ts: 1000 },
    },
  };
}

/** Render one perception value and return its nodes. */
function render(perception) {
  renderStreams(state(perception));
  return {
    obstacle: document.getElementById('perception-obstacle'),
    aggregate: document.getElementById('perception-floor-drop'),
    front: document.getElementById('perception-floor-drop-front'),
    rear: document.getElementById('perception-floor-drop-rear'),
  };
}

test('a clear obstacle is named, never a bare Clear', () => {
  const ui = render({
    obstacle: false,
    floor_drop: false,
    floor_drop_front: false,
    floor_drop_rear: false,
  });

  assertEqual(ui.obstacle.textContent, 'Obstacle: clear');
});

test('a current robot names the sensor that dropped', () => {
  const ui = render({
    obstacle: true,
    floor_drop: true,
    floor_drop_front: true,
    floor_drop_rear: false,
  });

  assertEqual(ui.front.textContent, 'Front: drop');
  assertEqual(ui.rear.textContent, 'Rear: floor');
  assertEqual(ui.front.dataset.detected, 'true');
  assertEqual(ui.rear.dataset.detected, 'false');
  assert(ui.aggregate.hidden, 'the aggregate line stays hidden when the split is known');
  assert(!ui.front.hidden && !ui.rear.hidden, 'both sensor spans are shown');
});

test('both sensors dropping shows both drops', () => {
  const ui = render({
    obstacle: false,
    floor_drop: true,
    floor_drop_front: true,
    floor_drop_rear: true,
  });

  assertEqual(ui.front.textContent, 'Front: drop');
  assertEqual(ui.rear.textContent, 'Rear: drop');
});

test('an aggregate-only peer still shows the floor drop', () => {
  const ui = render({
    obstacle: false,
    floor_drop: true,
    floor_drop_front: null,
    floor_drop_rear: null,
  });

  assertEqual(ui.aggregate.textContent, 'Floor drop: detected');
  assert(!ui.aggregate.hidden, 'the aggregate line is shown');
  assert(ui.front.hidden && ui.rear.hidden, 'the per-sensor spans are hidden');
});

test('a missing perception stream shows unknown sensors', () => {
  const ui = render({
    obstacle: null,
    floor_drop: null,
    floor_drop_front: null,
    floor_drop_rear: null,
  });

  assertEqual(ui.obstacle.textContent, 'Obstacle: unknown');
  assertEqual(ui.front.textContent, 'Front: unknown');
  assertEqual(ui.rear.textContent, 'Rear: unknown');
  assert(ui.aggregate.hidden, 'the aggregate line is hidden');
});

autorun();
