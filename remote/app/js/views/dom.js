// Small DOM helpers shared by the view modules. No framework, no virtual DOM.

/** Returns the element with `id`, or `null`. */
export function byId(id, root = document) {
  return root.getElementById(id);
}

/** Sets `textContent` only when it changes, so the DOM stays quiet. */
export function setText(node, value) {
  if (!node) return;
  const text = value == null ? '' : String(value);
  if (node.textContent !== text) node.textContent = text;
}

/** Sets a boolean attribute-like `hidden` only when it changes. */
export function setHidden(node, hidden) {
  if (!node) return;
  const next = Boolean(hidden);
  if (node.hidden !== next) node.hidden = next;
}

/** Toggles a class only when it changes. */
export function setClass(node, name, on) {
  if (!node) return;
  node.classList.toggle(name, Boolean(on));
}

/** Sets an attribute only when it changes. */
export function setAttr(node, name, value) {
  if (!node) return;
  const next = value == null ? null : String(value);
  if (next == null) {
    if (node.hasAttribute(name)) node.removeAttribute(name);
  } else if (node.getAttribute(name) !== next) {
    node.setAttribute(name, next);
  }
}

/** Sets a data attribute only when it changes. */
export function setData(node, name, value) {
  setAttr(node, `data-${name}`, value);
}

/** Replaces every child with one text node. */
export function setPlainText(node, text) {
  if (!node) return;
  node.replaceChildren(document.createTextNode(text ?? ''));
}

/** Formats an unknown-tolerant value with an explicit placeholder. */
export function orUnknown(value, suffix = '') {
  return value == null ? '—' : `${value}${suffix}`;
}

/** Formats a millisecond age as a short, honest string. */
export function formatAge(ts, now) {
  if (ts == null) return 'age unknown';
  const age = Math.max(0, now - ts);
  if (age < 1000) return `${Math.round(age)} ms old`;
  return `${(age / 1000).toFixed(age < 10_000 ? 1 : 0)} s old`;
}

/** Creates one element with optional class and text. */
export function element(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text != null) node.textContent = text;
  return node;
}
