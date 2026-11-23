//! The Bridge's station link: boot, association, reconnect and retry (ADR-0058).
//!
//! `RadioSession` is the pure, HAL-free half of the Bridge's radio policy. One call per step takes
//! the observed radio facts — the caller's `u32` milliseconds, whether the controller reports an
//! association and the station's IPv4 address — plus the results of the previous step's blocking
//! effects. It returns typed actions for the firmware to execute, typed events to log and an
//! edge-triggered publication for the shared statics.
//!
//! This slice owns the station link only: boot from a Credential Record, adopting an existing
//! association, link loss and reconnect, the DHCP timeout, the join-failure backoff and the
//! missing-credentials warning. Setup Mode, provisioning, portal status and the counters are later
//! slices of the same session (ADR-0058).
//!
//! The session owns no HAL, socket, clock or allocator and never panics: everything it needs is
//! passed in, and its time comparisons wrap like the rest of `bridge-core`.

use link_protocol::StationSsid;

use crate::{portal::Credentials, record::CredentialRecord};

/// The shortest reconnect wait after a link loss or a failed join (ADR-0036's 1 s).
pub const MIN_BACKOFF_MS: u32 = 1_000;

/// The longest reconnect wait (ADR-0036's 60 s).
pub const MAX_BACKOFF_MS: u32 = 60_000;

/// How long an association may go without a DHCP lease before it is recycled (ADR-0036's 30 s).
pub const DHCP_TIMEOUT_MS: u32 = 30_000;

/// How often the missing-credentials warning repeats while the Bridge is unprovisioned.
pub const MISSING_CREDENTIALS_WARNING_MS: u32 = 60_000;

/// The number of actions one step can return; further actions are refused and flagged.
const ACTION_CAPACITY: usize = 8;

/// The number of events one step can return; further events are refused and flagged.
const EVENT_CAPACITY: usize = 8;

/// Half a `u32`'s range, used to compare timestamps across a wrap.
const HALF_RANGE: u32 = 1 << 31;

/// Whether `now_ms` has reached `deadline_ms`, tolerating a `u32` wrap.
const fn reached(now_ms: u32, deadline_ms: u32) -> bool {
    now_ms.wrapping_sub(deadline_ms) < HALF_RANGE
}

/// Doubles the reconnect backoff, capped at [`MAX_BACKOFF_MS`].
const fn next_backoff(backoff_ms: u32) -> u32 {
    let doubled = backoff_ms.saturating_mul(2);
    if doubled > MAX_BACKOFF_MS {
        MAX_BACKOFF_MS
    } else {
        doubled
    }
}

/// What the caller observed about the radio at one step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Observation {
    /// The caller's monotonic milliseconds.
    pub now_ms: u32,
    /// Whether the radio controller reports an association.
    pub controller_connected: bool,
    /// The station's IPv4 address, once DHCP has leased one.
    pub station_ip: Option<[u8; 4]>,
}

/// The result of a join action the session asked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JoinResult {
    /// The station associated.
    Connected,
    /// The access point rejected the passphrase.
    WrongPassword,
    /// The attempt failed for any other reason, including a timeout.
    Failed,
}

/// An ordered input for one step: the result of a blocking action the session asked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Input {
    /// The result of the previous step's join action.
    Join(JoinResult),
}

/// Why a join attempt failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JoinFailure {
    /// The access point rejected the passphrase.
    WrongPassword,
    /// The attempt failed for any other reason, including a timeout.
    Failed,
}

/// One blocking effect the caller must run to completion before the next step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    /// Tear down the current association before rejoining.
    Disconnect,
    /// Enter station mode with these credentials, keeping the setup AP up.
    ApplyStation(
        /// The credentials to associate with.
        Credentials,
    ),
    /// Enter AP-only Setup Mode with a parked, never-associated station.
    ApplySetupParked,
    /// Scan for and join the Station Network with this SSID.
    Join(
        /// The target network's SSID.
        StationSsid,
    ),
}

/// A transition the caller may log or react to; no event carries a secret.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// The station lost its association.
    LinkLost,
    /// The station associated but did not receive a DHCP lease in time.
    DhcpTimeout,
    /// A join attempt associated successfully.
    Associated,
    /// A join attempt failed; the retry waits for the reported backoff.
    JoinFailed {
        /// Why the join failed.
        kind: JoinFailure,
        /// How long the session waits before the next join.
        retry_in_ms: u32,
    },
    /// The Bridge has no credentials, so the setup portal is its only network surface.
    MissingCredentials,
}

/// The derived values that changed this step and must be written to the shared statics.
///
/// Every field is `None` unless the value changed, so the portal worker's optimistic status hint
/// is not clobbered. [`Publication::station_ssid_cleared`] distinguishes "the Station Network is
/// still unset" from "the Station Network was cleared this step", because
/// [`Publication::station_ssid`] alone cannot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Publication {
    /// The provisioned flag, when it changed.
    pub provisioned: Option<bool>,
    /// The Station Network's SSID, when it changed to a named network.
    pub station_ssid: Option<StationSsid>,
    /// Whether the Station Network's SSID changed to unset this step.
    pub station_ssid_cleared: bool,
    /// The station-up flag, when it changed.
    pub station_up: Option<bool>,
}

/// A fixed-capacity, saturating list of items.
///
/// The crate has no allocator, so one step's output is bounded. A push onto a full list keeps the
/// items already stored and records the overflow, so a refused transition is visible instead of
/// being dropped silently.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Bounded<T, const N: usize> {
    /// The stored items; the first `len` are valid.
    items: [Option<T>; N],
    /// The number of valid items.
    len: usize,
    /// Whether a push was refused because the list was full.
    overflowed: bool,
}

impl<T, const N: usize> Bounded<T, N> {
    /// Creates an empty list.
    #[allow(
        clippy::large_stack_frames,
        reason = "a fixed-capacity list holds every slot inline by design; there is no allocator to move them to"
    )]
    const fn new() -> Self {
        Self {
            items: [const { None }; N],
            len: 0,
            overflowed: false,
        }
    }

    /// Appends `item`, or records the overflow when the list is full.
    fn push(&mut self, item: T) {
        if let Some(slot) = self.items.get_mut(self.len) {
            *slot = Some(item);
            self.len += 1;
        } else {
            self.overflowed = true;
        }
    }

    /// Returns the number of stored items.
    const fn len(&self) -> usize {
        self.len
    }

    /// Returns whether the list is empty.
    const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns whether a push was refused because the list was full.
    const fn overflowed(&self) -> bool {
        self.overflowed
    }

    /// Iterates the stored items in push order.
    fn iter(&self) -> core::iter::Flatten<core::slice::Iter<'_, Option<T>>> {
        self.items.get(..self.len).unwrap_or(&[]).iter().flatten()
    }
}

/// The actions one step produced: fixed capacity, saturating, allocation-free.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ActionSet {
    /// The stored actions, in execution order.
    items: Bounded<Action, ACTION_CAPACITY>,
}

impl ActionSet {
    /// Creates an empty set.
    #[allow(
        clippy::large_stack_frames,
        reason = "a fixed-capacity action set holds every slot inline by design; there is no allocator to move them to"
    )]
    const fn new() -> Self {
        Self { items: Bounded::new() }
    }

    /// Appends an action, or records the overflow when the set is full.
    fn push(&mut self, action: Action) {
        self.items.push(action);
    }

    /// Returns the number of actions.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.items.len()
    }

    /// Returns whether the step produced no actions.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Returns whether an action was refused because the set was full.
    #[must_use]
    pub const fn overflowed(&self) -> bool {
        self.items.overflowed()
    }

    /// Iterates the actions in the order the executor must run them.
    pub fn iter(&self) -> core::iter::Flatten<core::slice::Iter<'_, Option<Action>>> {
        self.items.iter()
    }
}

impl<'a> IntoIterator for &'a ActionSet {
    type Item = &'a Action;
    type IntoIter = core::iter::Flatten<core::slice::Iter<'a, Option<Action>>>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

/// The events one step produced: fixed capacity, saturating, allocation-free.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EventSet {
    /// The stored events, in occurrence order.
    items: Bounded<Event, EVENT_CAPACITY>,
}

impl EventSet {
    /// Creates an empty set.
    const fn new() -> Self {
        Self { items: Bounded::new() }
    }

    /// Appends an event, or records the overflow when the set is full.
    fn push(&mut self, event: Event) {
        self.items.push(event);
    }

    /// Returns the number of events.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.items.len()
    }

    /// Returns whether the step produced no events.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Returns whether an event was refused because the set was full.
    #[must_use]
    pub const fn overflowed(&self) -> bool {
        self.items.overflowed()
    }

    /// Iterates the events in the order they happened.
    pub fn iter(&self) -> core::iter::Flatten<core::slice::Iter<'_, Option<Event>>> {
        self.items.iter()
    }
}

impl<'a> IntoIterator for &'a EventSet {
    type Item = &'a Event;
    type IntoIter = core::iter::Flatten<core::slice::Iter<'a, Option<Event>>>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

/// One step's output: the actions to execute, the events to log and the edges to publish.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Outcome {
    /// The actions to execute in order.
    pub actions: ActionSet,
    /// The transitions to log, in the order they happened.
    pub events: EventSet,
    /// The derived values that changed this step.
    pub publication: Publication,
}

impl Outcome {
    /// Creates an outcome with nothing to do.
    #[allow(
        clippy::large_stack_frames,
        reason = "the outcome owns its bounded action and event sets inline by design; there is no allocator to move them to"
    )]
    const fn new() -> Self {
        Self {
            actions: ActionSet::new(),
            events: EventSet::new(),
            publication: Publication::new(),
        }
    }
}

impl Publication {
    /// Creates a publication with no edges.
    const fn new() -> Self {
        Self {
            provisioned: None,
            station_ssid: None,
            station_ssid_cleared: false,
            station_up: None,
        }
    }
}

/// The Station Network SSID the session last published.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PublishedSsid {
    /// Nothing has been published yet.
    Unpublished,
    /// The unset SSID was published.
    Cleared,
    /// This named SSID was published.
    Named(StationSsid),
}

/// The Bridge's station link: association, reconnect, retry and the missing-credentials warning.
///
/// The session is built once at boot from the boot Credential Record and driven by the
/// supervisory loop. It owns the station link state and the Station Network's SSID; it never reads
/// a published value back.
#[derive(Clone, Copy, Debug)]
#[allow(
    clippy::large_stack_frames,
    reason = "the session holds the credential pair and the published edges inline by design; there is no allocator to move them to"
)]
pub struct RadioSession {
    /// The station credentials in force, `None` while the Bridge is unprovisioned.
    credentials: Option<Credentials>,
    /// Whether the station is associated.
    station_connected: bool,
    /// The station's last observed IPv4 address.
    station_ip: Option<[u8; 4]>,
    /// When the current association began, in the caller's millisecond scale.
    associated_at: u32,
    /// The earliest time the next join may be emitted.
    next_attempt: u32,
    /// The wait applied to the next join failure.
    backoff: u32,
    /// Whether a join action is awaiting its result.
    join_in_flight: bool,
    /// Whether a failed join requires the operator to act before a retry.
    retry_blocked: bool,
    /// The earliest time the missing-credentials warning may repeat.
    next_missing_warning: u32,
    /// The provisioned flag last published.
    published_provisioned: Option<bool>,
    /// The Station Network SSID last published.
    published_ssid: PublishedSsid,
    /// The station-up flag last published.
    published_station_up: Option<bool>,
}

impl RadioSession {
    /// Creates the session at boot from the Credential Record and returns its initial outcome.
    ///
    /// The record's credentials are kept only when they form a usable WPA2-PSK pair; anything else
    /// means unprovisioned, exactly as the record decoder's fail-closed rule. The initial outcome is
    /// pure: it carries the mode action, the initial publication and no I/O.
    #[must_use]
    #[allow(
        clippy::large_stack_frames,
        reason = "the initial outcome owns its bounded sets inline by design; there is no allocator to move them to"
    )]
    pub fn boot(record: Option<&CredentialRecord>, now_ms: u32) -> (Self, Outcome) {
        let credentials = record
            .and_then(|record| record.credentials)
            .filter(Credentials::is_valid);

        let mut session = Self {
            credentials,
            station_connected: false,
            station_ip: None,
            associated_at: now_ms,
            next_attempt: now_ms,
            backoff: MIN_BACKOFF_MS,
            join_in_flight: false,
            retry_blocked: false,
            next_missing_warning: now_ms,
            published_provisioned: None,
            published_ssid: PublishedSsid::Unpublished,
            published_station_up: None,
        };

        let mut outcome = Outcome::new();
        match credentials {
            Some(credentials) => outcome.actions.push(Action::ApplyStation(credentials)),
            None => outcome.actions.push(Action::ApplySetupParked),
        }
        session.publish(&mut outcome.publication);
        (session, outcome)
    }

    /// Processes one step: the join results first, then the link policy at `observation.now_ms`.
    ///
    /// The caller passes the result of the previous step's join, if any, and the radio facts it
    /// observed. Every action must be executed to completion before the next step; its result
    /// arrives as an input on the following step.
    #[must_use]
    #[allow(
        clippy::large_stack_frames,
        reason = "the outcome owns its bounded sets inline by design; there is no allocator to move them to"
    )]
    pub fn step(&mut self, observation: &Observation, inputs: &[Input]) -> Outcome {
        let now_ms = observation.now_ms;
        let mut outcome = Outcome::new();

        for input in inputs {
            let Input::Join(result) = *input;
            self.join_in_flight = false;
            match result {
                JoinResult::Connected => self.on_connected(now_ms, &mut outcome),
                JoinResult::WrongPassword => {
                    self.on_join_failed(JoinFailure::WrongPassword, now_ms, &mut outcome);
                }
                JoinResult::Failed => self.on_join_failed(JoinFailure::Failed, now_ms, &mut outcome),
            }
        }

        self.station_ip = observation.station_ip;
        self.apply_link_policy(observation.controller_connected, now_ms, &mut outcome);
        self.publish(&mut outcome.publication);
        outcome
    }

    /// Returns whether the Bridge has a usable Station Network credential pair.
    #[must_use]
    pub const fn provisioned(&self) -> bool {
        self.credentials.is_some()
    }

    /// Returns the Station Network's SSID, or `None` while the Bridge is unprovisioned.
    #[must_use]
    pub fn station_ssid(&self) -> Option<StationSsid> {
        self.credentials
            .and_then(|credentials| StationSsid::new(credentials.ssid()))
    }

    /// Returns whether the station is associated and holds a DHCP address.
    #[must_use]
    pub const fn station_up(&self) -> bool {
        self.station_connected && self.station_ip.is_some()
    }

    /// Returns whether the last join failed and the operator must act before a retry.
    ///
    /// This slice records the block but gates retries on time alone: there is no Setup Mode window
    /// yet, so a block must not stop the steady-state backoff. Setup Mode suppresses retries inside
    /// an open window in a later slice (ADR-0058).
    #[must_use]
    pub const fn retry_blocked(&self) -> bool {
        self.retry_blocked
    }

    /// Handles a successful join: the station is associated and the backoff is back at its minimum.
    fn on_connected(&mut self, now_ms: u32, outcome: &mut Outcome) {
        self.station_connected = true;
        self.associated_at = now_ms;
        self.backoff = MIN_BACKOFF_MS;
        self.retry_blocked = false;
        outcome.events.push(Event::Associated);
    }

    /// Handles a failed join: reports the wait that applies, then doubles the one after it.
    fn on_join_failed(&mut self, kind: JoinFailure, now_ms: u32, outcome: &mut Outcome) {
        outcome.events.push(Event::JoinFailed {
            kind,
            retry_in_ms: self.backoff,
        });
        self.next_attempt = now_ms.wrapping_add(self.backoff);
        self.backoff = next_backoff(self.backoff);
        self.retry_blocked = true;
    }

    /// Applies the link policy at one timestamp.
    fn apply_link_policy(&mut self, observed_connected: bool, now_ms: u32, outcome: &mut Outcome) {
        // The controller's association state as it will be after this step's actions run. A
        // `Disconnect` action clears the association before the next step, so the rest of this step
        // must see the link as down. The blocking firmware loop observes exactly that.
        let mut controller_connected = observed_connected;

        if self.station_connected && !controller_connected {
            self.station_connected = false;
            outcome.events.push(Event::LinkLost);
            self.arm_retry(now_ms);
        } else if self.station_connected
            && self.station_ip.is_none()
            && reached(now_ms, self.associated_at.wrapping_add(DHCP_TIMEOUT_MS))
        {
            outcome.events.push(Event::DhcpTimeout);
            outcome.actions.push(Action::Disconnect);
            self.station_connected = false;
            self.arm_retry(now_ms);
            controller_connected = false;
        }

        // Dropping the station mode can leave an existing association in place; adopt it instead of
        // scanning and re-associating the network the station is already on.
        if self.credentials.is_some() && !self.station_connected && controller_connected {
            self.adopt(now_ms);
        }

        self.maybe_join(now_ms, outcome);
        self.maybe_warn_missing_credentials(now_ms, outcome);
    }

    /// Adopts an association the controller already holds.
    ///
    /// ADR-0058 preserves the firmware's quirk here: the retry schedule and the operator block are
    /// deliberately left alone, so an adoption does not forgive a failed join.
    const fn adopt(&mut self, now_ms: u32) {
        self.station_connected = true;
        self.associated_at = now_ms;
    }

    /// Emits the join action when the station is down, no result is pending and the retry gate is
    /// open.
    fn maybe_join(&mut self, now_ms: u32, outcome: &mut Outcome) {
        if self.station_connected || self.join_in_flight || !reached(now_ms, self.next_attempt) {
            return;
        }
        let Some(ssid) = self.station_ssid() else {
            return;
        };
        outcome.actions.push(Action::Join(ssid));
        self.join_in_flight = true;
    }

    /// Repeats the missing-credentials warning on its bounded cadence.
    fn maybe_warn_missing_credentials(&mut self, now_ms: u32, outcome: &mut Outcome) {
        if self.credentials.is_some() || !reached(now_ms, self.next_missing_warning) {
            return;
        }
        outcome.events.push(Event::MissingCredentials);
        self.next_missing_warning = now_ms.wrapping_add(MISSING_CREDENTIALS_WARNING_MS);
    }

    /// Restarts the reconnect schedule: wait the minimum backoff, then double the one after it.
    ///
    /// This is the firmware's reset-before-use pair expressed once: `next_attempt` gets the
    /// minimum, and the stored backoff becomes what a failure after that attempt would use.
    const fn arm_retry(&mut self, now_ms: u32) {
        self.backoff = MIN_BACKOFF_MS;
        self.next_attempt = now_ms.wrapping_add(self.backoff);
        self.backoff = next_backoff(self.backoff);
    }

    /// Publishes the derived values that changed since the last publication.
    fn publish(&mut self, publication: &mut Publication) {
        let provisioned = self.provisioned();
        if self.published_provisioned != Some(provisioned) {
            publication.provisioned = Some(provisioned);
            self.published_provisioned = Some(provisioned);
        }

        let station_ssid = self.station_ssid();
        let published_ssid = station_ssid.map_or(PublishedSsid::Cleared, PublishedSsid::Named);
        if self.published_ssid != published_ssid {
            publication.station_ssid = station_ssid;
            publication.station_ssid_cleared = station_ssid.is_none();
            self.published_ssid = published_ssid;
        }

        let station_up = self.station_up();
        if self.published_station_up != Some(station_up) {
            publication.station_up = Some(station_up);
            self.published_station_up = Some(station_up);
        }
    }
}
