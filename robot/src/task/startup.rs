//! Core0 startup task – fires the `Initialize` event once at boot.

use crate::system::event::{Events, raise_event};

/// Fires the `Initialize` event and then exits.
///
/// Spawned on core0 at boot. Runs as a task rather than a direct `.await` at
/// the tail of `main`, decoupling event dispatch from the entry point so that
/// the multicore `#[cortex_m_rt::entry]` pattern can be used.
#[embassy_executor::task]
pub async fn startup() {
    raise_event(Events::Initialize).await;
}
