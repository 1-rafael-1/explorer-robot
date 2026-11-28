//! The blocking station join and diagnostic scan the binaries execute for the Radio Session.
//!
//! The session decides *when* and *for which SSID* to join; this module owns the `esp-radio` calls
//! and classifies the result so the session can distinguish a rejected passphrase from any other
//! failure. It is HAL-bound, so it lives in the firmware crate rather than `bridge-core`.

use bridge_core::radio::JoinResult;
use defmt::{info, warn};
use embassy_time::{Duration, Timer, with_timeout};
use esp_radio::wifi::{ConnectionError, DisconnectReason, Ssid, WifiController, WifiError, scan::ScanConfig};

/// How long an association may take before the caller gives up and reports a failure.
pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);

/// Scans for the target SSID and logs every BSS's auth mode before associating.
///
/// `esp-radio` 1.0.0-beta.1 only exposes `Wpa2Personal`; a WPA3-only or PMF-required AP is the
/// usual cause of an association failure, and the advertised auth mode settles that before any
/// credentials are tried. `scan_async` on this beta can miss a sweep, so an empty result is retried.
pub async fn scan_station_target(controller: &mut WifiController<'_>, target: &[u8]) {
    let Ok(ssid) = Ssid::try_from(target) else {
        warn!("sta: SSID too long to scan for; skipping the scan");
        return;
    };
    let config = ScanConfig::default().with_ssid(ssid);

    for attempt in 1..=3 {
        match controller.scan_async(&config).await {
            Ok(aps) if !aps.is_empty() => {
                for ap in &aps {
                    info!(
                        "sta: scan: bssid={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ch={} rssi={}dBm auth={}",
                        ap.bssid[0],
                        ap.bssid[1],
                        ap.bssid[2],
                        ap.bssid[3],
                        ap.bssid[4],
                        ap.bssid[5],
                        ap.channel,
                        ap.signal_strength,
                        ap.auth_method
                    );
                }
                return;
            }
            Ok(_) => info!("sta: scan: sweep {} found nothing; retrying", attempt),
            Err(error) => warn!("sta: scan failed: {:?}", error),
        }
        if attempt < 3 {
            Timer::after(Duration::from_secs(1)).await;
        }
    }
    warn!("sta: scan: target SSID was not found in any sweep");
}

/// Associates the station, classifying a WPA2 credential rejection so the caller can say
/// "wrong password" instead of a generic failure.
pub async fn connect_station(controller: &mut WifiController<'_>) -> JoinResult {
    match with_timeout(CONNECT_TIMEOUT, controller.connect_async()).await {
        Ok(Ok(info)) => {
            info!("sta: associated ssid={} channel={}", info.ssid, info.channel);
            JoinResult::Connected
        }
        Ok(Err(ConnectionError::Failed(info))) => {
            if auth_failure(info.reason) {
                warn!("sta: association rejected: {:?}", info.reason);
                JoinResult::WrongPassword
            } else {
                warn!("sta: association failed: {:?}", info.reason);
                JoinResult::Failed
            }
        }
        Ok(Err(ConnectionError::WifiError(WifiError::InvalidPassword))) => {
            warn!("sta: the radio rejected the passphrase");
            JoinResult::WrongPassword
        }
        Ok(Err(error)) => {
            warn!("sta: association failed: {:?}", error);
            JoinResult::Failed
        }
        Err(_) => {
            warn!("sta: association timed out");
            JoinResult::Failed
        }
    }
}

/// Returns whether a disconnect reason means the WPA2 pre-shared key was wrong.
fn auth_failure(reason: DisconnectReason) -> bool {
    matches!(
        reason,
        DisconnectReason::AuthenticationExpired
            | DisconnectReason::MicFailure
            | DisconnectReason::FourWayHandshakeTimeout
            | DisconnectReason::GroupKeyUpdateTimeout
            | DisconnectReason::IeIn4wayDiffers
            | DisconnectReason::_802_1xAuthenticationFailed
            | DisconnectReason::CipherSuiteRejected
            | DisconnectReason::BadCipherOrAkm
    )
}
