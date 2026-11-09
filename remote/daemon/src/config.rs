//! The token store and endpoint cache (ADR-0039's `config` module).
//!
//! The store is one per-user `config.toml`, discovered through [`directories`], holding the
//! per-device pairing tokens, a `default_token` for manual endpoints, and the last endpoint and
//! device id (ADR-0039). It is written atomically with mode `0600` on Unix so the pairing token is
//! protected by the PC user account, and it is the [`TokenGate`] the app adapter compares against.
//!
//! No token is ever accepted on the command line: the CLI has no token flag, and the only writers
//! are the LAN `pair` message and the daemon's own cache updates.

use std::{
    collections::BTreeMap,
    fmt,
    fs::{self, File, OpenOptions},
    io::{self, Write as _},
    net::SocketAddr,
    path::{Path, PathBuf},
    sync::{
        Mutex, MutexGuard, PoisonError,
        atomic::{AtomicU64, Ordering},
    },
};

use bridge_net::identity::DeviceId;
use directories::ProjectDirs;
use serde::{Deserialize, Serialize};

use crate::app::TokenGate;

/// The per-user application directory name (`directories`).
const APP_DIR: &str = "explorer-remote";

/// The config file name inside the application directory.
const FILE_NAME: &str = "config.toml";

/// The config file's mode on Unix (ADR-0039).
#[cfg(unix)]
const FILE_MODE: u32 = 0o600;

/// The next temporary-file serial, so concurrent writers never share a name.
static NEXT_TEMP: AtomicU64 = AtomicU64::new(0);

/// Why the config store could not be loaded or saved.
#[derive(Debug)]
pub enum ConfigError {
    /// The per-user config directory is unavailable on this platform.
    NoConfigDir,
    /// A file operation failed.
    Io(io::Error),
    /// The file is not valid TOML or does not match the schema.
    Parse(toml::de::Error),
    /// The config could not be serialised.
    Encode(toml::ser::Error),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoConfigDir => write!(formatter, "no per-user config directory is available"),
            Self::Io(error) => write!(formatter, "the config file could not be used: {error}"),
            Self::Parse(error) => write!(formatter, "the config file did not parse: {error}"),
            Self::Encode(error) => write!(formatter, "the config file could not be encoded: {error}"),
        }
    }
}

impl std::error::Error for ConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Parse(error) => Some(error),
            Self::Encode(error) => Some(error),
            Self::NoConfigDir => None,
        }
    }
}

impl From<io::Error> for ConfigError {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

/// One device's cached record: its pairing token and last-seen display name.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeviceConfig {
    /// The device's pairing token, when one has been stored.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub token: Option<String>,
    /// The device's display name from its last Beacon, when one was seen.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
}

/// The on-disk `config.toml` (ADR-0039).
///
/// The schema is additive: absent fields default, and unknown fields are ignored by the store's
/// own reads so a newer daemon can add a key without an older one refusing the file.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Config {
    /// The token used for a manual `host:port` endpoint.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub default_token: Option<String>,
    /// Per-device tokens, keyed by the lower-case hex device id.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub devices: BTreeMap<String, DeviceConfig>,
    /// The last endpoint used, as `host:port`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_endpoint: Option<String>,
    /// The last device id used, as lower-case hex.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_device: Option<String>,
}

/// The per-user token store and endpoint cache (ADR-0039).
#[derive(Debug)]
pub struct Store {
    /// The file the store owns.
    path: PathBuf,
    /// The in-memory copy, written through the mutex so a reader never sees a partial update.
    state: Mutex<Config>,
}

impl Store {
    /// Returns the platform's per-user config path (`<config>/explorer-remote/config.toml`).
    ///
    /// # Errors
    ///
    /// Returns [`ConfigError::NoConfigDir`] when the platform exposes no config directory.
    pub fn default_path() -> Result<PathBuf, ConfigError> {
        let dirs = ProjectDirs::from("", "", APP_DIR).ok_or(ConfigError::NoConfigDir)?;
        Ok(dirs.config_dir().join(FILE_NAME))
    }

    /// Opens the config at `path`, creating the directory and an empty file only on first write.
    ///
    /// A missing file is an empty store. An existing file is read and, on Unix, tightened to
    /// `0600`; a malformed file is an error so the operator's tokens are never silently dropped.
    ///
    /// # Errors
    ///
    /// Returns [`ConfigError`] when the file cannot be read or parsed, or the parent directory
    /// cannot be created.
    pub fn open(path: impl Into<PathBuf>) -> Result<Self, ConfigError> {
        let path = path.into();
        if let Some(parent) = path.parent()
            && !parent.as_os_str().is_empty()
        {
            fs::create_dir_all(parent)?;
        }
        let config = match fs::read_to_string(&path) {
            Ok(text) => toml::from_str(&text).map_err(ConfigError::Parse)?,
            Err(error) if error.kind() == io::ErrorKind::NotFound => Config::default(),
            Err(error) => return Err(ConfigError::Io(error)),
        };
        #[cfg(unix)]
        if path.is_file() {
            set_private(&path)?;
        }
        Ok(Self {
            path,
            state: Mutex::new(config),
        })
    }

    /// Returns the file this store owns.
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Returns a copy of the in-memory config.
    #[must_use]
    pub fn config(&self) -> Config {
        self.lock().clone()
    }

    /// Returns a discovered device's own token, or the manual `default_token` for `None`.
    ///
    /// `default_token` is scoped to manual `host:port` endpoints. A discovered device has no
    /// fallback: when it has no token of its own this returns `None`, so the daemon surfaces
    /// `pairing_required` instead of attempting a preamble with a token paired to something else.
    /// A device's own token always wins for its own device id.
    #[must_use]
    pub fn token_for(&self, device: Option<DeviceId>) -> Option<String> {
        let state = self.lock();
        device.map_or_else(
            || state.default_token.clone(),
            |device| state.devices.get(&hex(device)).and_then(|record| record.token.clone()),
        )
    }

    /// Stores `token` for `device`, or as the manual `default_token` for `None`.
    ///
    /// # Errors
    ///
    /// Returns [`ConfigError`] when the atomic write fails.
    pub fn set_token(&self, device: Option<DeviceId>, token: &str) -> Result<(), ConfigError> {
        let key = device.map(hex);
        self.update(|state| match &key {
            Some(key) => {
                state.devices.entry(key.clone()).or_default().token = Some(token.to_owned());
            }
            None => state.default_token = Some(token.to_owned()),
        })
    }

    /// Stores the last endpoint and, when known, the device id.
    ///
    /// A manual endpoint passes `None` and leaves the previous device id in place, because the
    /// daemon only learns a manual Bridge's device id from its preamble.
    ///
    /// # Errors
    ///
    /// Returns [`ConfigError`] when the atomic write fails.
    pub fn set_last(&self, endpoint: SocketAddr, device: Option<DeviceId>) -> Result<(), ConfigError> {
        let device = device.map(hex);
        self.update(|state| {
            state.last_endpoint = Some(endpoint.to_string());
            if let Some(device) = device.clone() {
                state.last_device = Some(device);
            }
        })
    }

    /// Returns the last endpoint the daemon used.
    #[must_use]
    pub fn last_endpoint(&self) -> Option<SocketAddr> {
        self.lock().last_endpoint.as_deref().and_then(|text| text.parse().ok())
    }

    /// Returns the last device id the daemon used.
    #[must_use]
    pub fn last_device(&self) -> Option<DeviceId> {
        self.lock()
            .last_device
            .as_deref()
            .and_then(|text| unhex(text).map(DeviceId::new))
    }

    /// Locks the in-memory state, recovering it if another thread panicked.
    fn lock(&self) -> MutexGuard<'_, Config> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Applies `change` to the in-memory config and writes the result atomically.
    fn update(&self, change: impl FnOnce(&mut Config)) -> Result<(), ConfigError> {
        let mut state = self.lock();
        change(&mut state);
        self.save(&state)
    }

    /// Writes `state` to a private temporary file and renames it over the config.
    fn save(&self, state: &Config) -> Result<(), ConfigError> {
        let text = toml::to_string_pretty(state).map_err(ConfigError::Encode)?;
        let parent = self.path.parent().unwrap_or_else(|| Path::new("."));
        fs::create_dir_all(parent)?;
        let serial = NEXT_TEMP.fetch_add(1, Ordering::Relaxed);
        let temporary = parent.join(format!(".{FILE_NAME}.{}.{serial}.tmp", std::process::id()));
        let result = Self::write_temporary(&temporary, text.as_bytes());
        match result {
            Ok(()) => match fs::rename(&temporary, &self.path) {
                Ok(()) => Ok(()),
                Err(error) => {
                    let _ = fs::remove_file(&temporary);
                    Err(ConfigError::Io(error))
                }
            },
            Err(error) => {
                let _ = fs::remove_file(&temporary);
                Err(error)
            }
        }
    }

    /// Writes the temporary file with the private mode already applied.
    fn write_temporary(path: &Path, bytes: &[u8]) -> Result<(), ConfigError> {
        let mut file = private_file(path)?;
        file.write_all(bytes)?;
        file.sync_all()?;
        drop(file);
        #[cfg(unix)]
        set_private(path)?;
        Ok(())
    }
}

impl TokenGate for Store {
    /// Returns the token the LAN app gate compares against.
    ///
    /// With a discovered device (`last_device`) only that device's own token is valid; the manual
    /// `default_token` is never reused for it, so an unpaired robot answers `None` and the app
    /// takes its pairing path. Without a discovered device the manual `default_token` applies, and
    /// a single stored device token is the last fallback. Several ambiguous device tokens answer
    /// `None`, because the app cannot name a device in its `auth` message.
    fn token(&self) -> Option<String> {
        let state = self.lock();
        let last_device = state.last_device.as_deref().and_then(unhex).map(DeviceId::new);
        if let Some(device) = last_device {
            return state.devices.get(&hex(device)).and_then(|record| record.token.clone());
        }
        if state.default_token.is_some() {
            return state.default_token.clone();
        }
        let device_tokens: Vec<String> = state
            .devices
            .values()
            .filter_map(|record| record.token.clone())
            .collect();
        drop(state);
        match device_tokens.as_slice() {
            [only] => Some(only.clone()),
            _ => None,
        }
    }

    /// Stores a new pairing code for the last-used device, or as the manual default.
    ///
    /// The trait cannot report an I/O failure; a failed write leaves the old token in force and is
    /// logged rather than panicking.
    fn pair(&self, code: &str) {
        let device = self.last_device();
        if let Err(error) = self.set_token(device, code) {
            eprintln!("remote-daemon: the pairing code could not be saved: {error}");
        }
    }
}

/// Opens `path` for a new private file, refusing to replace an existing one.
#[cfg(unix)]
fn private_file(path: &Path) -> io::Result<File> {
    use std::os::unix::fs::OpenOptionsExt;
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(FILE_MODE)
        .open(path)
}

/// Opens `path` for a new private file, refusing to replace an existing one.
#[cfg(not(unix))]
fn private_file(path: &Path) -> io::Result<File> {
    OpenOptions::new().write(true).create_new(true).open(path)
}

/// Tightens `path` to the config file's private mode.
#[cfg(unix)]
fn set_private(path: &Path) -> io::Result<()> {
    use std::os::unix::fs::PermissionsExt;
    fs::set_permissions(path, fs::Permissions::from_mode(FILE_MODE))
}

/// Returns the lower-case hex encoding of a device id.
fn hex(device: DeviceId) -> String {
    let mut text = String::with_capacity(12);
    for byte in device.bytes() {
        use fmt::Write as _;
        let _ = write!(text, "{byte:02x}");
    }
    text
}

/// Decodes twelve lower-case or upper-case hex characters into six bytes.
fn unhex(text: &str) -> Option<[u8; 6]> {
    if text.len() != 12 {
        return None;
    }
    let mut bytes = [0u8; 6];
    for (index, chunk) in text.as_bytes().as_chunks::<2>().0.iter().enumerate() {
        let pair = std::str::from_utf8(chunk).ok()?;
        bytes[index] = u8::from_str_radix(pair, 16).ok()?;
    }
    Some(bytes)
}
