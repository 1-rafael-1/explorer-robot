//! The Setup Mode HTTP portal: request parsing, form decoding, the scan cache and the one-page
//! HTML renderer (ADR-0036).
//!
//! This module is the `no_std`, allocation-free half of the portal. The firmware owns the socket,
//! the radio scan and the provisioning state machine; this code owns only the wire formats:
//!
//! - [`parse_request`] finds a complete HTTP/1.x request and its body,
//! - [`parse_provision`] decodes the `application/x-www-form-urlencoded` credentials,
//! - [`ScanCache`] keeps the strongest observation of each `SSID`,
//! - [`write_status_text`] renders the live status line, and
//! - [`write_index_page`], [`write_status_response`], [`write_redirect`] and [`write_error`]
//!   render complete HTTP/1.1 responses into caller-supplied buffers.
//!
//! Every writer reports the number of bytes written, or `None` when the buffer is too small, so
//! the firmware can retry with a larger buffer rather than emitting a truncated response.

/// The largest request the portal accepts, including the header block and the body.
pub const MAX_REQUEST_BYTES: usize = 2048;

/// The largest response the portal renders, large enough for a full one-page scan list.
pub const MAX_RESPONSE_BYTES: usize = 6144;

/// How many distinct access points the scan cache keeps.
pub const MAX_SCAN_ENTRIES: usize = 12;

/// The largest `SSID` the cache and the form accept, in bytes.
pub const MAX_SSID_BYTES: usize = 32;

/// The shortest WPA2-PSK passphrase the form accepts, in bytes.
pub const MIN_PASSPHRASE_BYTES: usize = 8;

/// The longest WPA2-PSK passphrase the form accepts, in bytes.
pub const MAX_PASSPHRASE_BYTES: usize = 63;

/// The bytes reserved at the front of a page response for the HTTP head.
///
/// The longest head this module renders is well under this size, so the body can be rendered
/// first and then shifted left once its exact length is known.
const HEADER_RESERVE: usize = 192;

/// The longest status text, `connected at 255.255.255.255`, in bytes.
const MAX_STATUS_TEXT_BYTES: usize = 32;

/// The longest form key this module recognises, `passphrase`, in bytes.
const MAX_KEY_BYTES: usize = 10;

/// The HTTP methods the portal serves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Method {
    /// `GET`, used by the page and the status poll.
    Get,
    /// `POST`, used by the provisioning and forget forms.
    Post,
}

/// A complete HTTP request, borrowed from the caller's buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Request<'a> {
    /// The request's method.
    pub method: Method,
    /// The request path, with any query string removed.
    pub path: &'a str,
    /// The offset in the original request buffer where the body begins.
    pub body_start: usize,
}

/// Why a request could not be parsed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RequestError {
    /// No complete header block yet, or the body is shorter than `Content-Length`.
    Incomplete,
    /// The request is malformed or too large.
    Malformed,
}

/// The credentials decoded from a provisioning form.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Credentials {
    /// The `SSID` bytes, zero-padded after [`Credentials::ssid_len`].
    pub ssid: [u8; MAX_SSID_BYTES],
    /// The number of valid `SSID` bytes.
    pub ssid_len: u8,
    /// The passphrase bytes, zero-padded after [`Credentials::passphrase_len`].
    pub passphrase: [u8; MAX_PASSPHRASE_BYTES],
    /// The number of valid passphrase bytes.
    pub passphrase_len: u8,
}

impl Credentials {
    /// Borrows the `SSID` bytes.
    #[must_use]
    pub fn ssid(&self) -> &[u8] {
        &self.ssid[..usize::from(self.ssid_len).min(MAX_SSID_BYTES)]
    }

    /// Borrows the passphrase bytes.
    #[must_use]
    pub fn passphrase(&self) -> &[u8] {
        &self.passphrase[..usize::from(self.passphrase_len).min(MAX_PASSPHRASE_BYTES)]
    }
}

/// Why a provisioning form could not be decoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FormError {
    /// The `ssid` field is missing or empty.
    MissingSsid,
    /// The `ssid` field is longer than [`MAX_SSID_BYTES`].
    SsidTooLong,
    /// The passphrase is missing or outside the WPA2-PSK length range.
    BadPassphrase,
    /// A field is not `key=value`, or a percent escape is invalid.
    Malformed,
}

/// One access point in the scan cache.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScanEntry {
    /// The `SSID` bytes, zero-padded after [`ScanEntry::ssid_len`].
    pub ssid: [u8; MAX_SSID_BYTES],
    /// The number of valid `SSID` bytes.
    pub ssid_len: u8,
    /// The signal strength in dBm; a higher value is stronger.
    pub rssi: i8,
    /// Whether the access point advertises a secure network.
    pub secure: bool,
}

impl ScanEntry {
    /// Borrows the `SSID` bytes.
    #[must_use]
    pub fn ssid(&self) -> &[u8] {
        &self.ssid[..usize::from(self.ssid_len).min(MAX_SSID_BYTES)]
    }
}

/// The provisioned, connecting or connected state reported on the page.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    /// The Bridge has no credential record.
    Unprovisioned,
    /// The Bridge is joining the station network.
    Connecting,
    /// The station rejected the passphrase.
    WrongPassword,
    /// The join failed for another reason.
    Failed,
    /// The station joined at the given IPv4 address.
    Connected([u8; 4]),
}

/// A blank scan entry used to initialise a [`ScanCache`].
const EMPTY_ENTRY: ScanEntry = ScanEntry {
    ssid: [0; MAX_SSID_BYTES],
    ssid_len: 0,
    rssi: 0,
    secure: false,
};

/// The best observation of each distinct `SSID`, strongest first.
#[derive(Clone, Copy, Debug)]
pub struct ScanCache {
    /// The observed access points, strongest first while `length` entries are valid.
    slots: [ScanEntry; MAX_SCAN_ENTRIES],
    /// The number of valid entries in `slots`.
    length: usize,
}

impl ScanCache {
    /// Creates an empty cache.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            slots: [EMPTY_ENTRY; MAX_SCAN_ENTRIES],
            length: 0,
        }
    }

    /// Forgets every observation.
    pub const fn clear(&mut self) {
        self.length = 0;
    }

    /// Records one observation, keeping the strongest signal per `SSID`.
    ///
    /// The comparison is case-sensitive. Empty `SSID`s and `SSID`s longer than
    /// [`MAX_SSID_BYTES`] are ignored. When the cache is full, the weakest entry is replaced only
    /// if the new observation is stronger. The entries are left sorted strongest first.
    pub fn observe(&mut self, ssid: &[u8], rssi: i8, secure: bool) {
        if ssid.is_empty() || ssid.len() > MAX_SSID_BYTES {
            return;
        }

        let mut existing = None;
        for index in 0..self.length {
            let Some(entry) = self.slots.get(index) else {
                break;
            };
            if usize::from(entry.ssid_len) == ssid.len() && entry.ssid.get(..ssid.len()) == Some(ssid) {
                existing = Some(index);
                break;
            }
        }

        if let Some(index) = existing {
            if let Some(entry) = self.slots.get_mut(index)
                && rssi > entry.rssi
            {
                entry.rssi = rssi;
                entry.secure = secure;
            }
        } else if self.length < MAX_SCAN_ENTRIES {
            let index = self.length;
            if let Some(slot) = self.slots.get_mut(index) {
                store_entry(slot, ssid, rssi, secure);
                self.length += 1;
            }
        } else {
            let Some(index) = self.weakest_index() else {
                return;
            };
            let Some(weakest_rssi) = self.slots.get(index).map(|entry| entry.rssi) else {
                return;
            };
            if rssi > weakest_rssi
                && let Some(slot) = self.slots.get_mut(index)
            {
                store_entry(slot, ssid, rssi, secure);
            }
        }

        self.sort_entries();
    }

    /// Sorts the live entries strongest first with a bounded insertion sort.
    fn sort_entries(&mut self) {
        for index in 1..self.length {
            let mut current = index;
            while current > 0 && self.slots[current - 1].rssi < self.slots[current].rssi {
                self.slots.swap(current - 1, current);
                current -= 1;
            }
        }
    }

    /// Borrows the live entries, strongest first.
    #[must_use]
    pub fn entries(&self) -> &[ScanEntry] {
        &self.slots[..self.length]
    }

    /// Returns the index of the weakest entry, or `None` when the cache is empty.
    fn weakest_index(&self) -> Option<usize> {
        let mut weakest = None;
        let mut weakest_rssi = i8::MAX;
        for index in 0..self.length {
            let Some(entry) = self.slots.get(index) else {
                break;
            };
            if entry.rssi < weakest_rssi {
                weakest_rssi = entry.rssi;
                weakest = Some(index);
            }
        }
        weakest
    }
}

impl Default for ScanCache {
    fn default() -> Self {
        Self::new()
    }
}

/// Writes one observation into an empty cache slot.
fn store_entry(slot: &mut ScanEntry, ssid: &[u8], rssi: i8, secure: bool) {
    slot.ssid.fill(0);
    if let Some(destination) = slot.ssid.get_mut(..ssid.len()) {
        destination.copy_from_slice(ssid);
    }
    slot.ssid_len = u8::try_from(ssid.len()).unwrap_or(u8::MAX);
    slot.rssi = rssi;
    slot.secure = secure;
}

/// Parses an HTTP/1.x request, returning it only when the full body is present.
///
/// The returned [`Request::path`] omits any query string and [`Request::body_start`] indexes the
/// original `request` buffer, so the body is `&request[body_start..]`.
///
/// # Errors
///
/// Returns [`RequestError::Incomplete`] when the header block or body has not arrived yet, and
/// [`RequestError::Malformed`] when the request line, headers, version, path or
/// `Content-Length` is invalid, or when the request exceeds [`MAX_REQUEST_BYTES`].
pub fn parse_request(request: &[u8]) -> Result<Request<'_>, RequestError> {
    let scan_length = request.len().min(MAX_REQUEST_BYTES);
    let Some(window) = request.get(..scan_length) else {
        return Err(RequestError::Incomplete);
    };
    let Some(header_end) = find_header_end(window) else {
        return if request.len() >= MAX_REQUEST_BYTES {
            Err(RequestError::Malformed)
        } else {
            Err(RequestError::Incomplete)
        };
    };
    let body_start = header_end.saturating_add(4);
    let Some(header) = request.get(..header_end) else {
        return Err(RequestError::Malformed);
    };

    let (request_line, headers) = split_header(header)?;
    let (method, path) = parse_request_line(request_line)?;
    check_body(request, headers, body_start)?;

    Ok(Request {
        method,
        path,
        body_start,
    })
}

/// Parses an `application/x-www-form-urlencoded` body with `ssid` and `passphrase` fields.
///
/// Unknown fields are ignored and the last duplicate of a known field wins. `+` decodes to a
/// space and `%XX` decodes to one byte.
///
/// # Errors
///
/// Returns [`FormError::MissingSsid`] for a missing or empty `ssid`, [`FormError::SsidTooLong`]
/// for one longer than [`MAX_SSID_BYTES`], [`FormError::BadPassphrase`] for a missing passphrase
/// or one outside `MIN_PASSPHRASE_BYTES..=MAX_PASSPHRASE_BYTES`, and [`FormError::Malformed`] for
/// a field without `=` or an invalid percent escape.
pub fn parse_provision(body: &[u8]) -> Result<Credentials, FormError> {
    let mut ssid = [0u8; MAX_SSID_BYTES];
    let mut ssid_len = 0usize;
    let mut passphrase = [0u8; MAX_PASSPHRASE_BYTES];
    let mut passphrase_len = 0usize;

    for field in body.split(|byte| *byte == b'&') {
        if field.is_empty() {
            continue;
        }
        apply_field(field, &mut ssid, &mut ssid_len, &mut passphrase, &mut passphrase_len)?;
    }

    if ssid_len == 0 {
        return Err(FormError::MissingSsid);
    }
    if ssid_len > MAX_SSID_BYTES {
        return Err(FormError::SsidTooLong);
    }
    if !(MIN_PASSPHRASE_BYTES..=MAX_PASSPHRASE_BYTES).contains(&passphrase_len) {
        return Err(FormError::BadPassphrase);
    }

    let ssid_len = u8::try_from(ssid_len).map_err(|_| FormError::SsidTooLong)?;
    let passphrase_len = u8::try_from(passphrase_len).map_err(|_| FormError::BadPassphrase)?;
    Ok(Credentials {
        ssid,
        ssid_len,
        passphrase,
        passphrase_len,
    })
}

/// Writes only the status text, without HTTP framing, returning its length.
///
/// The text is one of `unprovisioned`, `connecting`, `wrong password`, `failed`, or
/// `connected at A.B.C.D`.
#[must_use]
pub fn write_status_text(status: Status, out: &mut [u8]) -> Option<usize> {
    let mut text = [0u8; MAX_STATUS_TEXT_BYTES];
    let length = {
        let mut writer = Writer::new(&mut text);
        let written = match status {
            Status::Unprovisioned => writer.text("unprovisioned"),
            Status::Connecting => writer.text("connecting"),
            Status::WrongPassword => writer.text("wrong password"),
            Status::Failed => writer.text("failed"),
            Status::Connected(address) => write_connected(&mut writer, address),
        };
        if !written {
            return None;
        }
        writer.written()
    };
    if out.len() < length {
        return None;
    }
    let destination = out.get_mut(..length)?;
    destination.copy_from_slice(text.get(..length)?);
    Some(length)
}

/// Writes a complete HTTP/1.1 `text/plain` response carrying the status text.
#[must_use]
pub fn write_status_response(status: Status, out: &mut [u8]) -> Option<usize> {
    let mut text = [0u8; MAX_STATUS_TEXT_BYTES];
    let text_len = write_status_text(status, &mut text)?;
    let body = text.get(..text_len)?;

    let mut writer = Writer::new(out);
    if !write_head(&mut writer, "HTTP/1.1 200 OK", "text/plain; charset=utf-8", body.len()) || !writer.bytes(body) {
        return None;
    }
    Some(writer.written())
}

/// Writes a complete `303 See Other` response that sends the browser back to `/`.
#[must_use]
pub fn write_redirect(out: &mut [u8]) -> Option<usize> {
    let mut writer = Writer::new(out);
    let written = writer.text(
        "HTTP/1.1 303 See Other\r\n\
         Location: /\r\n\
         Content-Type: text/plain; charset=utf-8\r\n\
         Content-Length: 0\r\n\
         Cache-Control: no-store\r\n\
         Connection: close\r\n\
         \r\n",
    );
    if written { Some(writer.written()) } else { None }
}

/// Writes a complete `text/plain` HTTP error response for `400`, `404` or `405`.
///
/// Returns `None` for an unsupported code or when `out` is too small.
#[must_use]
pub fn write_error(code: u16, out: &mut [u8]) -> Option<usize> {
    let reason = match code {
        400 => "Bad Request",
        404 => "Not Found",
        405 => "Method Not Allowed",
        _ => return None,
    };

    let mut writer = Writer::new(out);
    if !write_status_line(&mut writer, code, reason)
        || !write_header_tail(&mut writer, "text/plain; charset=utf-8", reason.len())
        || !writer.text(reason)
    {
        return None;
    }
    Some(writer.written())
}

/// Writes a complete HTTP/1.1 response for the portal page.
///
/// The page is HTML5 with inline CSS and a small inline script. It embeds the initial status text
/// and the scan list, HTML-escaping every `SSID`. It fits [`MAX_RESPONSE_BYTES`] even when the
/// cache holds [`MAX_SCAN_ENTRIES`] maximum-length `SSID`s. Returns `None` when `out` is too small.
#[must_use]
pub fn write_index_page(cache: &ScanCache, status: Status, out: &mut [u8]) -> Option<usize> {
    let body_buffer = out.get_mut(HEADER_RESERVE..)?;
    let body_length = {
        let mut writer = Writer::new(body_buffer);
        if !write_index_body(cache, status, &mut writer) {
            return None;
        }
        writer.written()
    };

    let mut head = [0u8; HEADER_RESERVE];
    let head_length = {
        let mut writer = Writer::new(&mut head);
        if !write_head(&mut writer, "HTTP/1.1 200 OK", "text/html; charset=utf-8", body_length) {
            return None;
        }
        writer.written()
    };
    if head_length > HEADER_RESERVE {
        return None;
    }

    out.copy_within(HEADER_RESERVE..HEADER_RESERVE.saturating_add(body_length), head_length);
    let destination = out.get_mut(..head_length)?;
    destination.copy_from_slice(head.get(..head_length)?);
    Some(head_length.saturating_add(body_length))
}

/// A bounds-checked cursor that appends bytes to a caller-supplied buffer.
struct Writer<'a> {
    /// The destination buffer.
    buffer: &'a mut [u8],
    /// The number of bytes written so far.
    written: usize,
}

impl<'a> Writer<'a> {
    /// Creates an empty writer over `buffer`.
    const fn new(buffer: &'a mut [u8]) -> Self {
        Self { buffer, written: 0 }
    }

    /// Appends every byte of `bytes`, returning whether they all fit.
    fn bytes(&mut self, bytes: &[u8]) -> bool {
        let Some(end) = self.written.checked_add(bytes.len()) else {
            return false;
        };
        let Some(destination) = self.buffer.get_mut(self.written..end) else {
            return false;
        };
        destination.copy_from_slice(bytes);
        self.written = end;
        true
    }

    /// Appends one byte, returning whether it fits.
    fn byte(&mut self, byte: u8) -> bool {
        let Some(slot) = self.buffer.get_mut(self.written) else {
            return false;
        };
        *slot = byte;
        self.written += 1;
        true
    }

    /// Appends the bytes of an ASCII string, returning whether they all fit.
    fn text(&mut self, text: &str) -> bool {
        self.bytes(text.as_bytes())
    }

    /// Returns the number of bytes written so far.
    const fn written(&self) -> usize {
        self.written
    }
}

/// Appends `bytes` with the five HTML metacharacters escaped.
fn write_escaped(writer: &mut Writer<'_>, bytes: &[u8]) -> bool {
    for &byte in bytes {
        let written = match byte {
            b'&' => writer.text("&amp;"),
            b'<' => writer.text("&lt;"),
            b'>' => writer.text("&gt;"),
            b'"' => writer.text("&quot;"),
            b'\'' => writer.text("&#39;"),
            _ => writer.byte(byte),
        };
        if !written {
            return false;
        }
    }
    true
}

/// Appends `value` in decimal without allocating.
fn write_decimal(writer: &mut Writer<'_>, value: usize) -> bool {
    let mut digits = [0u8; 20];
    let mut length = 0usize;
    let mut remaining = value;
    loop {
        let Some(slot) = digits.get_mut(length) else {
            return false;
        };
        let digit = u8::try_from(remaining % 10).unwrap_or(0);
        *slot = b'0' + digit;
        length += 1;
        remaining /= 10;
        if remaining == 0 {
            break;
        }
    }
    for &digit in digits.get(..length).unwrap_or(&[]).iter().rev() {
        if !writer.byte(digit) {
            return false;
        }
    }
    true
}

/// Writes a `u8` in decimal into a three-byte scratch buffer, returning the length.
const fn write_u8_decimal(value: u8, out: &mut [u8; 3]) -> usize {
    let hundreds = value / 100;
    let tens = (value / 10) % 10;
    let ones = value % 10;
    if value >= 100 {
        out[0] = b'0' + hundreds;
        out[1] = b'0' + tens;
        out[2] = b'0' + ones;
        3
    } else if value >= 10 {
        out[0] = b'0' + tens;
        out[1] = b'0' + ones;
        2
    } else {
        out[0] = b'0' + ones;
        1
    }
}

/// Appends `connected at A.B.C.D` for an IPv4 address.
fn write_connected(writer: &mut Writer<'_>, address: [u8; 4]) -> bool {
    if !writer.text("connected at ") {
        return false;
    }
    for (index, octet) in address.iter().enumerate() {
        if index > 0 && !writer.byte(b'.') {
            return false;
        }
        let mut digits = [0u8; 3];
        let length = write_u8_decimal(*octet, &mut digits);
        if !writer.bytes(digits.get(..length).unwrap_or(&[])) {
            return false;
        }
    }
    true
}

/// Writes a status line followed by the common response headers and the blank line.
fn write_head(writer: &mut Writer<'_>, status_line: &str, content_type: &str, body_len: usize) -> bool {
    writer.text(status_line) && write_header_tail(writer, content_type, body_len)
}

/// Writes the common response headers and the blank line that ends the head.
fn write_header_tail(writer: &mut Writer<'_>, content_type: &str, body_len: usize) -> bool {
    writer.text("\r\nContent-Type: ")
        && writer.text(content_type)
        && writer.text("\r\nContent-Length: ")
        && write_decimal(writer, body_len)
        && writer.text("\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n")
}

/// Writes `HTTP/1.1 CODE REASON` without its trailing line ending.
fn write_status_line(writer: &mut Writer<'_>, code: u16, reason: &str) -> bool {
    writer.text("HTTP/1.1 ") && write_decimal(writer, usize::from(code)) && writer.byte(b' ') && writer.text(reason)
}

/// Writes the status element with its escaped initial text.
fn write_status_element(writer: &mut Writer<'_>, status: Status) -> bool {
    let mut text = [0u8; MAX_STATUS_TEXT_BYTES];
    let Some(length) = write_status_text(status, &mut text) else {
        return false;
    };
    let Some(bytes) = text.get(..length) else {
        return false;
    };
    writer.text("<p id=\"status\">") && write_escaped(writer, bytes) && writer.text("</p>\n")
}

/// Writes one scan-list option with its escaped `SSID` and signal details.
fn write_option(writer: &mut Writer<'_>, entry: &ScanEntry) -> bool {
    writer.text("<option value=\"")
        && write_escaped(writer, entry.ssid())
        && writer.text("\">")
        && write_escaped(writer, entry.ssid())
        && writer.text(" (")
        && (entry.rssi >= 0 || writer.byte(b'-'))
        && write_decimal(writer, usize::from(entry.rssi.unsigned_abs()))
        && writer.text(" dBm, ")
        && writer.text(if entry.secure { "WPA2" } else { "open" })
        && writer.text(")</option>\n")
}

/// Writes the full portal document body.
fn write_index_body(cache: &ScanCache, status: Status, writer: &mut Writer<'_>) -> bool {
    if !writer.text(INDEX_HEAD)
        || !write_status_element(writer, status)
        || !writer.text(PROVISION_FORM_HEAD)
        || !writer.text("<datalist id=\"scan-list\">\n")
    {
        return false;
    }

    for entry in cache.entries() {
        if !write_option(writer, entry) {
            return false;
        }
    }

    writer.text("</datalist>\n")
        && writer.text(PROVISION_FORM_TAIL)
        && writer.text(FORGET_FORM)
        && writer.text(SCRIPT)
        && writer.text(INDEX_TAIL)
}

/// The page opening, up to and including the heading.
const INDEX_HEAD: &str = r#"<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Explorer Robot Setup</title>
<style>body{font-family:system-ui,sans-serif;margin:2rem auto;max-width:32rem;padding:0 1rem}label{display:block;margin:1rem 0 .25rem}input,button{font:inherit;padding:.5rem;width:100%;box-sizing:border-box}#status{font-weight:600}form{margin:1rem 0}</style>
</head>
<body>
<h1>Explorer Robot</h1>
"#;

/// The provisioning form opening, its `SSID` input and the start of the scan list.
const PROVISION_FORM_HEAD: &str = r#"<form method="post" action="/provision">
<label for="ssid">Wi-Fi network</label>
<input name="ssid" list="scan-list">
"#;

/// The provisioning form's passphrase input, submit button and closing tag.
const PROVISION_FORM_TAIL: &str = r#"<label for="passphrase">Password</label>
<input type="password" name="passphrase">
<button type="submit">Connect</button>
</form>
"#;

/// The separate forget-network form.
const FORGET_FORM: &str = r#"<form method="post" action="/forget">
<button type="submit">Forget network</button>
</form>
"#;

/// The inline script that polls `/status` once a second.
const SCRIPT: &str = r"<script>
async function poll(){try{const response=await fetch('/status', {cache:'no-store'});document.getElementById('status').textContent=await response.text();}catch(error){}}
setInterval(poll,1000);poll();
</script>
";

/// The page closing.
const INDEX_TAIL: &str = "</body>\n</html>\n";

/// Returns the index of the first `\r\n\r\n`.
fn find_header_end(bytes: &[u8]) -> Option<usize> {
    bytes.windows(4).position(|window| window == b"\r\n\r\n".as_slice())
}

/// Returns the index of the first `\r\n`.
fn find_crlf(bytes: &[u8]) -> Option<usize> {
    bytes.windows(2).position(|window| window == b"\r\n".as_slice())
}

/// Splits a header block into its request line and the remaining header lines.
fn split_header(header: &[u8]) -> Result<(&[u8], &[u8]), RequestError> {
    let Some(line_end) = find_crlf(header) else {
        return Ok((header, header.get(header.len()..).unwrap_or(&[])));
    };
    let Some(line) = header.get(..line_end) else {
        return Err(RequestError::Malformed);
    };
    let Some(rest) = header.get(line_end.saturating_add(2)..) else {
        return Err(RequestError::Malformed);
    };
    Ok((line, rest))
}

/// Checks that `request` holds the complete body declared by `headers` for `method`.
fn check_body(request: &[u8], headers: &[u8], body_start: usize) -> Result<(), RequestError> {
    // A bodyless POST (the portal's Forget form) is legal: an absent Content-Length means zero
    // bytes, exactly as it does for GET.
    let content_length = parse_content_length(headers)?.unwrap_or(0);
    if content_length > MAX_REQUEST_BYTES.saturating_sub(body_start) {
        return Err(RequestError::Malformed);
    }
    let Some(body_end) = body_start.checked_add(content_length) else {
        return Err(RequestError::Malformed);
    };
    if body_end > request.len() {
        return Err(RequestError::Incomplete);
    }
    Ok(())
}

/// Parses `METHOD SP PATH SP HTTP/1.x`, returning the method and the path without a query.
fn parse_request_line(line: &[u8]) -> Result<(Method, &str), RequestError> {
    let mut parts = line.split(|byte| *byte == b' ');
    let Some(method) = parts.next() else {
        return Err(RequestError::Malformed);
    };
    let Some(target) = parts.next() else {
        return Err(RequestError::Malformed);
    };
    let Some(version) = parts.next() else {
        return Err(RequestError::Malformed);
    };
    if parts.next().is_some() {
        return Err(RequestError::Malformed);
    }

    let method = match method {
        b"GET" => Method::Get,
        b"POST" => Method::Post,
        _ => return Err(RequestError::Malformed),
    };
    if version != b"HTTP/1.0" && version != b"HTTP/1.1" {
        return Err(RequestError::Malformed);
    }
    if target.is_empty() || target.iter().any(|byte| *byte < 0x20 || *byte >= 0x7F) {
        return Err(RequestError::Malformed);
    }
    let path_bytes = target
        .iter()
        .position(|byte| *byte == b'?')
        .map_or(target, |index| target.get(..index).unwrap_or(&[]));
    let path = core::str::from_utf8(path_bytes).map_err(|_| RequestError::Malformed)?;
    Ok((method, path))
}

/// Parses every header, returning the single `Content-Length` value if present.
fn parse_content_length(headers: &[u8]) -> Result<Option<usize>, RequestError> {
    let mut content_length = None;
    for line in headers.split(|byte| *byte == b'\n') {
        let line = line.strip_suffix(b"\r").unwrap_or(line);
        if line.is_empty() {
            continue;
        }
        let Some(colon) = line.iter().position(|byte| *byte == b':') else {
            return Err(RequestError::Malformed);
        };
        let Some(name) = line.get(..colon) else {
            return Err(RequestError::Malformed);
        };
        let Some(value) = line.get(colon.saturating_add(1)..) else {
            return Err(RequestError::Malformed);
        };
        if name.eq_ignore_ascii_case(b"content-length") {
            content_length = Some(parse_decimal(value.trim_ascii())?);
        }
    }
    Ok(content_length)
}

/// Parses a non-empty decimal into a saturating `usize`.
fn parse_decimal(bytes: &[u8]) -> Result<usize, RequestError> {
    if bytes.is_empty() {
        return Err(RequestError::Malformed);
    }
    let mut value = 0usize;
    for &byte in bytes {
        if !byte.is_ascii_digit() {
            return Err(RequestError::Malformed);
        }
        value = value
            .saturating_mul(10)
            .saturating_add(usize::from(byte.saturating_sub(b'0')));
    }
    Ok(value)
}

/// Decodes `+` and `%XX` escapes, returning `None` when the result does not fit `out`.
fn decode_component(input: &[u8], out: &mut [u8]) -> Result<Option<usize>, FormError> {
    let mut length = 0usize;
    let mut overflow = false;
    let mut index = 0usize;
    while let Some(&byte) = input.get(index) {
        let decoded = if byte == b'+' {
            index += 1;
            b' '
        } else if byte == b'%' {
            let high = input.get(index.saturating_add(1)).copied().and_then(hex_value);
            let low = input.get(index.saturating_add(2)).copied().and_then(hex_value);
            let (Some(high), Some(low)) = (high, low) else {
                return Err(FormError::Malformed);
            };
            index += 3;
            (high << 4) | low
        } else {
            index += 1;
            byte
        };

        if let Some(slot) = out.get_mut(length) {
            *slot = decoded;
            length += 1;
        } else {
            overflow = true;
        }
    }
    Ok(if overflow { None } else { Some(length) })
}

/// Decodes one hexadecimal digit.
const fn hex_value(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte.saturating_sub(b'0')),
        b'a'..=b'f' => Some(byte.saturating_sub(b'a') + 10),
        b'A'..=b'F' => Some(byte.saturating_sub(b'A') + 10),
        _ => None,
    }
}

/// Applies one `key=value` field to the credential buffers.
fn apply_field(
    field: &[u8],
    ssid: &mut [u8; MAX_SSID_BYTES],
    ssid_len: &mut usize,
    passphrase: &mut [u8; MAX_PASSPHRASE_BYTES],
    passphrase_len: &mut usize,
) -> Result<(), FormError> {
    let Some(separator) = field.iter().position(|byte| *byte == b'=') else {
        return Err(FormError::Malformed);
    };
    let Some(key_bytes) = field.get(..separator) else {
        return Err(FormError::Malformed);
    };
    let Some(value_bytes) = field.get(separator.saturating_add(1)..) else {
        return Err(FormError::Malformed);
    };

    let mut key = [0u8; MAX_KEY_BYTES];
    let key_len = match decode_component(key_bytes, &mut key) {
        Ok(Some(length)) => length,
        Ok(None) => {
            decode_component(value_bytes, &mut [])?;
            return Ok(());
        }
        Err(error) => return Err(error),
    };

    match key.get(..key_len).unwrap_or(&[]) {
        b"ssid" => match decode_component(value_bytes, ssid) {
            Ok(Some(length)) => *ssid_len = length,
            Ok(None) => *ssid_len = MAX_SSID_BYTES.saturating_add(1),
            Err(error) => return Err(error),
        },
        b"passphrase" => match decode_component(value_bytes, passphrase) {
            Ok(Some(length)) => *passphrase_len = length,
            Ok(None) => *passphrase_len = MAX_PASSPHRASE_BYTES.saturating_add(1),
            Err(error) => return Err(error),
        },
        _ => {
            decode_component(value_bytes, &mut [])?;
        }
    }
    Ok(())
}
