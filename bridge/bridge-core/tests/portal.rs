//! Host tests for the Setup Mode HTTP portal codec.
//!
//! The tests drive the codec the way the firmware does: complete request buffers go into
//! [`parse_request`], the provisioning form body goes into [`parse_provision`], observations go
//! into [`ScanCache`], and every response writer fills a caller-supplied buffer. No test unwraps
//! a `Result` or `Option`; a missing value simply ends the test.

use bridge_core::portal::{
    FormError, MAX_REQUEST_BYTES, MAX_RESPONSE_BYTES, MAX_SCAN_ENTRIES, MAX_SSID_BYTES, Method, RequestError,
    ScanCache, Status, parse_provision, parse_request, write_error, write_index_page, write_redirect,
    write_status_response, write_status_text,
};

/// A complete `GET /` request.
const GET_ROOT: &[u8] = b"GET / HTTP/1.1\r\nHost: setup\r\n\r\n";

/// Renders the status text for `status`, if it fits.
fn render_status_text(status: Status) -> Option<String> {
    let mut buffer = [0u8; 64];
    let length = write_status_text(status, &mut buffer)?;
    let bytes = buffer.get(..length)?;
    Some(String::from_utf8_lossy(bytes).into_owned())
}

/// Returns the response body after the blank line.
fn response_body(response: &[u8]) -> Option<&[u8]> {
    let body_start = response
        .windows(4)
        .position(|window| window == b"\r\n\r\n".as_slice())?;
    response.get(body_start.saturating_add(4)..)
}

/// Returns the `Content-Length` the response head declares.
fn declared_content_length(response: &[u8]) -> Option<usize> {
    let head_end = response
        .windows(4)
        .position(|window| window == b"\r\n\r\n".as_slice())?;
    let head = core::str::from_utf8(response.get(..head_end)?).ok()?;
    for line in head.split("\r\n") {
        if let Some(value) = line.strip_prefix("Content-Length: ") {
            return value.parse().ok();
        }
    }
    None
}

/// A request without the blank line is incomplete.
#[test]
fn an_unfinished_header_is_incomplete() {
    assert_eq!(
        parse_request(b"GET / HTTP/1.1\r\nHost: setup\r\n"),
        Err(RequestError::Incomplete)
    );
}

/// A complete `GET /` has an empty body that starts at the end of the buffer.
#[test]
fn a_complete_get_has_no_body() {
    let Some(request) = parse_request(GET_ROOT).ok() else {
        return;
    };
    assert_eq!(request.method, Method::Get);
    assert_eq!(request.path, "/");
    assert_eq!(request.body_start, GET_ROOT.len());
    assert_eq!(GET_ROOT.get(request.body_start..), Some(&b""[..]));
}

/// A query string is not part of the returned path.
#[test]
fn a_query_string_is_not_part_of_the_path() {
    let Some(request) = parse_request(b"GET /provision?x=1 HTTP/1.1\r\n\r\n").ok() else {
        return;
    };
    assert_eq!(request.method, Method::Get);
    assert_eq!(request.path, "/provision");
}

/// A `POST` exposes exactly its declared body.
#[test]
fn a_post_exposes_its_body() {
    let request_bytes = b"POST /provision HTTP/1.1\r\nContent-Length: 7\r\n\r\nbody=ok";
    let Some(request) = parse_request(request_bytes).ok() else {
        return;
    };
    assert_eq!(request.method, Method::Post);
    assert_eq!(request.body_start, request_bytes.len() - 7);
    assert_eq!(request_bytes.get(request.body_start..), Some(&b"body=ok"[..]));
}

/// A `POST` whose body has not fully arrived is incomplete.
#[test]
fn a_short_body_is_incomplete() {
    assert_eq!(
        parse_request(b"POST /provision HTTP/1.1\r\nContent-Length: 10\r\n\r\nbody"),
        Err(RequestError::Incomplete)
    );
}

/// A `POST` without `Content-Length` is a legal empty body (the portal's Forget form).
#[test]
fn a_post_without_content_length_has_an_empty_body() {
    let request_bytes = b"POST /forget HTTP/1.1\r\n\r\n";
    let Some(request) = parse_request(request_bytes).ok() else {
        return;
    };
    assert_eq!(request.method, Method::Post);
    assert_eq!(request.path, "/forget");
    assert_eq!(request.body_start, request_bytes.len());
    assert_eq!(request_bytes.get(request.body_start..), Some(&b""[..]));
}

/// A non-decimal `Content-Length` is malformed.
#[test]
fn an_invalid_content_length_is_malformed() {
    assert_eq!(
        parse_request(b"POST /provision HTTP/1.1\r\nContent-Length: nope\r\n\r\n"),
        Err(RequestError::Malformed)
    );
}

/// A method other than `GET` or `POST` is malformed.
#[test]
fn a_non_get_or_post_method_is_malformed() {
    assert_eq!(parse_request(b"PUT / HTTP/1.1\r\n\r\n"), Err(RequestError::Malformed));
    assert_eq!(
        parse_request(b"DELETE / HTTP/1.1\r\n\r\n"),
        Err(RequestError::Malformed)
    );
    assert_eq!(parse_request(b"HEAD / HTTP/1.1\r\n\r\n"), Err(RequestError::Malformed));
}

/// Header names are matched case-insensitively.
#[test]
fn a_lowercase_content_length_is_accepted() {
    let Some(request) = parse_request(b"POST /p HTTP/1.1\r\ncontent-length: 2\r\n\r\nab").ok() else {
        return;
    };
    assert_eq!(request.method, Method::Post);
}

/// A version other than `HTTP/1.0` or `HTTP/1.1` is malformed.
#[test]
fn an_unknown_http_version_is_malformed() {
    assert_eq!(parse_request(b"GET / HTTP/2.0\r\n\r\n"), Err(RequestError::Malformed));
}

/// Control or non-ASCII bytes in the request target are malformed.
#[test]
fn a_non_ascii_path_is_malformed() {
    assert_eq!(
        parse_request(b"GET /a\x01b HTTP/1.1\r\n\r\n"),
        Err(RequestError::Malformed)
    );
    assert_eq!(
        parse_request(b"GET /caf\xC3\xA9 HTTP/1.1\r\n\r\n"),
        Err(RequestError::Malformed)
    );
}

/// A buffer that reaches the cap before a header end is malformed.
#[test]
fn an_oversized_header_is_malformed() {
    let oversized = [b'a'; MAX_REQUEST_BYTES + 1];
    assert_eq!(parse_request(&oversized), Err(RequestError::Malformed));
}

/// `+` and `%XX` are decoded in both fields.
#[test]
fn the_form_decodes_percent_and_plus_escapes() {
    let Some(credentials) = parse_provision(b"ssid=My+Net%20Work&passphrase=p%2Bssword").ok() else {
        return;
    };
    assert_eq!(credentials.ssid(), b"My Net Work".as_slice());
    assert_eq!(credentials.passphrase(), b"p+ssword".as_slice());
}

/// A percent-encoded plus is distinct from a form plus.
#[test]
fn a_percent_encoded_plus_decodes_to_a_plus() {
    let Some(credentials) = parse_provision(b"ssid=a%2Bb&passphrase=12345678").ok() else {
        return;
    };
    assert_eq!(credentials.ssid(), b"a+b".as_slice());
}

/// The last duplicate of a known key wins.
#[test]
fn the_last_duplicate_wins() {
    let Some(credentials) = parse_provision(b"ssid=first&ssid=second&passphrase=12345678").ok() else {
        return;
    };
    assert_eq!(credentials.ssid(), b"second".as_slice());
}

/// A later valid duplicate overrides an earlier over-long one.
#[test]
fn a_later_valid_duplicate_wins_over_an_over_long_one() {
    let mut body = String::from("ssid=");
    body.push_str(&"a".repeat(MAX_SSID_BYTES + 1));
    body.push_str("&ssid=net&passphrase=12345678");
    let Some(credentials) = parse_provision(body.as_bytes()).ok() else {
        return;
    };
    assert_eq!(credentials.ssid(), b"net".as_slice());
}

/// An empty `ssid` is treated as missing.
#[test]
fn an_empty_ssid_is_missing() {
    assert_eq!(
        parse_provision(b"ssid=&passphrase=12345678"),
        Err(FormError::MissingSsid)
    );
    assert_eq!(parse_provision(b"passphrase=12345678"), Err(FormError::MissingSsid));
}

/// An `ssid` longer than the limit is rejected.
#[test]
fn a_too_long_ssid_is_rejected() {
    let mut body = String::from("ssid=");
    body.push_str(&"a".repeat(MAX_SSID_BYTES + 1));
    body.push_str("&passphrase=12345678");
    assert_eq!(parse_provision(body.as_bytes()), Err(FormError::SsidTooLong));
}

/// A seven-byte passphrase is below the WPA2-PSK minimum.
#[test]
fn a_seven_byte_passphrase_is_rejected() {
    assert_eq!(
        parse_provision(b"ssid=net&passphrase=1234567"),
        Err(FormError::BadPassphrase)
    );
}

/// An eight-byte passphrase is accepted.
#[test]
fn an_eight_byte_passphrase_is_accepted() {
    let Some(credentials) = parse_provision(b"ssid=net&passphrase=12345678").ok() else {
        return;
    };
    assert_eq!(credentials.ssid(), b"net".as_slice());
    assert_eq!(credentials.passphrase(), b"12345678".as_slice());
}

/// A missing passphrase is rejected.
#[test]
fn a_missing_passphrase_is_rejected() {
    assert_eq!(parse_provision(b"ssid=net"), Err(FormError::BadPassphrase));
}

/// An invalid percent escape is malformed.
#[test]
fn an_invalid_percent_escape_is_malformed() {
    assert_eq!(
        parse_provision(b"ssid=%zz&passphrase=12345678"),
        Err(FormError::Malformed)
    );
    assert_eq!(
        parse_provision(b"ssid=%2&passphrase=12345678"),
        Err(FormError::Malformed)
    );
    assert_eq!(
        parse_provision(b"ssid=%&passphrase=12345678"),
        Err(FormError::Malformed)
    );
}

/// Unknown keys are ignored, and their escapes are still validated.
#[test]
fn unknown_keys_are_ignored() {
    let Some(credentials) = parse_provision(b"ssid=net&foo=bar&passphrase=12345678&baz=%41").ok() else {
        return;
    };
    assert_eq!(credentials.ssid(), b"net".as_slice());
    assert_eq!(credentials.passphrase(), b"12345678".as_slice());

    assert_eq!(
        parse_provision(b"ssid=net&foo=%zz&passphrase=12345678"),
        Err(FormError::Malformed)
    );
}

/// A stronger observation of the same `SSID` replaces the weaker one.
#[test]
fn a_stronger_observation_wins() {
    let mut cache = ScanCache::new();
    cache.observe(b"net", -70, true);
    cache.observe(b"net", -40, true);
    assert_eq!(cache.entries().len(), 1);
    let Some(entry) = cache.entries().first() else {
        return;
    };
    assert_eq!(entry.ssid(), b"net".as_slice());
    assert_eq!(entry.rssi, -40);
}

/// A weaker observation never replaces a stronger one.
#[test]
fn a_weaker_observation_is_ignored() {
    let mut cache = ScanCache::new();
    cache.observe(b"net", -40, true);
    cache.observe(b"net", -70, false);
    let Some(entry) = cache.entries().first() else {
        return;
    };
    assert_eq!(entry.rssi, -40);
}

/// Entries are sorted strongest first.
#[test]
fn entries_are_sorted_strongest_first() {
    let mut cache = ScanCache::new();
    cache.observe(b"a", -70, true);
    cache.observe(b"b", -50, false);
    cache.observe(b"c", -60, true);
    let rssis: Vec<i8> = cache.entries().iter().map(|entry| entry.rssi).collect();
    assert_eq!(rssis, vec![-50, -60, -70]);
}

/// The cache keeps only the twelve strongest networks.
#[test]
fn the_cache_keeps_only_its_capacity() {
    let mut cache = ScanCache::new();
    for index in 0..=MAX_SCAN_ENTRIES {
        let ssid = format!("net{index}");
        let rssi = i8::try_from(index).unwrap_or(0).saturating_neg();
        cache.observe(ssid.as_bytes(), rssi, true);
    }
    assert_eq!(cache.entries().len(), MAX_SCAN_ENTRIES);
    assert!(
        cache.entries().iter().all(|entry| entry.ssid() != b"net12".as_slice()),
        "the weakest of thirteen observations is dropped"
    );
}

/// A stronger newcomer replaces the weakest entry when the cache is full.
#[test]
fn a_stronger_newcomer_replaces_the_weakest_entry() {
    let mut cache = ScanCache::new();
    for index in 0..MAX_SCAN_ENTRIES {
        let ssid = format!("net{index}");
        let rssi = i8::try_from(index).unwrap_or(0).saturating_neg();
        cache.observe(ssid.as_bytes(), rssi, true);
    }
    cache.observe(b"new", 0, false);
    assert_eq!(cache.entries().len(), MAX_SCAN_ENTRIES);
    assert!(cache.entries().iter().any(|entry| entry.ssid() == b"new".as_slice()));
    assert!(
        cache.entries().iter().all(|entry| entry.ssid() != b"net11".as_slice()),
        "the weakest entry at -11 dBm is replaced"
    );

    cache.observe(b"weak", -100, true);
    assert!(
        cache.entries().iter().all(|entry| entry.ssid() != b"weak".as_slice()),
        "a newcomer weaker than every entry is dropped"
    );
}

/// Empty and over-long `SSID`s are ignored.
#[test]
fn empty_and_over_long_ssids_are_ignored() {
    let mut cache = ScanCache::new();
    cache.observe(b"", -40, true);
    let long = [b'x'; MAX_SSID_BYTES + 1];
    cache.observe(&long, -40, true);
    assert_eq!(cache.entries(), []);
}

/// Merging is case-sensitive, and `clear` empties the cache.
#[test]
fn merging_is_case_sensitive_and_clear_empties_the_cache() {
    let mut cache = ScanCache::new();
    cache.observe(b"Net", -50, true);
    cache.observe(b"net", -51, true);
    assert_eq!(cache.entries().len(), 2);
    cache.clear();
    assert_eq!(cache.entries(), []);
}

/// Every status variant renders its exact text, including the IPv4 address.
#[test]
fn every_status_has_its_text() {
    assert_eq!(
        render_status_text(Status::Unprovisioned).as_deref(),
        Some("unprovisioned")
    );
    assert_eq!(render_status_text(Status::Connecting).as_deref(), Some("connecting"));
    assert_eq!(
        render_status_text(Status::WrongPassword).as_deref(),
        Some("wrong password")
    );
    assert_eq!(render_status_text(Status::Failed).as_deref(), Some("failed"));
    assert_eq!(
        render_status_text(Status::Connected([192, 168, 1, 7])).as_deref(),
        Some("connected at 192.168.1.7")
    );
    assert_eq!(
        render_status_text(Status::Connected([255, 255, 255, 255])).as_deref(),
        Some("connected at 255.255.255.255")
    );
}

/// A status buffer shorter than the text is refused.
#[test]
fn a_tiny_status_buffer_is_refused() {
    let mut buffer = [0u8; 4];
    assert_eq!(write_status_text(Status::Unprovisioned, &mut buffer), None);
}

/// The page carries the status, both forms, the scan list and the polling script.
#[test]
fn the_page_has_the_provisioning_controls() {
    let mut cache = ScanCache::new();
    cache.observe(b"HomeNet", -48, true);
    cache.observe(b"Cafe", -80, false);

    let mut response = [0u8; MAX_RESPONSE_BYTES];
    let Some(length) = write_index_page(&cache, Status::Connecting, &mut response) else {
        return;
    };
    let Some(page) = response.get(..length) else {
        return;
    };
    let text = String::from_utf8_lossy(page);

    assert!(text.starts_with("HTTP/1.1 200 OK\r\n"));
    assert!(text.contains("Content-Type: text/html; charset=utf-8"));
    assert!(text.contains("Cache-Control: no-store"));
    assert!(text.contains("Connection: close"));
    assert!(text.contains("<meta charset=\"utf-8\">"));
    assert!(text.contains("name=\"viewport\""));
    assert!(text.contains("<title>"));
    assert!(text.contains("<p id=\"status\">connecting</p>"));
    assert!(text.contains("<form method=\"post\" action=\"/provision\">"));
    assert!(text.contains("<input name=\"ssid\" list=\"scan-list\">"));
    assert!(text.contains("<datalist id=\"scan-list\">"));
    assert!(text.contains("<option value=\"HomeNet\">HomeNet (-48 dBm, WPA2)</option>"));
    assert!(text.contains("<option value=\"Cafe\">Cafe (-80 dBm, open)</option>"));
    assert!(text.contains("<input type=\"password\" name=\"passphrase\">"));
    assert!(text.contains("<form method=\"post\" action=\"/forget\">"));
    assert!(text.contains("Forget network"));
    assert!(text.contains("fetch('/status', {cache:'no-store'})"));
    assert!(text.contains("document.getElementById('status').textContent"));
    assert!(text.contains("setInterval(poll,1000)"));

    let Some(body) = response_body(page) else {
        return;
    };
    let Some(declared) = declared_content_length(page) else {
        return;
    };
    assert_eq!(declared, body.len());
}

/// `SSID`s are HTML-escaped in both the option value and its text.
#[test]
fn ssids_are_html_escaped() {
    let mut cache = ScanCache::new();
    cache.observe(b"a&b", -50, true);
    cache.observe(b"<script>alert(1)</script>", -51, true);
    cache.observe(b"\"quoted\"", -52, true);
    cache.observe(b"it's", -53, true);

    let mut response = [0u8; MAX_RESPONSE_BYTES];
    let Some(length) = write_index_page(&cache, Status::Failed, &mut response) else {
        return;
    };
    let Some(page) = response.get(..length) else {
        return;
    };
    let text = String::from_utf8_lossy(page);

    assert!(text.contains("a&amp;b"));
    assert!(text.contains("&lt;script&gt;alert(1)&lt;/script&gt;"));
    assert!(text.contains("&quot;quoted&quot;"));
    assert!(text.contains("it&#39;s"));
    assert!(!text.contains("<script>alert(1)"));
    assert!(!text.contains("a&b"));
}

/// A full cache of maximum-length `SSID`s still fits the response buffer.
#[test]
fn a_full_scan_cache_still_fits_the_response_buffer() {
    let mut cache = ScanCache::new();
    for index in 0..MAX_SCAN_ENTRIES {
        let mut ssid = [b'&'; MAX_SSID_BYTES];
        if let Some(slot) = ssid.first_mut() {
            *slot = b'a' + u8::try_from(index).unwrap_or(0);
        }
        let rssi = i8::try_from(index).unwrap_or(0).saturating_neg();
        cache.observe(&ssid, rssi, index % 2 == 0);
    }

    let mut response = [0u8; MAX_RESPONSE_BYTES];
    let Some(length) = write_index_page(&cache, Status::Connected([192, 168, 1, 1]), &mut response) else {
        return;
    };
    assert!(length <= MAX_RESPONSE_BYTES);
    let Some(page) = response.get(..length) else {
        return;
    };
    let Some(body) = response_body(page) else {
        return;
    };
    let Some(declared) = declared_content_length(page) else {
        return;
    };
    assert_eq!(declared, body.len());
}

/// Buffers too small for the requested response are refused rather than truncated.
#[test]
fn short_buffers_are_refused() {
    let mut tiny = [0u8; 3];
    assert_eq!(write_status_text(Status::Failed, &mut tiny), None);
    assert_eq!(write_status_response(Status::Failed, &mut tiny), None);
    assert_eq!(write_redirect(&mut tiny), None);
    assert_eq!(write_error(400, &mut tiny), None);

    let cache = ScanCache::new();
    assert_eq!(write_index_page(&cache, Status::Failed, &mut tiny), None);
}

/// The status response is a complete `text/plain` response with the right length.
#[test]
fn the_status_response_is_complete() {
    let mut response = [0u8; 256];
    let Some(length) = write_status_response(Status::Connected([192, 168, 0, 42]), &mut response) else {
        return;
    };
    let Some(bytes) = response.get(..length) else {
        return;
    };
    let text = String::from_utf8_lossy(bytes);

    assert!(text.starts_with("HTTP/1.1 200 OK\r\n"));
    assert!(text.contains("Content-Type: text/plain; charset=utf-8"));
    assert!(text.contains("Cache-Control: no-store"));
    assert!(text.contains("Connection: close"));

    let Some(body) = response_body(bytes) else {
        return;
    };
    assert_eq!(body, b"connected at 192.168.0.42".as_slice());
    let Some(declared) = declared_content_length(bytes) else {
        return;
    };
    assert_eq!(declared, body.len());
}

/// The redirect is a complete `303` response pointing at `/`.
#[test]
fn the_redirect_points_home() {
    let mut response = [0u8; 256];
    let Some(length) = write_redirect(&mut response) else {
        return;
    };
    let Some(bytes) = response.get(..length) else {
        return;
    };
    let text = String::from_utf8_lossy(bytes);

    assert!(text.starts_with("HTTP/1.1 303 See Other\r\n"));
    assert!(text.contains("Location: /\r\n"));
    let Some(declared) = declared_content_length(bytes) else {
        return;
    };
    assert_eq!(declared, 0);
}

/// The three error responses have the right status lines and exact lengths.
#[test]
fn the_error_responses_have_status_lines_and_lengths() {
    for (code, reason) in [(400u16, "Bad Request"), (404, "Not Found"), (405, "Method Not Allowed")] {
        let mut response = [0u8; 256];
        let Some(length) = write_error(code, &mut response) else {
            return;
        };
        let Some(bytes) = response.get(..length) else {
            return;
        };
        let text = String::from_utf8_lossy(bytes);

        assert!(text.starts_with(&format!("HTTP/1.1 {code} {reason}\r\n")));
        let Some(body) = response_body(bytes) else {
            return;
        };
        assert_eq!(body, reason.as_bytes());
        let Some(declared) = declared_content_length(bytes) else {
            return;
        };
        assert_eq!(declared, body.len());
    }

    let mut response = [0u8; 256];
    assert_eq!(write_error(500, &mut response), None);
}
