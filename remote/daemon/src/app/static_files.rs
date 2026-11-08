//! Static-file serving for the SPA (ADR-0039).
//!
//! The app is a folder with no build step and no hashed asset names, so every response is
//! `no-store` and carries the same strict headers. Resolution is deliberately small: an existing
//! file wins, an extensionless path falls back to `index.html`, a missing asset with an extension
//! is 404, and `/ws` is never a fallback. The folder is embedded at compile time with a disk
//! override for `<path>` development (`--assets` in ticket 13).

use std::path::{Path, PathBuf};

use axum::{
    body::Body,
    extract::State,
    http::{HeaderValue, Method, StatusCode, Uri, header},
    response::Response,
};

use super::AppState;

/// The Content Security Policy ADR-0039 fixes for every static response.
pub const CSP: &str = "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; \
                       img-src 'self' data:; connect-src 'self' ws: wss:; object-src 'none'; \
                       base-uri 'none'; frame-ancestors 'none'";

/// The compile-time embed of `remote/app` (ADR-0039).
static EMBEDDED: include_dir::Dir<'static> = include_dir::include_dir!("$CARGO_MANIFEST_DIR/../app");

/// Where the SPA's bytes come from.
#[derive(Clone, Debug, Default)]
pub enum Assets {
    /// The compile-time embed; the production default.
    #[default]
    Embedded,
    /// A directory on disk, used by `--assets <path>` during development.
    Disk(PathBuf),
}

impl Assets {
    /// Serves the embedded app folder.
    #[must_use]
    pub const fn embedded() -> Self {
        Self::Embedded
    }

    /// Serves the folder at `path` from disk.
    #[must_use]
    pub fn disk(path: impl Into<PathBuf>) -> Self {
        Self::Disk(path.into())
    }

    /// Loads one already-validated relative asset path.
    ///
    /// `relative` must not contain `..`, an absolute root or a percent escape; [`target`]
    /// guarantees that for every request path.
    async fn load(&self, relative: &str) -> Option<Vec<u8>> {
        match self {
            Self::Embedded => EMBEDDED.get_file(relative).map(|file| file.contents().to_vec()),
            Self::Disk(root) => {
                let path = root.join(relative);
                tokio::task::spawn_blocking(move || std::fs::read(path).ok())
                    .await
                    .ok()
                    .flatten()
            }
        }
    }
}

/// What a request path resolves to before the filesystem is consulted.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Target {
    /// The SPA entry point.
    Index,
    /// One relative asset path.
    File(String),
    /// A path that must not be served, including every `/ws` variant.
    NotFound,
}

/// Resolves a request path into a [`Target`].
///
/// The empty path and `/` are the index. Any segment that is empty (except from a doubled or
/// trailing slash), `.`, `..` or contains a percent escape is refused rather than decoded, so no
/// encoded traversal can leave the asset root. The first segment `ws` is reserved for the
/// WebSocket and is never a static fallback.
fn target(path: &str) -> Target {
    let trimmed = path.trim_start_matches('/');
    if trimmed.is_empty() {
        return Target::Index;
    }
    let mut segments = Vec::new();
    for segment in trimmed.split('/').filter(|segment| !segment.is_empty()) {
        if segment == "." || segment == ".." || segment.contains('%') {
            return Target::NotFound;
        }
        segments.push(segment);
    }
    if segments.is_empty() {
        return Target::Index;
    }
    if segments.first().is_some_and(|first| first.eq_ignore_ascii_case("ws")) {
        return Target::NotFound;
    }
    Target::File(segments.join("/"))
}

/// The `Content-Type` for an asset path.
fn content_type(path: &str) -> &'static str {
    let extension = Path::new(path)
        .extension()
        .and_then(|extension| extension.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    match extension.as_str() {
        "html" | "htm" => "text/html; charset=utf-8",
        "css" => "text/css; charset=utf-8",
        "js" | "mjs" => "text/javascript; charset=utf-8",
        "json" => "application/json",
        "svg" => "image/svg+xml",
        "png" => "image/png",
        "jpg" | "jpeg" => "image/jpeg",
        "webp" => "image/webp",
        "ico" => "image/x-icon",
        "woff2" => "font/woff2",
        "woff" => "font/woff",
        "wasm" => "application/wasm",
        "txt" | "md" => "text/plain; charset=utf-8",
        _ => "application/octet-stream",
    }
}

/// Builds a static response with ADR-0039's headers on both success and failure.
fn respond(status: StatusCode, content_type: &'static str, body: Vec<u8>) -> Response {
    let mut response = Response::new(Body::from(body));
    *response.status_mut() = status;
    let headers = response.headers_mut();
    headers.insert(header::CONTENT_TYPE, HeaderValue::from_static(content_type));
    headers.insert(header::X_CONTENT_TYPE_OPTIONS, HeaderValue::from_static("nosniff"));
    headers.insert(header::REFERRER_POLICY, HeaderValue::from_static("no-referrer"));
    headers.insert(header::CACHE_CONTROL, HeaderValue::from_static("no-store"));
    headers.insert(header::CONTENT_SECURITY_POLICY, HeaderValue::from_static(CSP));
    response
}

/// The 404 every missing static asset returns.
fn not_found() -> Response {
    respond(StatusCode::NOT_FOUND, "text/plain; charset=utf-8", Vec::new())
}

/// Serves one request from the embedded or on-disk app folder.
pub async fn static_handler(State(state): State<AppState>, uri: Uri, method: Method) -> Response {
    if method != Method::GET && method != Method::HEAD {
        return respond(StatusCode::METHOD_NOT_ALLOWED, "text/plain; charset=utf-8", Vec::new());
    }
    let empty = method == Method::HEAD;
    let assets = &state.assets;
    match target(uri.path()) {
        Target::NotFound => not_found(),
        Target::Index => serve_index(assets, empty).await,
        Target::File(relative) => {
            if let Some(body) = assets.load(&relative).await {
                return respond(
                    StatusCode::OK,
                    content_type(&relative),
                    if empty { Vec::new() } else { body },
                );
            }
            if Path::new(&relative).extension().is_some() {
                not_found()
            } else {
                serve_index(assets, empty).await
            }
        }
    }
}

/// Serves `index.html` or the ADR-0039 404 when the folder has no entry point.
async fn serve_index(assets: &Assets, empty: bool) -> Response {
    assets.load("index.html").await.map_or_else(not_found, |body| {
        respond(
            StatusCode::OK,
            "text/html; charset=utf-8",
            if empty { Vec::new() } else { body },
        )
    })
}

#[cfg(test)]
mod tests {
    use super::{CSP, Target, content_type, target};

    /// A missing asset with an extension is 404; an extensionless one falls back.
    #[test]
    fn a_path_resolves_to_the_index_or_a_file() {
        assert_eq!(target("/"), Target::Index);
        assert_eq!(target("/deep/route"), Target::File(String::from("deep/route")));
        assert_eq!(target("/deep/route/"), Target::File(String::from("deep/route")));
        assert_eq!(target("/../Cargo.toml"), Target::NotFound);
        assert_eq!(target("/%2e%2e/Cargo.toml"), Target::NotFound);
        assert_eq!(target("/ws"), Target::NotFound);
        assert_eq!(target("/ws/"), Target::NotFound);
    }

    /// The CSP literal is the ADR's, not an approximation.
    #[test]
    fn the_csp_is_the_documented_one() {
        assert_eq!(
            CSP,
            "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self' ws: wss:; object-src 'none'; base-uri 'none'; frame-ancestors 'none'"
        );
        assert!(!CSP.contains("  "), "the CSP must not contain doubled spaces");
    }

    /// Content types follow the extension, and an unknown one is not sniffed.
    #[test]
    fn content_types_follow_the_extension() {
        assert_eq!(content_type("index.html"), "text/html; charset=utf-8");
        assert_eq!(content_type("app.js"), "text/javascript; charset=utf-8");
        assert_eq!(content_type("app.unknown"), "application/octet-stream");
    }
}
