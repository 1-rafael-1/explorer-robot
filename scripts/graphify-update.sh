#!/usr/bin/env bash
# Check or refresh the graphify knowledge graph for this repository.
#
#   scripts/graphify-update.sh check   # is the graph older than the working tree?
#   scripts/graphify-update.sh code    # free, local AST refresh (Rust/JS/shell)
#   scripts/graphify-update.sh docs    # semantic pass over docs + code (needs a key)
#   scripts/graphify-update.sh full    # force a full extract + relabel + export
#   scripts/graphify-update.sh label   # rename communities only (cheap)
#
# The semantic pass reads DEEPSEEK_API_KEY from GRAPHIFY_ENV_FILE
# (default: ~/.config/deepseek.env, mode 600). Never commit that file.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/graphify-out"
ENV_FILE="${GRAPHIFY_ENV_FILE:-$HOME/.config/deepseek.env}"
GRAPH="$OUT/graph.json"

# DeepSeek caps a response at 16384 tokens. 20000 puts too many docs in one
# chunk and truncates a large file (CONTEXT.md) instead of splitting it; 6000
# makes graphify slice it and finish. Do not raise this without re-testing.
TOKEN_BUDGET="${GRAPHIFY_TOKEN_BUDGET:-6000}"

export PATH="$HOME/.local/bin:$PATH"
export GRAPHIFY_DISABLE_THINKING=1

usage() {
    sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

load_key() {
    if [[ ! -f "$ENV_FILE" ]]; then
        echo "error: $ENV_FILE not found; the semantic pass needs DEEPSEEK_API_KEY" >&2
        echo "       set GRAPHIFY_ENV_FILE to point at your env file" >&2
        exit 1
    fi
    set -a
    # shellcheck disable=SC1090
    . "$ENV_FILE"
    set +a
    if [[ -z "${DEEPSEEK_API_KEY:-}" ]]; then
        echo "error: $ENV_FILE does not export DEEPSEEK_API_KEY" >&2
        exit 1
    fi
}

# Source files touched in the working tree, excluding generated/vendored paths.
changed_sources() {
    {
        git -C "$ROOT" --no-optional-locks diff --name-only HEAD 2>/dev/null || true
        git -C "$ROOT" --no-optional-locks ls-files --others --exclude-standard 2>/dev/null || true
    } | grep -E '\.(rs|toml|py|js|sh|md)$' \
      | grep -v -E '^(graphify-out/|\.scratch/|target/|[^/]+/target/)' \
      || true
}

check() {
    if [[ ! -f "$GRAPH" ]]; then
        echo "no graph at $GRAPH"
        echo "build one with: scripts/graphify-update.sh full"
        return 1
    fi

    echo "graph marker: $(cat "$OUT/.graphify_build.json" 2>/dev/null || echo unknown)"
    echo "graph build:  $(date -r "$GRAPH" '+%Y-%m-%d %H:%M')"

    graphify check-update "$ROOT" || true

    # mtime is unreliable for untouched files (builds and rust-analyzer touch
    # them), so only compare files that git says have changed.
    local stale_code="" stale_docs="" f
    while IFS= read -r f; do
        [[ -n "$f" ]] || continue
        if [[ "$ROOT/$f" -nt "$GRAPH" ]]; then
            if [[ "$f" == *.md ]]; then
                stale_docs+="$f"$'\n'
            else
                stale_code+="$f"$'\n'
            fi
        fi
    done < <(changed_sources)

    if [[ -n "$stale_code" ]]; then
        echo "code changed since the graph:"
        printf '%s' "$stale_code" | sed 's/^/  /'
    fi
    if [[ -n "$stale_docs" ]]; then
        echo "docs changed since the graph:"
        printf '%s' "$stale_docs" | sed 's/^/  /'
    fi

    if [[ -n "$stale_code" ]]; then
        echo "refresh code: scripts/graphify-update.sh code"
    fi
    if [[ -n "$stale_docs" ]]; then
        echo "refresh docs+code: scripts/graphify-update.sh docs"
    fi
    if [[ -z "$stale_code" && -z "$stale_docs" ]]; then
        echo "fresh: every changed source is older than the graph"
    fi
}

code() {
    graphify update "$ROOT"
    echo "graph: $GRAPH"
}

# Extract, then relabel and export. `label` re-clusters and writes the report;
# its own export is aggregated above 5000 nodes, so export again at 12000.
docs() {
    load_key
    graphify extract "$ROOT" --backend deepseek --token-budget "$TOKEN_BUDGET"
    graphify label "$ROOT" --backend deepseek --batch-size 50 --missing-only
    graphify export html --node-limit 12000
    echo "graph: $GRAPH"
    echo "html:  $OUT/graph.html"
}

full() {
    load_key
    graphify extract "$ROOT" --backend deepseek --token-budget "$TOKEN_BUDGET" --force
    graphify label "$ROOT" --backend deepseek --batch-size 50
    graphify export html --node-limit 12000
    echo "graph: $GRAPH"
    echo "html:  $OUT/graph.html"
}

label() {
    load_key
    graphify label "$ROOT" --backend deepseek --batch-size 50
    graphify export html --node-limit 12000
    echo "labels: $OUT/.graphify_labels.json"
}

case "${1:-check}" in
    check) check ;;
    code) code ;;
    docs) docs ;;
    full) full ;;
    label) label ;;
    -h|--help|help) usage ;;
    *)
        echo "unknown command: $1" >&2
        usage >&2
        exit 2
        ;;
esac
