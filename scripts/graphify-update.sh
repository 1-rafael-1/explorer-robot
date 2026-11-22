#!/usr/bin/env bash
# Check or refresh the graphify knowledge graph for this repository.
#
#   scripts/graphify-update.sh check   # compare the graph to the corpus
#   scripts/graphify-update.sh update  # incremental docs+code refresh (needs a key)
#   scripts/graphify-update.sh full    # forced full docs+code extract + relabel + export
#   scripts/graphify-update.sh label   # (re)name communities only (cheap)
#
# Every build includes docs, ADRs and architecture notes. There is no code-only
# mode: a code-only graph omits the decision records the repository relies on.
# `docs` is accepted as a deprecated alias for `update`; `code` is refused.
#
# The semantic pass reads DEEPSEEK_API_KEY from GRAPHIFY_ENV_FILE
# (default: ~/.config/deepseek.env, mode 600). Never commit that file.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/graphify-out"
ENV_FILE="${GRAPHIFY_ENV_FILE:-$HOME/.config/deepseek.env}"
GRAPH="$OUT/graph.json"
CORPUS_MARKER="$OUT/.graphify_corpus"

# DeepSeek caps a response at 16384 tokens. 20000 puts too many docs in one
# chunk and truncates a large file (CONTEXT.md) instead of splitting it; 6000
# makes graphify slice it and finish. Do not raise this without re-testing.
TOKEN_BUDGET="${GRAPHIFY_TOKEN_BUDGET:-6000}"

export PATH="$HOME/.local/bin:$PATH"
export GRAPHIFY_DISABLE_THINKING=1

usage() {
    cat <<'EOF'
Usage: scripts/graphify-update.sh <command>

  check    compare the graph to the current corpus
  update   incremental docs+code refresh (needs DEEPSEEK_API_KEY)
  full     forced full docs+code extract + relabel + export
  label    rename communities only (cheap)
  help     show this help

Every build includes docs, ADRs and architecture notes. There is no code-only
mode. `docs` is a deprecated alias for `update`; `code` is refused.
EOF
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

# Every source file graphify reads, relative to ROOT. Excludes generated and
# scratch paths; `.scratch/` is deliberately outside the graph.
corpus_files() {
    {
        git -C "$ROOT" --no-optional-locks ls-files -z
        git -C "$ROOT" --no-optional-locks ls-files --others --exclude-standard -z
    } | tr '\0' '\n' \
      | grep -E '\.(rs|toml|py|js|sh|md)$' \
      | grep -v -E '^(graphify-out/|\.scratch/|target/|[^/]+/target/)' \
      | sort -u
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

# A content fingerprint of the corpus, independent of commit and file mtimes.
# The graph is current only when this matches the value recorded by the last
# successful update/full.
corpus_fingerprint() {
    local f
    while IFS= read -r f; do
        [[ -f "$ROOT/$f" ]] || continue
        printf '%s\0' "$f"
        sha256sum "$ROOT/$f" | cut -d' ' -f1
    done < <(corpus_files) | sha256sum | cut -d' ' -f1
}

record_corpus() {
    corpus_fingerprint > "$CORPUS_MARKER"
}

code_only_build() {
    [[ "$(cat "$OUT/.graphify_build.json" 2>/dev/null || true)" == *'"code_only": true'* ]]
}

check() {
    if [[ ! -f "$GRAPH" ]]; then
        echo "no graph at $GRAPH"
        echo "build one with: scripts/graphify-update.sh full"
        return 1
    fi

    echo "graph marker: $(cat "$OUT/.graphify_build.json" 2>/dev/null || echo unknown)"
    echo "graph build:  $(date -r "$GRAPH" '+%Y-%m-%d %H:%M')"

    local stale=0 expected actual
    expected="$(cat "$CORPUS_MARKER" 2>/dev/null || true)"
    actual="$(corpus_fingerprint)"
    if [[ -z "$expected" ]]; then
        echo "no corpus fingerprint recorded; run update to create one"
        stale=1
    elif [[ "$expected" != "$actual" ]]; then
        echo "corpus changed since the graph was built"
        stale=1
    fi

    if code_only_build; then
        echo "last build was code-only; docs/ADRs are missing from the graph"
        stale=1
    fi

    # List uncommitted changes for context. The fingerprint above is the gate;
    # a file touched without a content change does not make the graph stale.
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
        echo "uncommitted code changes:"
        printf '%s' "$stale_code" | sed 's/^/  /'
    fi
    if [[ -n "$stale_docs" ]]; then
        echo "uncommitted docs changes:"
        printf '%s' "$stale_docs" | sed 's/^/  /'
    fi

    if [[ "$stale" -eq 0 ]]; then
        echo "fresh: graph matches the corpus"
        return 0
    fi
    echo "refresh: scripts/graphify-update.sh update"
    echo "forced:  scripts/graphify-update.sh full"
    return 1
}

# Code-only updates are deliberately unavailable: they drop docs and ADRs from
# the graph and make it re-cluster with hub names.
code() {
    echo "error: code-only graph updates are disabled; the graph always includes docs/ADRs." >&2
    echo "       use: scripts/graphify-update.sh update" >&2
    exit 2
}

update() {
    load_key
    graphify extract "$ROOT" --backend deepseek --token-budget "$TOKEN_BUDGET"
    graphify label "$ROOT" --backend deepseek --batch-size 50 --missing-only
    graphify export html --node-limit 12000
    record_corpus
    echo "graph: $GRAPH"
    echo "html:  $OUT/graph.html"
}

docs() {
    echo "note: 'docs' is a deprecated alias for 'update'; the graph always includes docs." >&2
    update
}

full() {
    load_key
    graphify extract "$ROOT" --backend deepseek --token-budget "$TOKEN_BUDGET" --force
    graphify label "$ROOT" --backend deepseek --batch-size 50
    graphify export html --node-limit 12000
    record_corpus
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
    update) update ;;
    docs) docs ;;
    full) full ;;
    label) label ;;
    code) code ;;
    -h|--help|help) usage ;;
    *)
        echo "unknown command: $1" >&2
        usage >&2
        exit 2
        ;;
esac
