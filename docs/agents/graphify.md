# Code graph (graphify)

A **graphify** knowledge graph of this repository lives in `graphify-out/`. It is a
derived cache for navigation and impact analysis — the code and the docs are the source
of truth. Do not commit `graphify-out/`; it is generated, large and changes wholesale.

## What is in it

- `graph.json` — nodes and edges for code and docs (≈10 MB).
- `GRAPH_REPORT.md` — community hubs and a textual summary.
- `graph.html` — the interactive view.
- `.graphify_labels.json` — semantic community names.
- `.graphify_build.json` — whether the last build was code-only.
- `.graphify_corpus` — the SHA-256 fingerprint of the code/docs corpus recorded by the last `update`/`full`; `check` compares it.
- `cache/` — AST and semantic caches.

The corpus excludes `.scratch/` (gitignored) and `target/`, so specs and tickets are not
in the graph. That is deliberate: they are short-lived and the graph is for the
codebase.

## One command

```sh
scripts/graphify-update.sh check   # is the graph current for the corpus?
scripts/graphify-update.sh update  # incremental docs + code refresh (needs a key)
scripts/graphify-update.sh full    # forced full docs + code extract + relabel + export
scripts/graphify-update.sh label   # rename communities only (cheap)
```

`update` and `full` always extract docs and code together. There is no code-only
mode: a code-only graph omits the ADRs and glossary the repository relies on.
`docs` is accepted as a deprecated alias for `update`; `code` is refused. `update`
and `full` call DeepSeek and cost a small amount (well under a dollar per rebuild
on this corpus); `label` also needs the key, while `check` is free.

## Known pitfalls

- **DeepSeek caps one response at 16 384 tokens.** A token budget that packs too many
docs into one chunk truncates a large file instead of splitting it: at `--token-budget
20000`, `CONTEXT.md` came back 157 nodes instead of 190 and the shrink guard refused to
overwrite. The script uses **6000** and works; override with `GRAPHIFY_TOKEN_BUDGET` only
if you re-test.
- **Never reach for `--allow-partial` first.** It overwrites a complete graph with the
truncated one, deleting the missing concept nodes. Only use it after comparing the node
counts and confirming the loss is genuinely stale content.
- **Never run `graphify update` directly.** It re-extracts code only, drops the
docs/ADRs from the semantic graph and re-clusters with hub names. The script has
no code-only mode; use `update`. If `.graphify_build.json` says `"code_only":
true`, recover with `update` (or `full` if the manifest refuses).
- **`check` is content-based.** It compares a SHA-256 fingerprint of the corpus
recorded by the last `update`/`full`; it does not trust file mtimes or the graph's
timestamp. A graph rebuilt by another process without the fingerprint is reported
stale, so run `update` once after cloning or restoring a graph.
- **The aggregated HTML view.** Anything above 5000 nodes is exported as community nodes;
`export html --node-limit 12000` (what the script runs) gives the node-level view.

## Prerequisites

- The CLI: `graphify` on `PATH` (`~/.local/bin`), installed with the OpenAI extra:
  `uv tool install "graphifyy[openai]" --force`.
- The key: `~/.config/deepseek.env` (mode 600), exporting `DEEPSEEK_API_KEY`. The script
  sources it via `GRAPHIFY_ENV_FILE` (default that path). Never commit it.
- Thinking must be off: the script exports `GRAPHIFY_DISABLE_THINKING=1`. The DeepSeek
  models on this key are reasoning models; with thinking on, the reasoning tokens truncate
  the JSON response and graphify silently drops docs.

## When to update

| Change | Command |
| --- | --- |
| Any code and/or docs change | `update` |
| New corpus, changed clustering, first build | `full` |
| Community names only | `label` |

`update` is incremental: the manifest gate re-extracts only changed files. If a rebuild
refuses to overwrite because it produced fewer nodes, read the reason first. `full` is the
forced escape hatch; a semantic shrink is almost always a truncated chunk, so lower the
token budget rather than forcing the write.

## Querying

Prefer `explain`, `path` and `affected` over `query`. `query` is keyword BFS: generic
words seed on hub nodes and flood.

```sh
graphify explain "robot/crates/models/motor-cal/src/apply.rs::resolve_track_speed"
graphify path "A" "B" --undirected
graphify affected "X" --depth 2
graphify god-nodes --top 20
graphify section "docs/architecture.md#some-heading"
```

Name collisions are common because docs contribute nodes: `LinkCore` matches several.
Use `path::symbol`; exit code 3 lists candidates, exit code 4 means no match. Every edge
is marked `EXTRACTED` (read from source) or `INFERRED` (resolved by graphify); treat
`INFERRED` as a hint, not evidence.

## Do not commit the graph

Commit the recipe and the script, not the output. The JSON is ~10 MB, diffs unusably on
every rebuild, and conflicts on every branch. If a second machine or a CI job ever needs
it, publish it as a release artifact or on a separate orphan branch — not on `main`.

A repo-local skill is redundant: the global `graphify` skill already exists in
`~/.agents/skills/graphify/`. The reliable hook into every agent session is this file plus
the pointer in `AGENTS.md`, which are read at startup.

## Optional hook

A non-blocking pre-commit warning is safe; running the LLM pass in a hook is not (network,
cost, time, key). To add the warning:

```sh
cat > .git/hooks/pre-commit <<'EOF'
#!/bin/sh
[ -x scripts/graphify-update.sh ] && scripts/graphify-update.sh check || true
EOF
chmod +x .git/hooks/pre-commit
```

Git hooks are per clone; if this should be shared, commit a `scripts/install-hooks.sh`
and mention it in the README instead.
