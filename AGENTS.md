## Agent skills

### Issue tracker

Local markdown — issues live as files under `.scratch/<feature>/`. See `docs/agents/issue-tracker.md`.

### Triage labels

Default canonical roles: `needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix`. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context — `CONTEXT.md` + `docs/adr/` at the repo root. See `docs/agents/domain.md`.

### Code graph

A graphify knowledge graph is generated into `graphify-out/` (gitignored). See
`docs/agents/graphify.md` for what it contains, when to refresh it and how to query it;
refresh with `scripts/graphify-update.sh check|update|full`. Every build includes
docs, ADRs and code; there is no code-only mode. Treat the code as the source of
truth.

### Motor bench

Hardware-in-the-loop motor testing (auto-start recipe, power/floor caveats, RTT
parsing, and the corrected-divergence harness) is in `docs/agents/motor-bench.md`.
Use it before touching the panel or changing the measurement code.
