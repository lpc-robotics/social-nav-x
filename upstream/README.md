# Upstream reconstruction

`manifest.tsv` is the authoritative inventory of every nested source repository
used by the archived workspace.

- `vendored`: the complete current source is committed directly under `src/`;
  its patch is retained for provenance and comparison with the recorded base.
- `fetched`: `scripts/fetch_upstreams.sh` clones the exact commit into the
  recorded path.
- A non-`-` patch is automatically checked and applied after checkout.

The reconstruction script refuses to overwrite a non-Git path, rejects a
different existing commit, and never resets dirty work.
