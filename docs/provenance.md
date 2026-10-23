# Provenance and reproducibility

## Experiment source

The author supplied `cluster_code/` together with the saved outputs and
confirmed that it is the sole surviving version used for those experiments.
The root C++ sources and headers were copied from that snapshot. The
historical snapshot's SHA-256 digests are recorded in
[experiment-source.json](experiment-source.json) and remain unchanged.
The working sources have since received a behavior-preserving cleanup of
unused private helpers, disabled experiments, and stale diagnostics, plus
brief implementation comments. They are no longer byte-identical to the
experiment snapshot. Seeded before/after checks compare solver outcomes
with runtime measurements excluded.

Build/test integration and submission-script packaging have also been
updated; these are not claimed to be the historical build configuration.

No historical Git revision, compiler version, optimization flags, operating
system version, or OpenMP runtime version was preserved. The author-supplied
manuscript reports AMD EPYC 7543 CPUs at 3.1 GHz and two threads for ACO.
That is documentary hardware information, not a recovered node inventory.

The run parameters and scenarios are represented in the submission scripts
and manifests. The manuscript, benchmark records, tables, and result figures
remain local and are excluded from publication in this repository.

## Synthetic instances

The generator is [tem-kurbanov/art_city](https://github.com/tem-kurbanov/art_city).
The public revision used for reconstruction is
[`d9d9a225b5d3c82bcaf36ba7b4a2a8a54ede4d87`](https://github.com/tem-kurbanov/art_city/tree/d9d9a225b5d3c82bcaf36ba7b4a2a8a54ede4d87).
Its `main.py` SHA-256 is
`1d956a58edb492a85ad9b5d5dd8d1ce281845f497397c17ef6365b7a238cdfd0`.

All 60 bundled CSVs were reconstructed byte-for-byte from their recorded
parameters and this generator, using the serialization described below.

At the time of investigation, the repository's two public commits had identical `generate_graph` logic;
the later commit removes blank lines from CSV export. Consequently, matching
the generated graphs cannot uniquely identify which original checkout was
used. The historical revision remains unknown.

The bundled filenames encode node count, correlated and uncorrelated cost
counts, and seed. `art/manifest.json` records those values and the original
file checksums. Generation uses the upstream defaults for its remaining
parameters: a minimum angle of 60 degrees and up to 1,000 attempts. The `c`
and `u` family labels follow the generator's terminology; they are not a claim
that the observed costs have a particular measured correlation.

The original export command is unavailable. The bundled files preserve the
generator's edge insertion order and use LF newlines, with `c1`/`u1` cost
labels according to family. Upstream `save_graph_csv` sorts edges by endpoint
and labels both families `c1`. `tools/verify_instances.py` therefore
calls `generate_graph` with the recorded settings and reconstructs the
observed CSV serialization in memory. It never rewrites an input file.

To repeat the reconstruction with a local copy of the pinned public source:

```sh
mkdir -p .cache/art_city
curl -fL https://raw.githubusercontent.com/tem-kurbanov/art_city/d9d9a225b5d3c82bcaf36ba7b4a2a8a54ede4d87/main.py \
  -o .cache/art_city/main.py
MPLBACKEND=Agg MPLCONFIGDIR="$PWD/.cache/matplotlib" \
  .venv/bin/python tools/verify_instances.py \
  --generator .cache/art_city/main.py --jobs 4
```

The verifier checks the generator source hash before importing it. Full
reconstruction is substantially slower than the default checksum/structure
check and is intentionally separate from routine CI.

## Future runs

`tools/capture_provenance.py` records the current Git revision and dirty state,
root solver-source hashes, binary hash, available CMake cache settings,
compiler identification, platform, and Python package versions. Journal jobs
write this snapshot to their ignored bundle-log directory.

A snapshot identifies the files present at capture time. It does not attest
that an existing binary was compiled from those sources, nor does it fill in
missing historical information. Retain it with the exact command, inputs,
and outputs of a future run.

The code, documentation, bundled synthetic input graphs, and scenario
manifests are available under the [MIT License](../LICENSE). The unpublished
manuscript and withheld experimental results are excluded from this release
and this license grant.
