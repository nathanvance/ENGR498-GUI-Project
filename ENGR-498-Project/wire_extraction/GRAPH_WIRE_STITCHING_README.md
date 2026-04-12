# Ordered Span-Partition Wire Stitching

## Purpose

This module implements the post-processing wire-stitching stage that runs after
the existing wire extraction pipeline. The goal is to recover fragmented wire
segments while preserving the span ownership needed for sag measurement.

The implementation lives in:

- `wire_extraction/stitching.py`
- `wire_extraction/pipeline.py`

The pipeline writes:

- `wires_points.npz`: final stitched spans used downstream
- `wire_info.json`: final span metadata
- `wire_clusters_pre_stitch.npz`: extracted clusters before stitching
- `wire_clusters_pre_stitch_info.json`: metadata for the pre-stitch clusters
- `wire_stitching_report.json`: graph, track, and partition diagnostics

## Motivation

Sparse LiDAR sampling often breaks one physical span into several disconnected
clusters. A post-processing stitcher is therefore necessary.

However, there is a second failure mode near utility-pole support regions:

- one conductor passes over a support
- the conductor forms two different catenary spans
- both spans lie on the same lateral track
- the spans are close in space and direction

For sag measurement, those two spans must remain separate.

The key problem is ownership. A final algorithm must decide which extracted
clusters belong to the left span and which belong to the right span. If span
assignment is done by greedy merge growth, one span can absorb clusters that
should belong to the adjacent span and leave only fragmented leftovers behind.

That is the failure this algorithm is designed to fix.

## Why Greedy Stitching Fails At Support Regions

The earlier graph-based stitcher improved fragmented-wire recovery by using:

- direction similarity
- plane compatibility
- lateral-track consistency
- catenary plausibility

That is a strong candidate-track builder, but it is still not the right final
decision rule for support regions.

A greedy merge procedure makes local decisions such as:

- "this neighboring cluster looks compatible, so absorb it"
- "this merged set still fits one catenary well enough, so keep growing"

Those local decisions do not enforce exclusive ownership across the full track.
As a result:

- one span may steal clusters from the adjacent span
- the remaining clusters are then forced into inferior fragments
- the final output is biased for sag measurement

## Why "One Versus Two Catenaries" Alone Is Not Enough

A simple merge filter such as:

- "reject a merge if two catenaries fit better than one"

is not sufficient by itself.

It still works on local merge proposals, not on the full assignment problem. A
cluster near the support can often plausibly fit either neighboring span. If the
algorithm only asks whether one particular union is acceptable, it can still
produce:

- one oversized span that swallowed ambiguous clusters
- one undersized leftover span

The core issue is not just goodness of fit. It is global ownership.

## Ownership Problem

Given an ordered set of extracted clusters on one conductor track, every cluster
must be assigned to exactly one final span.

The required constraints are:

- no cluster can belong to two spans
- no cluster can be left ambiguous
- final spans must be contiguous along the track
- each final span must be explainable by one catenary

This makes the final stitch stage an ordered partition problem, not a greedy
merge problem.

## High-Level Algorithm

The new final stage is split into two parts:

1. candidate track grouping
2. ordered span partitioning inside each track

The first stage uses graph logic to decide which clusters lie on the same wire
track. The second stage uses dynamic programming to decide how to split that
track into final spans.

This separation is important:

- graph edges are good for track compatibility
- dynamic programming is good for exclusive span ownership

## Pipeline Placement

The upstream extraction stage is unchanged.

Pipeline order:

1. ground segmentation
2. wire-point extraction
3. Euclidean clustering and existing extractor heuristics
4. minimum span filtering
5. graph-based candidate-track construction
6. ordered dynamic-programming span partitioning
7. final stitched span outputs

The new logic replaces the old greedy final span merge behavior. It does not
replace the extractor.

## Inputs And Outputs

### Input

The stitcher consumes extracted clusters:

- each cluster is an `N x 3` array of `(x, y, z)` points
- all clusters are in the same local map frame

### Output

The stitcher produces:

- final stitched spans
- one metadata entry per final span
- a report with:
  - candidate edges
  - initial graph components
  - refined candidate tracks
  - final DP-selected partitions

## Stage 1: Candidate Track Grouping

The current graph logic is retained for candidate track construction.

Each extracted cluster is a node. Candidate edges are created only for cluster
pairs that pass deterministic geometric gating:

- plane compatibility
- direction similarity
- plausible projected gap
- lateral-track compatibility
- side-by-side wire rejection
- pairwise catenary plausibility

Connected components of accepted edges are interpreted as candidate tracks.

If a candidate component is laterally inconsistent or clearly multi-track, the
component is recursively split before span partitioning. This cleanup step is
about track identity only, not about final span ownership.

## Local `(u, v, z)` Coordinate System

Every candidate track receives a local wire-aligned frame.

### Axes

For a candidate track:

- `u_hat`: along-track or along-span direction
- `v_hat`: cross-track lateral direction
- `z_hat`: in-plane vertical or sag direction

In the implementation:

- `u_hat` is the dominant horizontal span direction from PCA
- `v_hat = normalize(u_hat x world_up)`
- `z_hat = normalize(v_hat x u_hat)`

This yields local coordinates:

- `u`: position along the conductor track
- `v`: lateral track coordinate
- `z`: vertical coordinate inside the wire plane

### Projection

For point `p` and track origin `p0`:

- `u = (p - p0) dot u_hat`
- `v = (p - p0) dot v_hat`
- `z = (p - p0) dot z_hat`

The catenary model is fit in `(u, z)` coordinates. The `v` coordinate is used
for track-consistency checks.

## Mathematical Definitions

### Centroid

For cluster points `p_k in R^3`:

`c = (1 / N) * sum_k p_k`

### Principal Direction

With centered points `q_k = p_k - c`, the covariance matrix is:

`Sigma = (1 / (N - 1)) * sum_k q_k q_k^T`

The dominant eigenvector of `Sigma` is the principal direction. Its horizontal
projection defines `u_hat`.

### Point-To-Plane Distance

Let the local wire plane pass through `p0` with normal `v_hat`. Then:

`dist_plane(p) = |(p - p0) dot v_hat|`

The plane RMS is:

`plane_rms = sqrt((1 / N) * sum_k dist_plane(p_k)^2)`

### Catenary Fit Residual

For projected points `(u_k, z_k)`, the fitted catenary is:

`z(u) = a * cosh((u - u0) / a) + z0`

Residuals are:

`r_k = z_k - z_hat_k`

and the root-mean-square error is:

`RMSE = sqrt((1 / N) * sum_k r_k^2)`

## Ordered Cluster Sequence

Inside each candidate track:

1. project each cluster into the local track frame
2. compute its along-track interval `[u_min, u_max]`
3. compute a representative center such as `median(u)`
4. sort clusters by that along-track center

This creates an ordered sequence:

`C_0, C_1, ..., C_{n-1}`

The final span assignment is restricted to contiguous blocks of this sequence.

That contiguity requirement is essential. It prevents a left span from taking a
cluster, skipping over a middle cluster, and then taking another cluster farther
to the right.

## Contiguous Segment Definition

For ordered clusters `C_0, ..., C_{n-1}`, any contiguous subsequence:

`[i..j] = {C_i, C_{i+1}, ..., C_j}`

is a candidate final span.

For each such segment:

- gather all points from clusters `i` through `j`
- fit one catenary in the track's local `(u, z)` frame
- evaluate lateral consistency in `v`
- assign a scalar segment cost

## Candidate Segment Validity

A segment is considered a valid single-span candidate if:

- the catenary fit succeeds
- catenary RMSE is below threshold
- RMSE increase over the member-cluster baseline is below threshold
- RMSE ratio over the member-cluster baseline is below threshold
- lateral-track spread is below threshold
- member cluster track centers remain near one shared `v` center

If a multi-cluster segment fails these checks, it is marked invalid and cannot
appear in the final DP partition.

A singleton cluster is always allowed as a fallback segment so that every
cluster still receives an owner even if it is short or underconstrained.

## Segment Cost Function

For a valid candidate span `[i..j]`, the implementation uses a scalar cost of
the form:

`J(i, j) = lambda_0 + J_rmse + J_increase + J_gap + J_short + J_track`

where:

- `lambda_0` is a per-segment base penalty
- `J_rmse` penalizes catenary residual
- `J_increase` penalizes degradation relative to local cluster fits
- `J_gap` penalizes internal missing distance between consecutive clusters
- `J_short` penalizes very short or weakly constrained segments
- `J_track` penalizes poor lateral-track consistency

### RMSE Term

`J_rmse = w_rmse * (RMSE(i, j) / RMSE_max)`

### RMSE Increase Term

Let `RMSE_base(i, j)` be the mean local catenary RMSE of the member clusters.
Then:

`Delta_RMSE(i, j) = RMSE(i, j) - RMSE_base(i, j)`

and:

`J_increase = w_increase * max(Delta_RMSE(i, j), 0) / Delta_RMSE_max`

### Internal Gap Term

For consecutive clusters in the ordered segment, let:

`gap_k = max(0, u_min(C_{k+1}) - u_max(C_k))`

Then:

`J_gap = w_gap * (sum gap_k / gap_scale)`

This does not force a split by itself, but it discourages large stitched gaps.

### Short-Segment Term

If the total projected span is too short:

`L(i, j) = u_max(i, j) - u_min(i, j)`

then:

`J_short = w_short * max(L_min - L(i, j), 0) / L_min`

### Track Consistency Term

For projected lateral centers `v_center(C_k)` inside the segment, compute a
shared lateral center:

`v0(i, j) = median(v_center(C_k))`

Then penalize large deviations from that shared center as well as large track
spread in the segment.

## Dynamic Programming Formulation

Let `DP[t]` be the minimum total partition cost for the prefix of ordered
clusters:

`C_0, C_1, ..., C_{t-1}`

Base case:

`DP[0] = 0`

Recurrence:

`DP[t] = min over s < t { DP[s] + J(s, t-1) }`

subject to:

- the candidate segment `[s..t-1]` is valid
- the segment is contiguous by construction

The algorithm also stores backpointers so that the optimal partition can be
reconstructed after the DP table is filled.

The chosen partition is therefore:

- globally minimum-cost under the model
- contiguous
- non-overlapping
- exhaustive over the ordered cluster sequence

## Why This Solves The Ownership Problem

The critical guarantee is:

- every cluster appears in exactly one DP segment

This prevents one span from swallowing clusters that should belong to the next
span while also leaving leftovers behind.

In other words, the algorithm does not ask:

- "should I greedily absorb one more neighbor?"

Instead, it asks:

- "what is the best full partition of this entire ordered track?"

That is the correct abstraction for support-region ownership.

## Pseudocode

```text
function STITCH_WIRES(clusters):
    features = EXTRACT_CLUSTER_FEATURES(clusters)
    candidate_edges = BUILD_GRAPH_EDGES(features)
    graph_components = CONNECTED_COMPONENTS(candidate_edges)

    candidate_tracks = []
    for component in graph_components:
        candidate_tracks.extend(REFINE_TRACK_COMPONENT(component))

    final_spans = []
    for track in candidate_tracks:
        track_frame = BUILD_TRACK_FRAME(track)
        ordered_clusters = SORT_CLUSTERS_BY_U(track, track_frame)
        segment_table = PRECOMPUTE_SEGMENT_COSTS(ordered_clusters, track_frame)
        partition = SOLVE_DP_PARTITION(segment_table)
        final_spans.extend(BUILD_OUTPUT_SPANS(partition))

    return final_spans


function SOLVE_DP_PARTITION(segment_table):
    DP[0] = 0
    for t in 1..n:
        DP[t] = +inf
        for s in 0..t-1:
            if SEGMENT[s, t-1] is valid:
                DP[t] = min(DP[t], DP[s] + COST[s, t-1])
                store backpointer
    return reconstruct partition from backpointers
```

## Deterministic Behavior

The implementation is deterministic:

- pair evaluation order is fixed
- direction signs are canonicalized
- graph edges are sorted deterministically
- cluster ordering on each track is deterministic
- the DP recurrence is solved in fixed index order
- ties are broken deterministically

No random seed selection or random cluster growth is used.

## Assumptions

- extracted clusters are already approximately wire-like
- global `z` is a meaningful up direction
- each candidate track can be represented by a single local wire-aligned frame
- final spans are contiguous along the ordered track sequence
- one span is reasonably described by one catenary in the local `(u, z)` frame

## Failure Cases And Limitations

This method is stronger than greedy stitching, but it is not perfect.

Likely failure cases include:

- extremely short fragments with too little span for stable fitting
- heavy outlier contamination inside a cluster
- very noisy support-region geometry that corrupts ordering
- true conductors whose shape strongly departs from a catenary
- candidate-track grouping that is badly wrong before partitioning begins

The DP stage fixes ownership within a track. It is not a substitute for all
upstream geometric filtering.

## Tuning Guidance

Important tuning parameters in `WireStitchingParameters`:

- `max_clusters_per_span_candidate`
  - limits how many consecutive clusters are tested as one span candidate
- `max_span_catenary_rmse_m`
  - hard validity threshold for a single-span catenary fit
- `max_span_rmse_increase_m`
  - limits degradation relative to member cluster fits
- `max_span_rmse_ratio`
  - relative version of the same idea
- `max_span_track_spread_m`
  - limits lateral spread inside one final span
- `max_span_cluster_track_deviation_m`
  - limits cluster-to-cluster cross-track drift inside one span
- `span_partition_base_cost`
  - discourages over-fragmentation by charging a fixed cost per segment
- `singleton_span_extra_cost`
  - makes unnecessary singleton partitions more expensive
- `segment_gap_cost_weight`
  - discourages segments with large internal missing gaps
- `segment_short_span_cost_weight`
  - discourages underconstrained very short segments

Recommended tuning workflow:

1. inspect `wire_clusters_pre_stitch.npz`
2. inspect `wire_stitching_report.json`
3. verify candidate tracks first
4. then inspect the ordered cluster sequence on each track
5. adjust span-cost weights and thresholds before touching the graph logic

## Why This Is Better For Sag Measurement

Sag measurement needs final outputs that correspond to physical spans, not just
to conductor tracks.

This algorithm enforces:

- track identity first
- span ownership second
- exactly one owner per cluster
- contiguous final spans

That is why adjacent spans on the same conductor track can remain separate even
when they are close together and geometrically similar.

## Summary

The final wire-stitching stage is now formulated as:

1. graph-based candidate-track construction
2. local `(u, v, z)` track coordinates
3. deterministic ordering of clusters by `u`
4. precomputation of contiguous segment validity and cost
5. minimum-cost dynamic-programming partition
6. final span outputs with exclusive cluster ownership

This is the strongest practical fix for the support-region failure mode where
greedy stitching can let one span absorb clusters from the neighboring span.
