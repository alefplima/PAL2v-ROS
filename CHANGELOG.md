# Changelog

## 1.1.0 (2026-11-25)

**PAL2vFusion without exclusion of information sources.** In the PAL2v, a non-classical logic, a contradictory
source is not discarded. The new default `mode="graded"` keeps every sensor in the network and attenuates its
evidence by its tolerance interval:

- temporal analysis by a paraconsistent artificial neural cell of learning (CNAPap / lPANCell), one iteration
  per sample, against the learned pattern (previous reading);
- spatial analysis by a PAN of each sensor against the ParaExtrctr of the others;
- tolerance interval φ = 1 inside the contradiction limit (as the Ftct factor of the CNAPp), falling linearly to 0
  at `band` × limit (default 1.5, i.e. full attenuation at 6σ with k = 4); φᵢ = min(φ temporal, φ spatial)
  (AND connective);
- each sensor enters the final ParaExtrctr with μ̃ᵢ = μE₋ᵢ + φᵢ (μᵢ − μE₋ᵢ), from the most to the least
  contradictory one (Gauss-Seidel order);
- with only two effective sources in contradiction the logical output is indefiniteness (certainty 0); holding
  the last defined value is now documented as a decision of the application layer (`hold_on_indefinite`).

`mode="step"` reproduces v1.0.0 exactly. It is the limit case band → 1 (unit test `test_step_is_limit_case`).

Other changes:
- `FusionOutput.weights` (φᵢ of each sensor); `rejected` now means "outside the tolerance" (attenuated in graded mode).
- `pal2v_fusion` node: parameters `mode` (default `graded`) and `band`.
- Experiments: new method `pal2v_g` in E2–E5 (the v1.0.0 method is kept as `pal2v`, step); E3 selects the
  mode with `LPA2V_FUSION_MODE`; 6th seed in E5 for every method; E1 benchmarks both modes.
- 32 unit tests (6 new).

## 1.0.0 (2026-11-24)

First public release. DOI [10.5281/zenodo.24499334](https://doi.org/10.5281/zenodo.24499334).
