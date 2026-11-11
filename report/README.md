# Comparison deck (English)

`LOAM_vs_LIOSAM_comparison.pptx` is **generated**, not hand-edited. Its numbers are bound to
the measurement pipeline, so a slide cannot drift away from a measurement.

```bash
# after the experiments have run (results/tables/all_runs.csv exists)
python3 report/build_deck.py

# while waiting for data: renders the same deck with explicit DATA PENDING placeholders
python3 report/build_deck.py

# check the data binding without writing a file
python3 report/build_deck.py --dry-run
```

## Where each number comes from

| Slide content | Source |
|---|---|
| ATE / RPE / final error / divergence / output rate | `results/tables/all_runs.csv` (written by `evaluate.py`) |
| Scenario names, worlds, motions, mechanisms | `SCENARIOS` in `build_deck.py` (mirrors `experiments/scenarios/*.yaml`) |
| Improvement % and verdict labels | computed in `improvement()` / `verdict_for()` |
| Trajectory and error figures | `results/figures/*.png` (written by `make_figures.py` on the robot) |

`--repeats N` runs are merged per scenario (median) before they reach a slide.

## Verdict thresholds

Fixed in `verdict_for()` and printed on the attribution slide, so the conclusion is not tuned
after seeing the data:

```
|Δ| < 5%    marginal        5–25%   limited
25–60%      clear win       ≥ 60%   decisive        Δ < 0  LIO-SAM worse
```

## Two generator details worth remembering

1. **Text boxes need `auto_size = NONE`.** python-pptx text boxes default to `spAutoFit`;
   LibreOffice/PowerPoint then grow the shape and re-centre it, so text lands away from its
   declared position and appears to overflow.
2. **The bundled LibreOffice kit renders content at a fixed 1.5× magnification** into a
   1280×720 canvas, so a 13.333 in design only shows its top-left 8.889 in — previews look
   cropped even when the PPTX geometry is correct. `--preview` builds a copy whose *content*
   (positions, sizes, font sizes) is scaled by 1/1.5 while the page stays at design size; the
   renderer's 1.5× cancels it and the preview becomes faithful. Use `--preview` for any visual
   check, e.g.:

   ```bash
   python3 report/build_deck.py --preview --out report/.deck-preview-src.pptx
   # then render .deck-preview-src.pptx with the bundled LibreOffice kit
   ```

   Never ship the `--preview` file: its fonts are 2/3 size by construction.

## Slide order

1 title · 2 executive summary · 3 problem · 4 the two algorithms · 5 fairness protocol ·
6 measurement chain · 7 experiment matrix · 8 metrics · 9 headline table · 10 trajectories ·
11 error vs time · 12 ATE across conditions · **13 when is LIO-SAM more accurate** ·
14 mechanism: distortion · 15 mechanism: degeneracy · 16 cost · 17 threats to validity ·
18 conclusion

Every slide carries speaker notes; export them with PowerPoint's notes view if the deck is used
as a talk script.
