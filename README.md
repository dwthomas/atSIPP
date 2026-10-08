# atSIPP

Heuristic search algorithms for planning with arrival-time functions (ATFs)
in Safe Interval Path Planning (SIPP) state spaces.

This planner targets applications represented using the SIPP state space,
where states and actions have safe intervals.

The `grid2d` domain provides an example environment using MovingAI 2D grid
maps with randomly generated obstacles.

## Included Algorithms

### SIPP

- **SIPP** — Based on Phillips and Likhachev (2011)
- **Augmented SIPP** — SIPP with ATFs; see Thomas et al. (2026)
- **Augmented SIPPαβ** — Augmented SIPP with ↓f↑α↑β ordering;
  see Thomas et al. (2026)

### Real-Time SIPP

- **MaxATFS** — Real-time SIPP with maximal learning using an
  LSS-LRTA*-style strategy; see Thomas, Ruml, and Shimony (2024)
- **MedATFS** — Real-time SIPP with medium learning using LRTA*-style
  dynamic learning and LSS-LRTA*-style static learning; see Thomas, Ruml,
  and Shimony (2024)
- **RTAS** — Real-time SIPP with LRTA*-style learning;
  see Thomas, Ruml, and Shimony (2024)
- **PLRTS** — Real-time SIPP with static-only learning;
  see Thomas, Ruml, and Shimony (2024)

### Any-Start-Time Planning

- **RePEAT** — Optimal planning for a dynamic world;
  see Thomas et al. (2026)

## Tested Toolchain

- GCC 16.2.1
- Boost 1.92.0
- GMP 6.3.0
- Meson 1.12.1

## Building

### Using the helper scripts

```bash
./configure.sh
./build.sh
```

### Using Meson directly

```bash
meson setup --reconfigure build --clearcache \
  -Dprefer_static=true \
  -Ddefault_library=static \
  -Dc_link_args=-static \
  -Dcpp_link_args=-static

meson compile -C build
```

## References

1. Phillips, M., & Likhachev, M. (2011). SIPP: Safe interval path planning
   for dynamic environments. *2011 IEEE International Conference on Robotics
   and Automation*, 5628–5635.
   [https://doi.org/10.1109/ICRA.2011.5980306](https://doi.org/10.1109/ICRA.2011.5980306)

2. Thomas, D. W., Shimony, S. E., Ruml, W., Karpas, E., Shperberg, S. S.,
   & Coles, A. (2026). *Optimal Planning in a Dynamic World*. arXiv.
   [https://doi.org/10.48550/ARXIV.2610.03312](https://doi.org/10.48550/ARXIV.2610.03312)

3. Thomas, D. W., Ruml, W., & Shimony, S. E. (2024). Real-time safe interval
   path planning. *Proceedings of the International Symposium on Combinatorial
   Search, 17*, 161–169.
   [https://doi.org/10.1609/socs.v17i1.31554](https://doi.org/10.1609/socs.v17i1.31554)
