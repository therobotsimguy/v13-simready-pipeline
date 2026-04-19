---
name: sim-ready-datasets
description: >-
  Comprehensive catalog of datasets and benchmarks for articulated 3D objects,
  simulation-ready assets, and robot learning. Covers ArtVIP, PartNet-Mobility,
  PhysX-Mobility, BEHAVIOR-1K, Objaverse, and 10+ others. Use when selecting
  training data, comparing against benchmarks, or understanding what exists in
  the ecosystem. Includes asset counts, categories, formats, quality notes, and
  download sources. Also includes musculoskeletal benchmarks (OpenSim full-body,
  MyoSuite hand), cloth calibration sources (CLO 3D / Browzwear / Optitex), and
  sim-to-real Pearson-r metrics.
---

# Sim-Ready Datasets Skill

## When to Use
- Choosing a dataset for training or evaluation
- Understanding the landscape of available articulated object data
- Comparing your pipeline against published benchmarks
- Finding assets for a specific object category

## Dataset Comparison Matrix

### Articulated Object Datasets

| Dataset | Objects | Categories | Format | Articulated | Physics | Visual Quality | Access |
|---------|---------|-----------|--------|-------------|---------|---------------|--------|
| **ArtVIP** | 992 | 9 cat / 37 subcat | USD | Yes (2156 prismatic + 1809 revolute) | Fine-tuned dynamics | Professional (manifold mesh, PBR) | HuggingFace |
| **PartNet-Mobility** | 2,346 | 46 | URDF | Yes | Basic | Low-medium (unsmoothed, imprecise joints) | SAPIEN |
| **PhysX-Mobility** | 2,000+ | 47 | URDF/XML | Yes | Rich annotations | Real-world objects | PhysX-Anything |
| **BEHAVIOR-1K** | 5,213 | 1000+ activities | USD | 24 articulated | Better visual | Good | OmniGibson only |
| **Objaverse** | 800K+ | Diverse | GLB/OBJ | No (rigid only) | No | Variable | Open |
| **Infinite-Mobility** | 10,833 | 13 | Procedural | Yes | Generated | Synthetic | Generated |
| **ArtLLM Mixture** | 20,673 | 43 | Point cloud + URDF | Yes | Basic | Mixed | Combined |
| **Articulation-XL** | 33,000+ | Diverse (characters, creatures) | Skeleton + skinning | Yes (rigging) | No | Variable | Open-sourced |
| **PhysX3D** | 7,672 | 23 | Voxel + physical repr | Yes | Rich | Generated | PhysX-Anything |

### Key Quality Differences

| Quality Axis | ArtVIP | PartNet-Mobility | BEHAVIOR-1K |
|-------------|--------|-----------------|-------------|
| Mesh quality | Manifold, high triangle count | Unsmoothed, low poly | Medium |
| Textures | High-res PBR, UV-mapped | Often missing | Good |
| Physics tuning | Per-asset fine-tuned (0.2-2h each) | Default parameters | Not fine-tuned |
| Collision shapes | Mix of hull + decomposition, hand-tuned | Auto-generated | Basic |
| Joint parameters | Validated with motion capture | Often imprecise | Not validated |
| Behavior annotations | 5 primitives, pixel-level affordances | None | Activity-level |
| Sim platform | Isaac Sim | SAPIEN | OmniGibson |
| Sim-to-real gap | Low (Pearson r=0.99) | High | Unknown |

## ArtVIP Deep Dive (Best-in-Class)

### Statistics
- **9 categories**: furniture, kitchenware, kitchen appliances, fixtures, appliances, cleaning tools, stationery, storage, mechanical equipment
- **37 subcategories** (see breakdown below)
- **992 total objects**, **2,156 prismatic joints**, **1,809 revolute joints**
- **6 sim-ready scenes**: childrenroom, diningroom, kitchen, kitchen with parlor, large/small livingroom

### Modeling Time Per Category
| Category | Subcategory | Modeling Time | Physics Tuning | Count |
|----------|------------|--------------|---------------|-------|
| Furniture | chair | 2h | 0.3h | 23 |
| Furniture | table | 1.5h | 0.2h | 131 |
| Furniture | cabinet | 3.1h | 0.4h | 183 |
| Furniture | cupboard | 15h | 2h | 11 |
| Kitchen | fridge | 6h | 0.5h | 22 |
| Kitchen | dishwasher | 5h | 0.4h | 19 |
| Kitchen | oven | 4h | 0.3h | 11 |
| Appliances | washing machine | 5.7h | 0.5h | 30 |
| Appliances | fan | 1.8h | 0.3h | 34 |
| Fixtures | toilet | 4h | 0.4h | 14 |
| Cleaning | trash can | 2h | 0.3h | 18 |
| Stationery | scissors | 1h | 0.2h | 28 |
| Storage | toolbox | 2.5h | 0.3h | 22 |

### ArtVIP Annotation Labels (for Part Classification)
| Label | Description |
|-------|------------|
| armrest | Chair armrest |
| body | Parts needing labeling excluding base and lid |
| button | All push-button switch components |
| door | Door of cabinets, refrigerators, ovens, etc. |
| drawer | Drawer of cabinets, refrigerators, toolboxes |
| handle | Any handles |
| knob | All rotary switch components |
| lid | Cardboard box lid, electric steamer lid, trash can lid |
| pedal | Foot pedal (step-on trash can) |
| pipe | Water pipe part of faucet |
| rack | Oven/refrigerator door shelf |
| roller | Washing machine drum |
| shelf | Shelf part of cabinets, refrigerators |
| wheel | Chair wheels |

### Sim-to-Real Transfer Results (ArtVIP)
| Task | Method | Real-Only | Sim-Only | Real+Sim Mixed |
|------|--------|----------|---------|---------------|
| PullDrawer | ACT | 64% | 39% | 81% |
| OpenCabinet | ACT | 34% | 12% | 46% |
| SlideShelf | ACT | 27% | 13% | 36% |
| CloseOven | ACT | 58% | 23% | 68% |

**Key finding**: Mixing 100 real + 100 sim trajectories from ArtVIP assets nearly matches real-only performance, proving the physical fidelity.

## Benchmark Metrics

### Part Segmentation
- **mIoU**: Intersection over Union between predicted and ground-truth part meshes
- **Count Accuracy**: % of instances where inferred number of active kinematic links matches ground truth

### Joint Parameter Estimation
- **Joint Type Error**: Binary (correct type or not)
- **Joint Axis Error**: Angular deviation between predicted and ground-truth axes, normalized [0, pi]
- **Joint Pivot Error**: L2 Euclidean distance between predicted and actual pivot locations
- **Joint Range IoU**: IoU of predicted limit ranges vs ground truth

### Physical Executability
- **Definition**: Success rate of URDFs that can be fully actuated along valid ranges without catastrophic behaviors (inter-penetration, structural detachment, kinematic freezing)
- **Gold standard**: Load into SAPIEN physics simulator, actuate all joints through full range

### Geometry Quality
- **Chamfer Distance (CD)**: Symmetric distance between point clouds (lower = better)
- **CD_static**: For static base parts
- **CD_movable**: For movable parts (harder, more informative)

## Extended Benchmarks & Calibration Sources

### Sim-to-Real Empirical Numbers

| Benchmark | Sim-to-Real Metric | Value | Context |
|-----------|--------------------|------:|---------|
| **ArtVIP** | Pearson r | **0.99** | Contact-rich task success correlation; 81% ACT success sim+real vs 34% real-only |
| PartNet-Mobility | Contact accuracy | 0.5 | Not sim-to-real validated |
| RoboCasa / BEHAVIOR-1K | Task success | Variable | Scene-level, not asset-level metric |

**Rule of thumb:** choose dataset by sim-to-real gap tolerance. ArtVIP for production-validated; PartNet-Mobility for scale + category diversity (tolerate the gap).

<!-- source: bundle3/validation_report (ArtVIP citation), confidence: HIGH -->

### Musculoskeletal Benchmarks

For humanoid / medical / prosthetic assets. Not rigid-body interchange format — reference models for validation and training targets.

| Benchmark | DOF | Muscles | Backend | Use Case |
|-----------|----:|--------:|---------|----------|
| **OpenSim full-body** (Rajagopal 2016) | 29 | 80 | Simbody | Gait analysis, rehabilitation, prosthetic design |
| **MyoSuite hand** | 28 | 39 | MuJoCo | Dexterous RL, hand rehabilitation |
| **Parametric Human Project** (Autodesk) | varies | — | — | Population-varying anatomical atlas for ergonomics |

Cross-ref `simready-mechanism-lookup §Biomechanical Joint Equivalents` and `robot-model §OR Deployment`.

<!-- source: bundle3/surgical_simulation_memo + bundle4/section_file(4)/0_biomechanics_and_anatomy, confidence: HIGH -->

### Cloth / Textile Calibration Sources

Commercial textile tools ship with proprietary but validated cloth physics. Treat as ground-truth for cloth material params.

| Tool | Use Case | Access |
|------|----------|--------|
| **CLO 3D** | Fashion industry cloth sim | Commercial subscription |
| **Browzwear** | Apparel industry | Commercial |
| **Optitex** | Technical pattern fit | Commercial |

**Strategy:** reverse-engineer material params from CLO/Browzwear/Optitex outputs, or partner for dataset access. Feeds `deformable-physics-robotics §9 Sim-to-Real Gaps`.

<!-- source: bundle2/findings_file(2)_wide_research_unzipped/5_textile_and_apparel_industry_systems, confidence: MED -->

### MobilityGen (Scale-Only, Not Topology-Generative)

NVIDIA MobilityGen generates **large datasets from predefined asset templates** — useful for scale, but does NOT generate novel topology. Treat it as a data-expansion tool, not a generative system. Cross-ref `blender-3d-generation §Generative Front-End Architectures`.

<!-- source: bundle6/research_generative_methods_and_validation.csv, confidence: HIGH -->

## Reference Papers
Located at: `scripts/tools/simready_assets/reference_library/`
- ArtVIP paper: `papers_collision/02_artvip_...pdf`
- PhysX-Anything paper: `papers_articulation/10_physx_anything_...pdf`
- ArtLLM paper: `papers_articulation/05_artllm_...pdf`
