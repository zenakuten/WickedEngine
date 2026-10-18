# WickedEngine Box3D backend design

## Goal and decisions

Add a build-time choice between Jolt and Box3D without changing the default
physics backend or the public meaning of existing scene components.

- Exactly one backend is compiled and linked into a WickedEngine build.
- Jolt remains the default and retains its existing behavior.
- A Box3D build contains no Jolt simulation, fallback world, or Jolt dependency.
- The selection applies to every scene in that engine build.
- Runtime switching, per-body selection, and mixed-backend scenes are out of scope.
- SDL3 remains an independent platform-layer choice, not a physics requirement.
- Unsupported features are reported explicitly, never silently approximated or
  passed to the other backend.

The initial Linux/CMake rigid-body backend is now implemented. The execution
status below distinguishes verified behavior from the remaining plan. Windows,
native project selectors, packaging, interactive acceptance and advanced
features still need work. Selecting Box3D does not imply Jolt feature parity.

## Execution status

| Stage | Current state |
|---|---|
| 1. Selector and dependency | Implemented and verified on Linux/CMake; pinned MIT-licensed core is vendored |
| 2. World and primitives | Implemented; headless behavior, serialized access and deferred teardown tests pass |
| 3. Usable rigid-body backend | Implemented for the supported subset; headless coverage passes, interactive Editor acceptance remains |
| 4. Advanced rigid-body features | Deferred; characters, ragdolls, vehicles, soft bodies and buoyancy are explicitly unavailable |
| 5. Platform and packaging | Linux static/shared builds and GCC/Clang covered; Windows/Apple/native selectors and packaged consumers remain |

### Delivered implementation

`WICKED_PHYSICS_BACKEND` accepts exactly `Jolt` or `Box3D`, with Jolt as the
default. CMake adds only the selected adapter and dependency, including the
external/embedded-shader engine target wiring. Reconfiguring one build tree from
Box3D to Jolt and back removes the old dependency from the generated target set;
invalid selections fail immediately. Compile inputs and executable link commands
were inspected in addition to building successfully.

The Box3D adapter is in `WickedEngine/wiPhysics_Box3D.cpp`. It implements world
ownership, synchronous per-world access gates, deferred body/joint releases,
paused cleanup, fixed stepping, interpolation, rigid-body synchronization,
geometry cooking, common constraints, queries, picking and debug callbacks.
World creation/destruction is globally serialized. Box3D uses its built-in
scheduler rather than callbacks that wait on nested WickedEngine jobs.

The engine capability API reports the backend name and unsupported features.
Editor component controls, constraint/shape selection, ragdoll controls,
soft-body painting, the cloth sample and Lua activation paths are gated.
Authored unsupported data is retained; it is not converted, erased or sent to
a second backend. Lua exposes `physics.GetBackendName()`.

### Current restrictions

- Mesh and heightfield bodies must be static. Dynamic convex hulls are supported.
- Boxes use convex hulls; cylinders use documented 32-sided hull geometry.
- Coplanar convex hulls use a thin, symmetric volumetric hull so static,
  kinematic and dynamic bodies can retain their authored motion type. Total
  collision thickness is normally 0.002 engine units; exceptionally large
  geometry uses a scale-aware floating-point minimum. This compatibility
  approximation is logged once per geometry configuration. Source vertices
  and authored scene data are not changed. Collinear/point hulls remain invalid.
- Mirrored or sheared collision geometry is rejected instead of guessed.
- Fixed, point, distance, hinge, slider and circular cone configurations are
  supported. Cone half angles above pi/2 are rejected.
- SwingTwist accepts only the exactly representable circular configuration,
  with equal swing half angles. General SixDOF is unavailable.
- Characters, ragdolls, vehicle controllers, soft bodies and buoyancy have no
  Box3D implementation. Existing Jolt support remains available in Jolt builds.
- Access serialization protects native worlds, not arbitrary concurrent ECS
  mutation. Callers must keep component references alive and coordinate changes
  to component storage.
- Stable body/joint records are retained until native world destruction to keep
  deferred releases and query user data safe. Long-running creation/deletion
  workloads still need memory-growth measurements and possible record reclamation.
- Hand-maintained Visual Studio and Xcode projects remain Jolt-only.
  CMake-generated Windows/Apple builds have not been validated.
- Relocatable installation/exported-package support is not claimed. The existing
  package configuration's SDL2 dependency and selected-backend consumer linkage
  still require stage 5 work.

### Verification performed

All configurations below use SDL3; the existing SDL2-only ImGui examples are
skipped by the engine's SDL3 configuration.

| Build directory | Configuration | Result |
|---|---|---|
| `build` | Default Jolt, GCC, Release, static | All enabled applications build; both CTests pass |
| `build-box3d` | Box3D, GCC, Release, static | All enabled applications build; both CTests pass |
| `build-box3d-shared` | Box3D, GCC, Release, shared | All enabled applications build; both CTests pass |
| `build-box3d-clang` | Box3D, Clang, Release | Headless physics contract builds and passes |
| `build-box3d-asan` | Box3D, GCC, Debug, ASan + UBSan | Contract passes five consecutive runs |
| `build-box3d-tsan` | Box3D, GCC, RelWithDebInfo, TSan | Contract passes ten consecutive runs without reported races |

Both the adapter's C++ and the vendored Box3D C core are instrumented in the
sanitizer builds. A clean sanitizer run is evidence for these scenarios, not a
proof that every engine/editor path is race-free.

`Samples/Tests/PhysicsContractTest.cpp` covers gravity/floor settling, mass-scaled
impulses, force/torque, teleports/reset, ray bounds/normals, ghosts, primitives,
convex/mesh/heightfield support, joint registration/restraint and broken-state
repair, offsets/parenting, parameter refresh, interpolation bounds, 2D locks,
kinematic transitions, picking/deleted picks and Lua backend identity.
Box3D-specific cases cover unsupported-data preservation and correction/retry,
dynamic-mesh rejection and static recovery, recooking edited mesh data, live
scene merging, concurrent set/get calls, 128 deferred body releases while
stepping, and parallel creation/stepping/destruction of 48 worlds across four
caller threads. Paused deletion and fixture teardown are also exercised.

The actual `Content/models/physics_test.wiscene` asset is also checked using
the optional Linux/Vulkan mode:

```sh
build-box3d/Samples/Tests/PhysicsContractTest \
  --sample-scene Content/models/physics_test.wiscene
```

This initializes SDL/Vulkan for asset deserialization but does not open a window.
The same check passes on Jolt and Box3D: all 15 rigid bodies, including the
kinematic planar floor, register; 14 dynamic bodies simulate for ten seconds,
with no penetration in the interior of the finite floor. Bodies that bounce off
its edges are not incorrectly treated as floor penetration. The headless suite
also checks coplanar floor queries from both sides, box/sphere/capsule/cylinder
support, a three-point dynamic planar hull and a rising kinematic planar
platform. The moving-platform assertion for the dynamic planar hull is
Box3D-specific because its compatibility hull has volume, unlike Jolt's
zero-volume representation.

The headless ASan/UBSan and TSan suites pass. The optional GPU-backed asset
check completes its physics checks under ASan/UBSan, but LeakSanitizer reports a
256-byte allocation from an unloaded module at process shutdown. A probe that
opens no scene and performs no simulation reproduces that allocation, so
GPU/SDL-path sanitizer cleanliness is not claimed. No leak suppressions were
added.

Editor binaries and UI changes compile, but no graphical Editor acceptance run,
performance comparison, allocator-failure injection or complete joint
limit/motor/CCD matrix has been performed. These remain acceptance work rather
than implied successes.

### Correctness fixes discovered during execution

The contract exposed Jolt's inverted `IsConstraintBroken()` result; it now
matches `SetConstraintBroken()` and the public API. Shared builds required fixing
the PIC option default to use the value of `WICKED_DYNAMIC_LIBRARY`, not its
literal variable name.

ThreadSanitizer exposed a real adapter flag read outside the world gate; that
read now shares the gate with parameter-refresh writes. It also exposed missing
jobsystem publication: relaxed completion-counter decrements/reads did not
establish a happens-before relationship between job writes and `Wait()`.
Completion now uses acquire/release semantics, and the shutdown wake-loop flag
is atomic. These barriers are necessary for safe transform/physics phase
ordering, independently of the native simulation lock.

Unsupported-feature diagnostics exercised the existing asynchronous logger's
synchronous flush and static-local shutdown races. File writes/flushes/path
changes now share a stream mutex, and queue buffers have lifetimes bounded by
the writer/flush operation. No TSan suppressions were added.

Interactive testing exposed a gap in the original synthetic coverage:
`physics_test.wiscene` uses a coplanar **kinematic** convex hull for its floor.
Box3D rejected that geometry, so all rigid bodies fell through an unregistered
floor. Its mesh collision is static-only, so converting the floor to a triangle
mesh would also lose the authored motion behavior. The adapter now cooks a thin
volumetric hull and explicitly reports its numerical thickness. The sample
selector labels this backend's Physics Test as rigid-body-only; its soft-body
torus is intentionally not simulated, and the Cloth Physics Test remains hidden.

The new asset check also exposed missing Vulkan fallback-resource teardown:
the device destructor freed its null buffer but not its buffer view or three
null images/seven image views. Those resources are now destroyed before the
Vulkan memory allocator, avoiding the Debug allocator's live-allocation
assertion. The asset runner advances deferred GPU-page releases before closing
the device; this does not change the headless CTest path.

### Reproducing the main builds

From the WickedEngine project root:

```sh
cmake -S . -B build-jolt -DWICKED_USE_SDL3=ON -DWICKED_PHYSICS_BACKEND=Jolt
cmake --build build-jolt --parallel
ctest --test-dir build-jolt --output-on-failure

cmake -S . -B build-box3d -DWICKED_USE_SDL3=ON -DWICKED_PHYSICS_BACKEND=Box3D
cmake --build build-box3d --parallel
ctest --test-dir build-box3d --output-on-failure
```

For shared builds, add `-DWICKED_DYNAMIC_LIBRARY=ON`. For ASan/UBSan use a Debug
build with `-DWICKED_ENABLE_ASAN=ON -DWICKED_ENABLE_UBSAN=ON`. For TSan use a
separate build with `-DWICKED_ENABLE_TSAN=ON`; add
`-DWICKED_ENABLE_SAN_ALWAYS=ON` when using a non-Debug configuration. Build
`PhysicsContractTest` and select it with `ctest -R '^PhysicsContractTest$'`
for headless-only validation. Do not combine TSan with ASan.

The sections below retain the comprehensive design and its remaining acceptance
criteria; future-tense descriptions do not override this execution status.

## Source baseline

The assessment uses these downloaded revisions:

| Project | Revision |
|---|---|
| WickedEngine | `24e4e57f3a7823105a2302939b85c0499c6d0095` |
| Box3D | `a2e4816b4359b32408d5623784a7fe2d7259d3d6` |

Relevant integration points:

| Location | Existing responsibility |
|---|---|
| `WickedEngine/wiPhysics.h` | Engine-facing physics API, queries, picking, character and vehicle operations |
| `WickedEngine/wiPhysics_Jolt.cpp` | 4,205-line backend, world lifetime, shapes, stepping, constraints and advanced features |
| `WickedEngine/wiScene.h:80` | Opaque `Scene::physics_scene` ownership |
| `WickedEngine/wiScene_Components.h:404` | Rigid-body settings and opaque runtime state |
| `WickedEngine/wiScene_Components.h:551` | Constraint settings and opaque runtime state |
| `WickedEngine/wiScene_Components.h:1271` | Soft-body component |
| `WickedEngine/wiScene.cpp:291` | Physics update in the scene update pipeline |
| `WickedEngine/wiScene.cpp:426` | Vehicle wheel transform override |
| `WickedEngine/wiPhysics_BindLua.cpp` | Lua-facing physics API |
| `Editor/RigidBodyWindow.cpp`, `Editor/ConstraintWindow.cpp` | Shape and constraint controls |
| `Editor/SoftBodyWindow.cpp`, `Editor/PaintToolWindow.cpp` | Soft-body editing surfaces |
| `WickedEngine/CMakeLists.txt` | Backend source discovery, Jolt setup, dependency linkage and shader variants |
| `WickedEngine/WickedEngine_Windows.vcxproj` | Native Windows engine build |
| `WickedEngine/WickedEngine.xcodeproj/project.pbxproj` | Native Apple engine build |

The scene and public physics API already hide most backend types behind opaque
ownership. Preserve that boundary rather than introducing a new runtime virtual
backend hierarchy.

## Build selection

### Configuration contract

Introduce one top-level CMake cache string:

```cmake
set(WICKED_PHYSICS_BACKEND "Jolt" CACHE STRING "Physics backend")
set_property(CACHE WICKED_PHYSICS_BACKEND PROPERTY STRINGS Jolt Box3D)
if(NOT WICKED_PHYSICS_BACKEND STREQUAL "Jolt"
   AND NOT WICKED_PHYSICS_BACKEND STREQUAL "Box3D")
    message(FATAL_ERROR "WICKED_PHYSICS_BACKEND must be Jolt or Box3D")
endif()
```

Values are case-sensitive. There is no `Both`, `Auto`, or implicit fallback.
Print the selected backend and dependency revision during configuration and
engine initialization.

Use separate build directories to avoid confusing cached options, stale
libraries, and applications linked to the wrong engine:

```sh
cmake -S . -B build-jolt \
  -DWICKED_PHYSICS_BACKEND=Jolt -DWICKED_USE_SDL3=ON
cmake --build build-jolt --parallel

cmake -S . -B build-box3d \
  -DWICKED_PHYSICS_BACKEND=Box3D -DWICKED_USE_SDL3=ON
cmake --build build-box3d --parallel
```

These commands describe the proposed interface and will work only after it is
implemented.

### Source and dependency wiring

In `WickedEngine/CMakeLists.txt`:

1. Remove both backend `.cpp` paths from the globbed source list, then append
   exactly the selected implementation. Merely adding a second file to the
   current glob would compile duplicate `wi::physics` symbols.
2. Execute the existing Jolt include, target configuration, warning workaround,
   and `JPH_DEBUG_RENDERER` definition only in the Jolt branch.
3. In the Box3D branch, add the pinned dependency and select
   `wiPhysics_Box3D.cpp`.
4. Replace the unconditional `Jolt` entry in
   `WICKEDENGINE_STATIC_LIBRARIES` with the selected dependency target.
5. Keep the selected backend out of unity builds.
6. Define exactly one backend identification macro on the engine target, such as
   `WICKED_PHYSICS_JOLT=1` or `WICKED_PHYSICS_BOX3D=1`.
7. Verify propagation through `WickedEngine_common`, both shader library
   variants, `offlineshadercompiler`, Editor, Tests and templates.

Keep general engine SIMD options separate from Jolt configuration. The current
`USE_AVX`, `USE_F16C`, and related options also configure DirectXMath and must not
disappear just because Jolt is disabled. Box3D owns its own SIMD dispatch settings.

### Dependency ownership

Vendor the pinned Box3D source as `WickedEngine/Box3D`, consistent with the
existing in-tree Jolt dependency. Include its license and record the complete
upstream revision. Do not make production builds depend on the sibling checkout
at `/data/dev/wt/box3d`, a moving branch, or an automatic network download.

Add Box3D as a subdirectory of the engine build. Its top-level-only samples,
benchmarks and tests should not be built as part of WickedEngine. Maintain
Box3D's own standalone test coverage separately.

Before integration, account for these build details:

- Box3D requires C17 and its root build requires CMake 3.22, while WickedEngine
  currently declares CMake 3.19. Retain 3.19 for Jolt and reject a Box3D selection
  on older CMake with an actionable error.
- Audit directory-wide Box3D settings and output-directory changes for leakage.
  Do not globally overwrite WickedEngine compiler flags or `BUILD_SHARED_LIBS`.
- Honor `WICKED_PIC` for the Box3D library when building a shared engine.
- Align MSVC runtime selection. Box3D currently selects the static runtime;
  mixing `/MT` and `/MD` across allocating/freeing modules is unacceptable.
- Propagate `BOX3D_DOUBLE_PRECISION` consistently to library and consumers if
  enabled. Keep the initial integration on Box3D's default precision.
- Avoid importing engine-wide fast-math or contraction settings into Box3D;
  preserve its deterministic floating-point requirements.
- Verify static link order and transitive platform dependencies, including
  `libm` and any threading requirements on supported platforms.
- Confirm how dependency install rules interact with WickedEngine packaging.
  Do not advertise a relocatable engine package until a separate consumer can
  link it successfully.

### Other build systems and packaging

CMake is the initial authoritative selector, including generated Visual Studio
projects. Existing hand-maintained Visual Studio and Xcode builds remain
explicitly Jolt-only until their corresponding selector is implemented.

For native projects, add the same mutually exclusive source/dependency choice,
with Jolt as the default. Reject invalid selection rather than compiling both.
Audit any broad source inclusion rules as well as project references and
preprocessor definitions.

Where an installed CMake package is produced, export backend identity and the
selected dependency requirements. Audit `WickedEngineConfig.cmake.in`: it
currently requests SDL2, so an SDL3 package must not reintroduce that dependency.
This is a packaging requirement, not a reason to change platform behavior in
the core physics adapter.

## API and scene compatibility

Implement existing `wi::physics` entry points in `wiPhysics_Box3D.cpp`. Keep
Box3D headers and handles out of serialized scene components and public headers.
Use narrowly scoped conversion helpers for vectors, quaternions and transforms.

Add small backend-independent capability queries to `wiPhysics.h`:

- Active backend name.
- Support for each collision-shape and constraint type.
- Support for characters, ragdolls, vehicles, soft bodies and buoyancy.
- A reason when a requested configuration is unsupported.

Capabilities describe completed, validated adapter functionality, not everything
the underlying library might eventually support. Use these queries in Editor,
Lua validation, scene registration and sample selection.

Preserve serialized shape/constraint enum values and all existing properties.
Scenes saved in either build remain readable in the other, but readability does
not imply identical dynamics or support for every feature. Do not automatically
delete, rewrite, or downgrade unsupported components when loading a scene.

For unsupported scene features, keep the authored component intact, leave its
runtime physics state unregistered, and issue a clear entity-specific diagnostic.
Deduplicate repeated reports until the configuration changes. The editor should
show why simulation is unavailable; existing assets must not lose information.

Lua requests for unsupported operations should use the existing Lua error
mechanism. C++ entry points with no error return must use repository-standard
error reporting and leave state unchanged; an unsupported request must not
produce fabricated positions, successful shape changes, or valid drag handles.

## Runtime ownership

Use three principal internal records:

| Record | Responsibility |
|---|---|
| Physics scene | `b3WorldId`, access gate, fixed-step accumulator, pending teardown, cooked geometry ownership and backend lifecycle |
| Rigid-body state | Scene reference, `b3BodyId`, entity identity, owned shape references, interpolation and initial pose, local/parent transform conversion |
| Constraint state | Scene reference, `b3JointId`, body-generation references, authored parameters and broken-state bookkeeping |

These are backend-private structures, not new public component types.

Box3D IDs are generational handles, not pointers. Validate IDs before use and
reject stale commands after destruction or body recreation. A recycled numeric
slot must never cause an old constraint or impulse to affect a new body.

Use stable allocations for body user data. Do not store pointers into ECS arrays
or movable containers. Keep the record alive through pending destruction so
raycasts and contact processing cannot observe freed user data.

### Lifetime and destruction

Existing component `physicsobject.reset()` calls occur throughout the editor.
Their destructors must remain safe without assuming that the caller is the
physics update thread.

- Destructors enqueue a teardown record rather than calling Box3D immediately.
- Teardown records contain owned handles/data, not dangling wrapper pointers.
- At a safe boundary, remove dependent joints and drag anchors before bodies.
- Remove bodies before releasing mesh and heightfield data that their shapes
  reference.
- Keep each scene alive while backend wrappers or drag operations reference it.
- Drain pending teardown during normal updates, paused/disabled updates, scene
  clearing, and final world shutdown. Cleanup must not depend on a positive `dt`.
- Stop queries and commands before destroying a world; coordinate teardown with
  other users of the scene access gate.
- Guard Box3D world creation/destruction globally, as its documentation requires
  for concurrent multiple-world management.

Avoid a scene/teardown ownership cycle. Pending teardown may own detached state,
but must not retain a strong reference back to the scene that owns the queue.

## Threading model

The risk is API concurrency compatibility, not demonstrated simulation speed.
Both engines support internally multithreaded simulation.

Box3D's contract is stricter:

- No external reads or writes during `b3World_Step()`.
- No concurrent writes to one world.
- Read-only queries may run in parallel outside the simulation step.

WickedEngine currently creates bodies from parallel jobs and uses Jolt's locking
body interface. A mechanical replacement of those calls is unsafe.

### Initial safe implementation

Start with a per-world mutex that serializes all Box3D API access. Hold it across
the entire step. Read operations must also acquire it. This is intentionally
conservative and can later become a reader/writer gate for proven read-only
query batches.

Parallel scene jobs may prepare immutable geometry and mutation descriptions,
but only the serialized commit phase mutates the world. Commit creation and
recreation in a stable entity order, with bodies before constraints.

Existing public force, velocity and teleport operations should acquire the gate
and execute synchronously outside a step, blocking if another thread is
simulating. Do not silently defer their effects to a later frame. This preserves
read-after-write behavior for callers. Destruction is the explicit deferred
exception because it can originate from ownership release on arbitrary threads.

Do not hold the world gate while waiting for scene jobs that might call physics.
Do not nest world gates. Perform parallel preparation and its waits before
acquiring the gate for commit or stepping.

### Task scheduling and callbacks

Initially use Box3D's built-in scheduler with a bounded worker count selected
from the engine's available CPU budget. Account for multiple scenes and existing
engine workers to avoid oversubscription.

Only integrate `wi::jobsystem` callbacks after verifying the fork/join contract.
Box3D warns against calling `b3World_Step()` inside a non-yielding job that waits
on its own sub-jobs. A blocking callback must not exhaust all workers.

Allocator, assertion and logging callbacks must be thread-safe. Callbacks invoked
inside the step must not reenter public physics operations or try to acquire the
same world gate. Buffer gameplay events and process them after stepping.

Off-world cooking also needs an explicit audit: do not assume that every
geometry builder is thread-safe just because it does not take a world ID.

## Frame pipeline and synchronization

Preserve the current scene update ordering and physics settings:

1. Finish prerequisite scene/hierarchy jobs.
2. Prepare body and shape changes; do not mutate Box3D from preparation jobs.
3. Acquire the world gate and drain teardown.
4. Create/recreate bodies, refresh properties, then create/refresh constraints.
5. Synchronize authored static and kinematic transforms.
6. Apply scene gravity and supported environmental forces.
7. Accumulate elapsed time and perform bounded fixed steps.
8. Consume transient Box3D movement/contact/joint events before the next step.
9. Publish simulated transforms and interpolation state.
10. Release the gate before scheduling dependent scene work.

Preserve the distinction between simulation disabled and physics disabled.
Paused simulation may still need parameter updates and cleanup. Skip stepping
for invalid or nonpositive elapsed time, but do not skip necessary teardown.

Do not equate `SetAccuracy()` with Box3D substep count. The existing setting limits
fixed-step catch-up; Box3D solver substeps are a separate parameter. Document and
validate any chosen substep policy without changing the public setting's meaning.
Use the actual configured frame rate; the header comment and current Jolt default
are not identical.

Preserve previous/current poses for interpolation, including sleeping bodies,
teleports, resets and body recreation. Kinematic movement needs target transforms
and velocities rather than unconditional teleportation.

## Body, shape and material mapping

| WickedEngine feature | Box3D mapping / condition |
|---|---|
| Static, dynamic, kinematic | Corresponding body types; preserve mass-zero precedence |
| Mass and inertia | Compute shape-derived inertia and scale consistently to authored mass |
| Linear/angular damping | Body damping; validate numerical differences |
| Sleep and start inactive | Sleep/awake flags with transition coverage |
| Locked 2D | Motion locks matching the current plane convention |
| Box | Box hull |
| Sphere / capsule | Native shape geometry |
| Cylinder | Polygonal cylinder hull; document approximation and resolution |
| Convex hull | Cooked hull from mesh vertices |
| Triangle mesh | Cooked static mesh with authored LOD and correct winding |
| Heightfield | Cooked static heightfield with correct row/column orientation |
| Friction / restitution | Shape material settings; compare mixing behavior |
| Ghost mode | Collision filtering, not automatically sensor conversion |
| CCD | Explicit policy aligned with existing motion-quality expectations |

Reject nonstatic mesh/heightfield configurations where Box3D does not support
their requested collision behavior. Do not replace them with a convex hull
without an explicit future authoring option.

Respect `local_offset`, parent transforms, nonuniform scale and center-of-mass
differences in both synchronization directions. Verify capsule height conventions
and quaternion component order rather than copying fields by assumption.

Mesh and heightfield shapes retain references to cooked data. Hold those assets
until the final shape referencing them is destroyed. Prefer a scene-local cache;
include geometry revision, LOD, shape kind and relevant scale/cooking parameters
in its key. Raw vertex-buffer addresses alone do not identify immutable content.

`CreateRigidBodyShape()` is currently usable for precomputation on a worker.
Preserve that role as geometry preparation without creating a Box3D body, then
attach the result during the commit phase.

## Constraints and picking

| WickedEngine type | Planned mapping |
|---|---|
| Fixed | Weld |
| Point | Spherical with angular limits disabled |
| Distance | Distance joint |
| Hinge | Revolute, including limits and motor |
| Slider | Prismatic, including limits and motor |
| Cone | Spherical cone limit |
| SwingTwist | Only exact supported configurations; no automatic circularization of unequal swing limits |
| SixDOF | Unsupported initially; requires separate design |

Convert authored world-space frames to Box3D local joint frames. WickedEngine's
axis conventions do not automatically match Box3D's frame axes. Validate world
attachments using a dedicated static anchor where necessary.

Preserve collision suppression, parameter refresh and broken-state semantics.
WickedEngine's `break_distance` is a distance criterion; Box3D force/torque event
thresholds are not equivalent replacements. Retain adapter-side distance tracking.

Use closest-hit queries with existing ray bounds and filtering. Preserve entity,
world position, local position and normal output. Audit `position_local` carefully:
the editor feeds it back into `ApplyImpulseAt()`, and the current backend derives
it from the center-of-mass transform.

`RayIntersectionResult::physicsobject` must refer only to backend-private stable
state, never an address manufactured from a Box3D ID. Document result lifetime;
validate the handle again before dragging. Keep soft-body triangle and ragdoll
bone fields invalid when those features are unavailable.

Implement rigid-body picking with a noncolliding kinematic anchor and weld or
spherical joint. Handle deleted target bodies, scene resets, break distance and
destruction of the drag operation through the same safe teardown path.

## Advanced features and capability policy

### Characters

Box3D includes mover queries and both geometric and rigid-body character samples.
Choose a controller model only after comparing it with WickedEngine's current
rigid-body character behavior, including pushes and moving-platform interaction.

Required coverage includes ground-state classification, support normal/velocity,
slope limits, gravity factor, jump control, capsule shape changes and collision
tolerance. Publish character support only when the whole public contract works.

### Ragdolls

Use Box3D ragdoll samples as reference, not as a drop-in WickedEngine skeleton
adapter. Port bone mapping, rest-basis conversion, scale recreation, animation
to physics transitions, collision groups, ghost mode and bone-specific impulses.
Unequal swing limits remain a compatibility constraint.

### Vehicles

Box3D wheel joints and its driving sample provide building blocks. They do not
replace Jolt's transmission, differential, collision testers, suspension tuning,
motorcycle balance logic or wheel animation integration.

Treat vehicle support as a separate milestone. Cars and motorcycles may have
different capability status; do not expose a broad vehicle capability if only
one subtype is implemented.

### Soft bodies

No equivalent soft-body API exists in the inspected Box3D public interface.
Box3D builds therefore do not simulate soft-body components initially. Keep
their authored data, disable unsupported creation/editing operations, and report
unsupported scene content.

A future soft-body solver would need its own design for two-way rigid-body
interaction. It cannot be described as Jolt fallback inside a Box3D build.

### Buoyancy and environmental behavior

The Jolt adapter applies buoyancy to rigid bodies and ragdolls. No direct
equivalent buoyancy API was found in Box3D's inspected public headers.

Either implement and validate an explicit submerged-volume/drag model, or report
buoyancy as unsupported. Do not silently ignore authored buoyancy. Wind forces
also require behavior comparison; a Box3D aerodynamic helper is not automatically
equivalent to the existing engine force application.

## Editor, Lua and diagnostics

- Show the active backend in physics diagnostics and the editor.
- Disable unsupported creation choices and explain the limitation in tooltips.
- Keep an already loaded unsupported selection visible instead of replacing it.
- Gate character, vehicle, ragdoll and soft-body panels by validated capability.
- Preserve all Lua names, but report unavailable operations through Lua errors.
- Identify errors by backend, scene/entity, requested feature and invalid setting.
- Report cooking failures, allocation failures and invalid handles explicitly.
- Distinguish ordinary misses/deleted objects from integration failures.
- Do not leave refresh flags consumed as if failed registration succeeded.
- Support retry after the user corrects a configuration without log flooding.

Use Box3D debug-draw callbacks to drive existing WickedEngine debug geometry.
Keep debug distance, constraint size and visualization toggles meaningful.
Expose backend timing/counters without claiming direct equivalence between
different engines' profiling categories.

## Implementation milestones

| Stage | Deliverables | Exit criteria |
|---|---|---|
| 1. Selector and dependency | Conditional source/link wiring, pin/license, backend identity | Default Jolt unchanged; Box3D selection builds without Jolt |
| 2. World and primitives | Lifetime, access gate, fixed steps, primitive bodies, basic forces and poses | Falling/stacking scenes work; teardown safe while paused |
| 3. Usable rigid-body backend | Cooking, terrain, common constraints, picking, interpolation, material refresh and capabilities | Editor workflows and compatibility tests pass |
| 4. Advanced rigid-body features | Characters, ragdolls, environmental behavior, then vehicles | Each feature independently meets its capability contract |
| 5. Platform and packaging coverage | Native build selectors, shared builds and consumer linkage | Supported build matrix and packaged consumer checks pass |

Stages 1-3 are the first usable release boundary. Later stages must not block
preserving a fully working Jolt default.

For one experienced engine developer, initial Linux/CMake planning ranges are
3-5 working days for a primitive prototype, 2-4 weeks total for stages 1-3, and
6-12+ weeks total for broader character/ragdoll/vehicle coverage. These are rough
source-based estimates, not commitments. Full SixDOF and soft-body parity require
separate estimates; native platform and packaging work adds effort.

## Verification and acceptance

### Build matrix

- Default configuration and explicit `Jolt` both use only Jolt.
- Explicit `Box3D` uses only Box3D; invalid values fail configuration.
- Inspect compile commands/link inputs, not just a successful executable.
- Build Editor, Tests, templates and offline shader compiler with each backend.
- Exercise external and embedded shader variants where supported.
- Cover Debug/Release and static/shared engine configurations.
- Cover Linux GCC/Clang, Windows MSVC and later Apple/native project paths.
- Verify SDL3 remains selected independently in both physics builds.
- Build a separate consumer against any claimed installed engine package.

### Behavioral coverage

Create a headless backend contract suite using the public `wi::physics` API.
Run it against both separately built engines. Compare toleranced invariants,
not bit-identical trajectories across different solvers.

Required cases:

- Gravity, mass/inertia, forces, impulses, torque and local/global force points.
- Static/dynamic/kinematic transitions, sleep/wake, ghost mode and CCD.
- Parenting, local offsets, rotation, scaling, teleport and reset.
- Fixed-step catch-up, interpolation and paused/disabled/nonpositive-dt cleanup.
- Static meshes and heightfields, winding, LOD changes and cache invalidation.
- Constraint frames, limits, motors, refresh, deletion and break distance.
- Ray bounds/filtering, local hit coordinates and pick-drag lifecycle.
- Unsupported scene loading and C++/Lua/editor capability behavior.
- Body destruction from another thread while stepping or querying.
- Same-world concurrent callers, stale IDs, scene clear and pending teardown.
- Multiple scenes/worlds and scheduler pressure without deadlock.

Add ASan/UBSan coverage and targeted TSan scenarios where supported. Validate
allocator failure and cooking failure paths, not just successful creation.

Advanced-feature acceptance adds grounded movement and moving platforms,
ragdoll transitions and bone hits, wheel animation and driving, and environmental
behavior. Until those cases pass, the corresponding capability remains false.

### Performance

Benchmark equivalent authored scenes with equivalent collision geometry, fixed
time step, quality targets and CPU budgets. Record setup/cooking time, mutation
commit time, step time, synchronization time and total frame contribution.

Include piles, many sleeping bodies, mesh/terrain-heavy scenes, frequent
creation/deletion and single-/multi-worker runs. Report median and tail timings,
memory and correctness differences. Do not claim one backend is faster based
solely on its threading API or one favorable sample.

### Completion definition

The first usable Box3D release is complete when stages 1-3 pass their build and
behavioral gates, Jolt remains the unchanged default, unsupported features are
visible and non-destructive, and every Box3D build contains only its selected
physics dependency. Full parity is explicitly not the first release criterion.

## Primary source references

Paths below are relative to the corresponding project root; line references
describe the pinned baseline and may shift during implementation.

- WickedEngine: `WickedEngine/CMakeLists.txt:80-90, 93-100, 175-179, 297-305`.
- WickedEngine: `WickedEngine/wiPhysics_Jolt.cpp:265-420` (ownership/destruction).
- WickedEngine: `WickedEngine/wiPhysics_Jolt.cpp:493-685` (body/shape setup).
- WickedEngine: `WickedEngine/wiPhysics_Jolt.cpp:1955-2105` (shape mapping).
- WickedEngine: `WickedEngine/wiPhysics_Jolt.cpp:2134-2380` (parallel registration).
- WickedEngine: `WickedEngine/wiPhysics_Jolt.cpp:2856-2943` (fixed-step loop).
- WickedEngine: `WickedEngine/wiPhysics_Jolt.cpp:4003-4205` (query/picking).
- Box3D: `include/box3d/types.h:616-679, 890-940` (joint types and spherical limits).
- Box3D: `include/box3d/box3d.h:842-878` (shape lifetime/static restrictions).
- Box3D: `docs/foundation.md:142-228` (scheduler and external access contract).
- Box3D: `samples/sample_character.cpp`, `samples/sample_ragdoll.cpp`,
  `samples/sample_joint.cpp` (higher-level reference implementations).
- Box3D: `CMakeLists.txt`, `src/CMakeLists.txt` (embedding and build requirements).
