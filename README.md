<img align="left" src="Content/logo_small.png" width="180px"/>

# Wicked Engine

[![Github Build Status](https://github.com/turanszkij/WickedEngine/workflows/Build/badge.svg)](https://github.com/turanszkij/WickedEngine/actions)
[![Discord chat](https://img.shields.io/discord/602811659224088577?logo=discord)](https://discord.gg/CFjRYmE)
<a href="https://twitter.com/intent/follow?screen_name=turanszkij"><img src="https://img.shields.io/twitter/follow/turanszkij.svg?style=social" alt="follow on Twitter"></a>
<br/>
[![Steam](https://img.shields.io/badge/Steam-%23000000.svg?logo=steam&logoColor=white)](https://store.steampowered.com/app/1967460/Wicked_Engine/)
[![Itch.io](https://img.shields.io/badge/itch.io-%23FF0B34.svg?logo=Itch.io&logoColor=white)](https://turanszkij.itch.io/wicked-engine)

<br/>
<img align="right" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/girl_pose.png" width="240px"/>
Wicked Engine is an open-source 3D engine with modern graphics. Use this as a C++ framework for your graphics projects, a standalone 3D editor, LUA scripting or just for learning.<br/>

- [Website](https://wickedengine.net/)<br/>
- [Features](features.txt)<br/>
- [Editor Manual](Content/Documentation/WickedEditor-Manual.pdf)<br/>
- [C++ Documentation](Content/Documentation/WickedEngine-Documentation.md)<br/>
- [Lua Documentation](Content/Documentation/ScriptingAPI-Documentation.md)<br/>
- [Videos](https://www.youtube.com/playlist?list=PLLN-1FTGyLU_HJoC5zx6hJkB3D2XLiaxS)<br/>

You can get the full source code by using Git version control and cloning https://github.com/turanszkij/WickedEngine.git, or downloading it as zip. You can also download nightly packaged builds of the Editor here (requires Github sign in): [![Github Build Status](https://github.com/turanszkij/WickedEngine/workflows/Build/badge.svg)](https://github.com/turanszkij/WickedEngine/actions)

<br/>

<img align="left" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/guy_pose.png" width="240px"/>

### Platforms:
- Windows
- Linux
- Mac OS
- iOS
- Xbox Series X|S
- PlayStation 5

### Native CSG authoring

The editor's left-side **CSG** options panel builds ordered additive/subtractive
cube and polygonal-cone brushes from an initially empty result. Create a preview,
choose its operation, commit, then explicitly **Rebuild CSG**. Source edits keep
the last generated result visible until a successful rebuild.

Choose Surface/material, Brush move, Vertex edit or Face drag mode. Source
outlines and handles use the active perspective/orthographic pane. **Ctrl** snaps
drags, **Shift-click** toggles vertex groups, **Alt-click** repicks a brush,
**Esc** cancels a drag and **F** frames the source selection. Orthographic vertex
picking groups coincident projections. Numeric movement, uniform scale and
axis/angle rotation are also available. Source brushes remain closed,
planar-faced convex solids. A vertex move that warps a face splits it into planar
pieces, which keep the original face's UV mapping and material; moves that would
make the brush concave or flat, and topology-changing face drags, are rejected.

`wiCSG` is a bounded, deterministic CPU geometry kernel. Native scenes register
`CSGComponent` in every Scene, retaining source IDs and material bindings through
serialization/remapping and duplication. Rebuild stages ordinary per-surface
meshes before replacing generated objects. Native materials/UV edits and unrelated
entities survive; generated lightmaps must be recreated after rebuilding.
Use the source tools instead of derived-object transform controls. Undo/redo
restores source and result together.

The focused CMake test targets are `CSGTest`, `CSGSceneTest` and
`EditorViewportTest`; they do not require a graphics device. Native scenes remain
trusted local files, and the internal Boolean BSP is not a runtime level format.

### How to build: 

<img align="right" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/videoprojectors.gif" width="320px"/>

#### Windows
To build Wicked Engine for Windows, open the `WickedEngine.sln` solution file with the latest Visual Studio. When it's opened, press F5, then Wicked Engine and the Editor application will be built and then start. You can check out other sample projects in the solution too.

If you want to develop a C++ application that uses Wicked Engine, you can build the WickedEngine_Windows static library project and link against it. Including the `"WickedEngine.h"` header will attempt to link the binaries for the appropriate platform, but search directories should be set up beforehand. For example, you can set additional library directories to `$(SolutionDir)BUILD\$(Platform)\$(Configuration)` by default. For examples, see the `Samples/Template_Windows`, `Samples/Tests`, and `Editor_Windows` projects. 

You can also use cmake to build on windows with these commands:
```
cmake -B build
cmake --build build --config Release
```

<img align="right" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/fps.gif" width="320px"/>

#### Linux
To build the engine for Linux, use Cmake. You can find a sample build script for Ubuntu [here](.github/workflows/build.yml) (in the linux section). 

On Linux you will need to ensure some additional dependencies are installed, such as Cmake (3.7 or newer), g++ compiler (C++ 17 compliant version) and SDL2. You can use the following commands to install dependencies:

<img align="right" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/fighting_game.gif" width="320px"/>

```bash
sudo apt update
sudo apt install libsdl2-dev
sudo apt install build-essential
```

Example commands to build the engine with cmake in Release mode:
```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

<img align="right" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/character_grass.gif" width="320px"/>

#### Physics backend (CMake)

Jolt remains the default. An experimental Box3D rigid-body backend can be selected
at build time; exactly one backend is compiled and linked:

```bash
cmake -S . -B build-box3d -DWICKED_PHYSICS_BACKEND=Box3D -DWICKED_USE_SDL3=ON
cmake --build build-box3d --parallel
ctest --test-dir build-box3d --output-on-failure
```

Use `-DWICKED_PHYSICS_BACKEND=Jolt` (or omit the option) for Jolt. Box3D requires
CMake 3.22 or newer and is vendored, so no separate download is needed. SDL3 is
independent of the physics selection.

The Box3D adapter supports rigid bodies, primitive/convex/static mesh/heightfield
shapes, common constraints, ray queries and rigid-body picking. Character and
vehicle controllers, ragdolls, soft bodies, buoyancy and general SixDOF constraints
are not supported. Unsupported scene data is retained and diagnosed rather than
silently converted. There is no runtime switching or Jolt fallback in a Box3D
build. Native Visual Studio/Xcode project files remain Jolt-only; use CMake for
the backend selector.

Mesh/heightfield collision is static-only, cylinders use 32-sided hulls, and
mirrored/sheared geometry is rejected. SwingTwist is limited to circular
equal-angle configurations; cone half angles must not exceed pi/2. Interactive
Editor acceptance and installed-package consumer support remain unverified.
Flat convex hulls use a small, symmetric collision thickness (normally 0.002
engine units), reported in the log, to preserve kinematic floors/platforms.
The Box3D Physics Test is rigid-body-only: its crumpling torus and the hidden
Cloth Physics Test require the unimplemented soft-body solver.

See [WickedEngineBox3D.md](WickedEngineBox3D.md) for the design, feature restrictions,
verification coverage and remaining work.

#### Mac OS and iOS
To build  the engine for Mac OS and iOS, use the provided .xcodeproj files with the Xcode development environment, for example:
- WickedEngine/WickedEngine.xcodeproj to build static library
- Editor/Editor.xcodeproj to build the Wicked Editor
- Samples/Template_MacOS/Template_MacOS.xcodeproj to build a minimal template application for MacOS that you can use as a base for your application
- Samples/Template_iOS/Template_iOS.xcodeproj to build a minimal template application for iOS that you can use as a base for your application

#### Xbox Series X|S and PlayStation 5
Console specific extension files required for building are currently private.


### Examples:

#### Initialization (C++):

<img align="right" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/clouds.gif" width="320px"/>

```cpp
// Include engine headers:
#include "WickedEngine.h"

// Create the Wicked Engine application:
wi::Application application;

// Assign window that you will render to:
application.SetWindow(hWnd);

// Run the application:
while(true) {
   application.Run(); 
}
```

#### Basics (C++):
```cpp
application.Initialize(); // application will start initializing at this point (asynchronously). If you start calling engine functionality immediately before application.Run() gets called, then you must first initialize the application yourself.

wi::RenderPath3D myGame; // Declare a game screen component, aka "RenderPath" (you could also override its Update(), Render() etc. functions). 
application.ActivatePath(&myGame); // Register your game to the application. It will call Start() once, then Update(), Render(), etc. from now on every frame...

wi::scene::LoadModel("myModel.wiscene"); // Simply load a model into the current global scene
wi::scene::GetScene(); // Get the current global scene

wi::scene::Scene scene2; // create a separate scene
wi::scene::LoadModel(scene2, "myModel2.wiscene"); // Load model into a separate scene
wi::scene::GetScene().Merge(scene2); // Combine separate scene with global scene

myGame.setFXAAEnabled(true); // You can enable post process effects this way...

wi::RenderPath2D myMenuScreen; // This is an other render path, but now a simple 2D one. It can only render 2D graphics by default (like a menu for example)
application.ActivatePath(&myMenuScreen, 0.8f); // activate the menu after a 0.8 second fade out, the previous path (myGame) will be stopped

wi::Sprite mySprite("image.png"); // There are many utilities, such as a "sprite" helper class
myMenuScreen.AddSprite(&mySprite); // The 2D render path is ready to handle sprite and font rendering for you

wi::audio::Sound mySound;
wi::audio::CreateSound("explosion.wav", &mySound); // Loads a sound file
wi::audio::SoundInstance mySoundInstance;
wi::audio::CreateSoundInstance(&mySound, &mySoundInstance); // Instances the sound file, it can be played now
wi::audio::Play(&mySoundInstance); // Play the sound instance
wi::audio::SetVolume(0.6, &mySoundInstance); // Set the volume of this soundinstance
wi::audio::SetVolume(0.2); // Set the master volume

if (wi::input::Press(wi::input::KEYBOARD_BUTTON_SPACE)) { wi::audio::Stop(&mySoundInstance); } // You can check if a button is pressed or not (this only triggers once)
if (wi::input::Down(wi::input::KEYBOARD_BUTTON_SPACE)) { wi::audio::Play(&mySoundInstance); } // You can check if a button is pushed down or not (this triggers repeatedly)
```

#### Scripting (LUA):
```lua
-- Set a rendering path for the application
path = RenderPath3D;
application.SetActivePath(path);    -- "application" is created automatically by wi::Application

-- Load a model entity into the global scene:
entity = LoadModel("myModel.wiscene");

-- Load a model entity into a separate scene:
scene2 = Scene()
entity2 = LoadModel(scene2, "myModel2.wiscene");

-- Combine the separate scene with the global scene:
scene.Merge(scene2);

-- Get the current global scene:
scene = GetScene();

-- Move model to the right using the entity-component system:
transform = scene.Component_GetTransform(entity);
transform.Translate(Vector(2, 0, 0));

-- Print any WickedEngine class information to the backlog:
getprops(application);	-- prints the application methods
getprops(scene);	-- prints the Scene class methods
getprops(path);	-- prints the deferred render path methods

-- Play a sound:
sound = Sound()
audio.CreateSound("explosion.wav", sound)
soundinstance = SoundInstance()
audio.CreateSoundInstance(sound, soundinstance)  -- several instances can be created from one file
audio.Play(soundinstance)
audio.SetVolume(0.6, soundinstance)  -- sets the volume of this soundinstance
audio.SetVolume(0.2)  -- sets the master volume

-- Check for input:
if(input.Press(KEYBOARD_BUTTON_LEFT)) then
   audio.Play(soundinstance); -- this will play the sound if you press the left arrow on the keyboard
end
```

<img align="right" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/inverse_kinematics.gif" width="320px"/>

<i>(You can enter lua scripts into the backlog (HOME button), or the startup.lua script which is always executed on application startup if it is found near the app, or load a script via dofile("script.lua") command)</i>

For more code samples and advanced use cases, please see the example projects, like the Template_Windows, Tests, or Editor project. There are also sample models and scripts included with Wicked Engine in the Content/models and Content/scripts folders. Check them out to learn about more features.

### Scripting API:

<img align="right" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/grass.gif" width="320px"/>

You can use a lot of of engine features through Lua scripting. The included applications, like the Editor,
contain a scripting input method toggled by the "Home" key. A text screen will be presented where the user can type in LUA commands.
For further details, check the scripting API documentation: [Wicked Engine Scripting API](Content/Documentation/ScriptingAPI-Documentation.md)

### Model import/export:
The native model format is the <b>WISCENE</b> format. Any application using Wicked Engine can open this format efficiently.

<img align="right" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/character_lookat.gif" width="320px"/>

In addition, the Editor supports importing some common model formats: 
- <b>OBJ</b>
- <b>FBX</b>
- <b>GLTF, GLB</b>
- <b>VRM, VRMA</b>
- <b>PLY</b>

You can import models into the Editor, and save them as <b>WISCENE</b>, then any Wicked Engine application can open them.<br/>

<img align="right" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/snowstorm.gif" width="320px"/>

### Graphics API:
The default renderer is `DirectX 12` on Windows, `Vulkan` on Linux and `Metal` on Apple systems.
You can specify command line arguments (without any prefix) to switch between render devices or other settings. Currently the list of options:
<table>
  <tr>
	<th>Argument</th>
	<th>Description</th>
  </tr>
  <tr>
	<td>vulkan</td>
	<td>Use the Vulkan rendering device on Windows</td>
  </tr>
  <tr>
	<td>debugdevice</td>
	<td>Use debug layer for graphics API validation. Performance will be degraded, but graphics warnings and errors will be written to the "Output" window</td>
  </tr>
  <tr>
	<td>gpuvalidation</td>
	<td>Use GPU Based Validation for graphics. This must be used together with the debugdevice argument. Currently DX12 only.</td>
  </tr>
  <tr>
	<td>gpu_verbose</td>
	<td>Enable verbose GPU validation mode.</td>
  </tr>
  <tr>
	<td>igpu</td>
	<td>Prefer integrated GPU selection for graphics. By default, dedicated GPU selection will be preferred.</td>
  </tr>
  <tr>
	<td>amdgpu</td>
	<td>Prefer AMD GPU selection for graphics.</td>
  </tr>
  <tr>
	<td>nvidiagpu</td>
	<td>Prefer Nvidia GPU selection for graphics.</td>
  </tr>
  <tr>
	<td>intelgpu</td>
	<td>Prefer Intel GPU selection for graphics.</td>
  </tr>
  <tr>
	<td>alwaysactive</td>
	<td>The application will not be paused when the window is in the background.</td>
  </tr>
</table>


<img align="right" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/talking.gif" width="320px"/>
<br/>

### Other software using Wicked Engine
- <a href="https://turanszkij.itch.io/wicked-shooter">Wicked Shooter</a>: FPS sample game in Wicked Engine
- <a href="https://turanszkij.itch.io/wicked-adventure">Wicked Adventure</a>: Adventure RPG sample game in Wicked Engine
- <a href="https://turanszkij.itch.io/grass-zen">Grass Zen</a>: A relaxing game made with Wicked Engine where you control the wind
- <a href="https://www.game-guru.com/max">Game Guru MAX</a>: Easy to use game creator
- <a href="https://www.youtube.com/watch?v=0SxXmnSQ6Q4">Flytrap</a>: Demoscene production by qop
- <a href="https://youtu.be/mbmNU5QVM8A?si=9sDMS1LrMsz03f5r">doddering</a>: Demoscene production by qop
- Your project: add your project to this readme and open a pull request

<br/>

### Troubleshooting
If you are having trouble getting the applications to run, make sure that you satisfy the following conditions:
- If you built the application with Visual Studio, run it from the Visual Studio environment, where the executable working directory is set up to be the Project directory (not the build directory where the exe will be found)
- If you want to run an application without Visual Studio, either copy the executable from the BUILD directory to the correct project directory, or set the working directory appropriately. You can also check the Working directory setting in Visual Studio to find out the right working directory of every project. 

<img align="right" src="https://github.com/turanszkij/wickedengine-gifs/raw/main/weather.gif" width="320px"/>

- If you experience crashes, you can try these to find out the problem:
	- make sure your environment is up to date, with latest graphics drivers and operating system updates.
	- see if there is a log.txt in the working directory of the application (most likely near the application exe)
	- request help on [Discord](https://discord.gg/CFjRYmE) or via a [Github issue](https://github.com/turanszkij/WickedEngine/issues)
	- build the engine in Debug mode and try to run it, see where it crashes
	- run the engine with the `debugdevice` command argument and post the text from your console output window when the crash happens
		- for very advanced users, using `gpuvalidation` with `debugdevice` will print additional graphics debug information
