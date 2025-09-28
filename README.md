# Raylib-CPP-Starter-Template-for-VSCODE-V2
Raylib C++ Starter Template for Visual Studio Code on Windows.
This demo project contains a bouncing ball raylib example program.
It works with raylib version 5.0. Tested on both Windows 10 and Windows 11.

# How to use this template
1. Double click on the main.code-workspace file. This will open the template in VS Code.
2. From the Explorer Window of VS Code navigate to the src folder and double click on the main.cpp file.
3. Press F5 on the keyboard to compile and run the program.

## Building with CMake

The project now ships with a cross-platform CMake build in addition to the original Makefile. You need the raylib SDK (headers and static library). On Windows the default installation path is `C:/raylib/raylib`.

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DRAYLIB_ROOT=C:/raylib/raylib
cmake --build build --config Debug
```

The `RAYLIB_ROOT` cache variable should point at the folder containing `src/raylib.h` and `src/libraylib.a`. Once the CMake cache is generated you can open the `build` folder in VS Code or your IDE of choice.

# What's changed
The template now uses folders for better organizion of the files. So, all the source code now lives in the src folder.

# Video Tutorial

<p align="center">
  <img src="preview.jpg" alt="" width="800">
</p>

<p align="center">
🎥 <a href="https://www.youtube.com/watch?v=PaAcVk5jUd8">Video Tutorial on YouTube</a>
</p>

<br>
<br>
<p align="center">
| 📺 <a href="https://www.youtube.com/channel/UC3ivOTE5EgpmF2DHLBmWIWg">My YouTube Channel</a>
| 🌍 <a href="http://www.programmingwithnick.com">My Website</a> | <br>
</p>
