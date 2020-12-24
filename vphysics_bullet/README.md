# source-sdk-bullet-physics
Experimental Bullet Physics Injection for Source SDK 2013.

![image](https://i.ibb.co/nkcRSLg/download.gif)

## About
The repository is based on https://github.com/DrChat/Gmod-vphysics. It's not forked because of that one having whole Bullet SDK pushed into same repository, to keep my repository crisp & clear. Major changes on this implementation are:

- Updated version of Bullet SDK. The base repository had an ancient version of Bullet with lots of deprecated stuff being used.
- Utilization of new multithreaded modules of Bullet SDK
- Lots of performance improvements

Demonstration recordings can be found on:
- https://www.youtube.com/watch?v=6CH1BSyM92A
- https://www.youtube.com/watch?v=kMz_1qonMqs

## Building Vphysics.dll for Source SDK 2013
- Checkout submodules inside thirdparty folder
- Follow the instructions on bullet3 repository to build Bullet libs & binaries
- Open vphysics solution and build the project
- Place generated vphysics.dll binary into desired Source SDK 2013 based game (Half-Life 2, GMod etc.), or into your custom built Source SDK 2013 game.

## Known Issues
- Save/Load functionality doesn't work, and mostly crashes the game. You should disable physics restore functionality on save/load module of Source SDK 2013 to fix this issue.
- Small objects with very high speed (Thrown grenades for example) may pass through landscape mesh. Also, big objects with very high speed may have a tunnelling effect while colliding with landscape meshes. That's mostly an issue with Bullet's messed up convex mesh collision algorithm, and it's not likely to be solved because of core part of the physics engine being abandoned on development.
- Impact damage is not implemented. Because of Bullet's lack of a proper callback system for hit/impact/collision events, it requires huge amount of effort while using multithreaded modules of Bullet.
- Physics impact sounds are broken, again, it will probably require huge amount of effort to be fixed.
- The implementation is highly experimental, it's not recommended to use it with production purposes.

## License
```
MIT License

Copyright (c) 2020 Doğa Can Yanıkoğlu

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
## Contribution
Source code is licensed under MIT license, and other developers are encouraged to fork the repository, open issues & pull requests to help the development.
