# Kisak-Strike
Kisak-Strike: Gentoo Offensive(KSGO) is a CSGO port aimed towards Linux enthusiasts. 

It can be built 100% Open-Source with optional Closed-source components.

Want to learn more about the History of Kisak-Strike? See this accompanying blog post: https://lwss.github.io/Kisak-Strike/
# Current Building Steps

```
Attention!

Kisak-Strike will build into folder ../game/
Make sure you clone this repo inside an existing folder!
```

Kisak-Strike uses CMake, the following sections will provide more information.

If you want to build with VPC for an authentic experience(not recommended), see https://gist.github.com/LWSS/9d2cd3205d197545d64fd27ee897fd53 for a rough draft from around when this project started.

## Packages
SDL2 SDL2_mixer tcmalloc_minimal rt openal curl ssl z crypto dl pthread fontconfig freetype GL

#### Ubuntu 
```
sudo apt install git build-essential cmake libsdl2-mixer-dev libsdl2-dev libgoogle-perftools-dev libopenal-dev libcurlpp-dev libssl-dev libfontconfig1-dev libcurl4-openssl-dev net-tools
```
#### Fedora
```
sudo dnf install git SDL2-devel SDL2_mixer-devel gperftools-devel openal-soft-devel libcurl-devel openssl-devel fontconfig-devel freetype-devel cmake gcc g++ mesa-libGL-devel mesa-libGLU-devel
```
#### Arch
```
sdl2 sdl2_mixer gperftools openal libcurl-compat openssl fontconfig freetype2 mesa cmake gcc base-devel
```

#### Gentoo
```
media-libs/libsdl2 media-libs/sdl2-mixer dev-util/google-perftools media-libs/openal net-misc/curl dev-libs/openssl media-libs/fontconfig media-libs/freetype media-libs/mesa dev-util/cmake sys-devel/gcc
```

## BUILD - cmake/make
See the wiki page for building options: https://github.com/SwagSoftware/Kisak-Strike/wiki/CMake

 #### Basic Usage
```
cd ./cmake-build
cmake .. <VARIOUS OPTIONS HERE>
make -j<NUM_THREADS>
```
## POSTBUILD - Acquire Original Game Files
Use Depot Downloader( https://github.com/SteamRE/DepotDownloader ) with your steam account. The depotdownloader built-into Steam was broken earlier this year and this is the only option currently.
```
CSGO SteamAppID: 730
CSGO Assets: DepotID: 731 ManifestID: 7043469183016184477
Windows Binaries: DepotID: 732 4047004309608881181
Linux Binaries: Depot ID: 734 4197642562793798650
```

* Copy over all files from the 731 assets depot (manifest: 7043469183016184477)
* Copy over *only needed* files from the 734 linux binary depot (manifest: 4197642562793798650)
    * ./bin/map_publish/* - (FOLDER which seems to contain some vgui assets)
    * ./csgo.sh
    * [OPTIONAL]./game/bin/linux64/libphonon3d.so -- If you want HRTF 3D sound
    * [OPTIONAL]./game/bin/linux64/vphysics_client.so -- If you want Valve-Original physics engine. (It runs a bit better than the rebuild, but is closed-source)
    * [OPTIONAL]./game/bin/linux64/scaleformui_client.so -- If you want the ScaleformUI for some reason.

## POSTBUILD - Acquire Additional Kisak-Strike Files
Head over to the Files Repo: https://github.com/SwagSoftware/Kisak-Strike-Files
These belong inside of your `../game/` folder.


## Current Nonfree blobs
* ${LIBPUBLIC}/libsteam_api.so  - Left for Convenience, can be removed in the future.

## Launch
`./csgo_linux64`




## License(s)
##### Any contributions made to Kisak-Strike will be considered donations to the public domain.

##### The following Inherited License from Source SDK also applies.

SOURCE 1 SDK LICENSE

Source SDK Copyright(c) Valve Corp.  

THIS DOCUMENT DESCRIBES A CONTRACT BETWEEN YOU AND VALVE 
CORPORATION ("Valve").  PLEASE READ IT BEFORE DOWNLOADING OR USING 
THE SOURCE ENGINE SDK ("SDK"). BY DOWNLOADING AND/OR USING THE 
SOURCE ENGINE SDK YOU ACCEPT THIS LICENSE. IF YOU DO NOT AGREE TO 
THE TERMS OF THIS LICENSE PLEASE DON’T DOWNLOAD OR USE THE SDK.  

  You may, free of charge, download and use the SDK to develop a modified Valve game 
running on the Source engine.  You may distribute your modified Valve game in source and 
object code form, but only for free. Terms of use for Valve games are found in the Steam 
Subscriber Agreement located here: http://store.steampowered.com/subscriber_agreement/ 

  You may copy, modify, and distribute the SDK and any modifications you make to the 
SDK in source and object code form, but only for free.  Any distribution of this SDK must 
include this LICENSE file and thirdpartylegalnotices.txt.  
 
  Any distribution of the SDK or a substantial portion of the SDK must include the above 
copyright notice and the following: 

    DISCLAIMER OF WARRANTIES.  THE SOURCE SDK AND ANY 
    OTHER MATERIAL DOWNLOADED BY LICENSEE IS PROVIDED 
    "AS IS".  VALVE AND ITS SUPPLIERS DISCLAIM ALL 
    WARRANTIES WITH RESPECT TO THE SDK, EITHER EXPRESS 
    OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, IMPLIED 
    WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT, 
    TITLE AND FITNESS FOR A PARTICULAR PURPOSE.  

    LIMITATION OF LIABILITY.  IN NO EVENT SHALL VALVE OR 
    ITS SUPPLIERS BE LIABLE FOR ANY SPECIAL, INCIDENTAL, 
    INDIRECT, OR CONSEQUENTIAL DAMAGES WHATSOEVER 
    (INCLUDING, WITHOUT LIMITATION, DAMAGES FOR LOSS OF 
    BUSINESS PROFITS, BUSINESS INTERRUPTION, LOSS OF 
    BUSINESS INFORMATION, OR ANY OTHER PECUNIARY LOSS) 
    ARISING OUT OF THE USE OF OR INABILITY TO USE THE 
    ENGINE AND/OR THE SDK, EVEN IF VALVE HAS BEEN 
    ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.  
 
       
If you would like to use the SDK for a commercial purpose, please contact Valve at 
sourceengine@valvesoftware.com.

