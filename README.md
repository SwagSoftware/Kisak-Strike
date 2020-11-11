# Kisak-Strike
Kisak-Strike: Gentoo Offensive is a CSGO port aimed towards Linux enthusiasts. 

Our goal is to accomplish a fully open-source CSGO, something good enough to be "gentoo-approved".

# Current Building Steps

**Note: builds game in folder `../game`, make sure to clone the repo into a folder first!**

Kisak-Strike uses CMake, the following sections will provide more information.

If you want to build with VPC for an authentic experience(not recommended), see https://gist.github.com/LWSS/9d2cd3205d197545d64fd27ee897fd53 for a rough draft from around when this project started.

## Packages
SDL2 SDL2_mixer tcmalloc_minimal rt openal curl ssl z crypto dl pthread fontconfig freetype GL

#### blobs
* steamdatagramlib_client.a
* libsteam_api.so
* gcsdk_client.a

#### Ubuntu 
```
sudo apt install git build-essential cmake libsdl2-mixer-dev libsdl2-dev libgoogle-perftools-dev libopenal-dev libcurlpp-dev libssl-dev libfontconfig1-dev libcurl4-openssl-dev net-tools
```
#### Fedora
```
TODO
```
#### Arch
```
TODO
```
#### Gentoo
```
TODO
```

## BUILD - cmake/make
* -DUSE_KISAK_PHYSICS=1
    * Use the open source Physics rebuild instead of a blob from the 734 Binary repo
* -DUSE_ROCKETUI=1
    * Use Custom RocketUI. Without this, the UI will be a broken mess of VGUI. (Scaleform is disabled by default use -DUSE_SCALEFORM=1 if you really want)
* -DDEDICATED=1
    * Change the build to dedicated server mode. Note that the builds are not in-tree compatible, some things will have to be rebuilt. Make sure to use -DDEDICATED=0 once you want to go back to the client build.
* -DUSE_VALVE_HRTF=1
    * By default the HRTF is disabled because it requires a proprietary blob(libphonon3d.so), set this flag to re-enable it.
```
cd ./cmake-build
cmake .. <VARIOUS OPTIONS HERE>
make -j<NUM_THREADS>
```
## POSTBUILD - Acquire needed extras
Use Depot Downloader( https://github.com/SteamRE/DepotDownloader ) with your steam account
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

## Current Nonfree blobs
* ${LIBPUBLIC}/gcsdk_client.a
* ${LIBPUBLIC}/libsteam_api.so
* ${LIBPUBLIC}/steamdatagramlib_client.a

## Launch
`./csgo_linux64`
    
