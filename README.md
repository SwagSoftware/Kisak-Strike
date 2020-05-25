# Kisak-Strike

*Note: builds game in folder `../game`, make sure to clone inside another folder.*

```
cd cmake-build
cmake ..
make -j123
```

instructions not complete, https://gist.github.com/LWSS/9d2cd3205d197545d64fd27ee897fd53 for a rough draft

# Current Building Steps
## Packages
* google perftools (tcmalloc)
* SDL2
* openal
## PREBUILD - cryptopp
```
cd ./external/crypto*
ISX64=1 IS_SUN_CC=0 make libcryptopp.a -j3 #Ignore the error at the end.
mkdir -p ../../lib/linux64/release
cp ./libcryptopp.a ../../lib/linux64/release
```

## BUILD - cmake/make
```
cd ./cmake-build
cmake ..
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
    * ./game/bin/linux64/libsteam_api.so
    * ./game/bin/linux64/libtogl_client.so #this is on github, we can probably build this
    * ./game/bin/linux64/libphonon3d.so #proprietary HRTF 3d audio system.
    * ./game/bin/linux64/vphysics_client.so #proprietary Havok physics system.
    * ./game/bin/linux64/scaleformui_client.so #proprietary Scaleform flash UI, this is needed by main menu
    * ./game/bin/linux64/shaderapidx9_client.so 

Each one of these proprietary files is an enemy to our freedoms, over time we aim to remove these.


## Launch
`./csgo_linux64`
    
