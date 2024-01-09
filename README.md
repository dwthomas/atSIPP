# Interval generation


## Install

Install `meson`, `boost`, `zlib` and `nlohmann/json`

On arch linux: 
```
yay -S meson boost-libs nlohmann-json
```

On MacOS:
```
brew install meson boost zlib nlohmann-json
```

## Build

Setup build dirs:
```
meson setup --buildtype release  build
meson setup --buildtype debug  build_debug
meson setup --buildtype debugoptimized  build_debugoptimized
```

build:
```
./build.sh
```


## Run
First, create the @SIPP search graph:
```
cd ../python_generation
python3 create_intervals.py -s enkhuizen/scenario_small.json -l enkhuizen/location_enkhuizen.json -o output
```

Run:
```
./build_debug/atsipp --start t-401A --goal t-407B --edgegraph ../python_generation/output --search asipp --startTime 200
```