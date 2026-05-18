# GAASD CARLA P0 Smoke Project

This minimal project is used to verify that GAASD `run_simulation.sh` can link
and execute `libcarla_gaasd_adapter.so` from `objectCode/total/DLL`.

It is not a GAASD canvas export. It only contains the minimum files required by
the current runner:

- `FuncStep.c`
- `icvos/blocks/functions/dictionaryData/struct.json`
- `objectCode/total/DLL/libcarla_gaasd_adapter.so` generated during testing
