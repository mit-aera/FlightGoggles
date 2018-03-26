# Reducing FlightGoggles VRAM Usage

FlightGoggles normally requires `~1.5GB` of VRAM to function. However, for graphics cards with low available VRAM, an (unsupported) alternative exists:

```bash
./<FLIGHTGOGGLES_SIM_BINARY> <CONNECTION_ARGS> -screen-quality Fastest
```

This reduces FlightGoggles' VRAM usage to `~450MB`, but at the cost of lighting accuracy and photorealism.

