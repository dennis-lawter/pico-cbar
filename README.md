A prop of a crowbar with original Half-Life sound effects.

# Software
## Setup
### Assets
You'll need to have a copy of Half-Life to acquire the sound effect files.

Assuming your Half-Life copy is installed in Steam,
navigate to `Steam/steamapps/common/Half-Life/valve/sound/weapons` and copy these files into this project's `./sfx/` folder:
- cbar_hit1.wav
- cbar_hit2.wav
- cbar_hitbod1.wav
- cbar_hitbod2.wav
- cbar_hitbod3.wav
- cbar_miss1.wav

### Build setup
You may need to update your toolchain to support Raspberry Pico development.
```bash
make init
```

## Uploading to a pico
Plug the pico in and run:
```bash
make run
```

# Electronics
