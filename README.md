A prop of a crowbar with original Half-Life sound effects.

# Software
## Setup
### Assets
You'll need to have a copy of Half-Life to acquire the sound effect files.

Assuming your Half-Life copy is installed in Steam,
navigate to `Steam/steamapps/common/Half-Life/valve/sound` and copy these folders into this project's `./sfx/` folder.
Not all folders need to be copied, but it's best to at least copy barney, scientist, and weapons.

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
