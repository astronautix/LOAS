# LOAS - Low Orbit Attitude Simulator (WIP)
WIP Satellite attitude simulator, equipped with reaction wheels and magnetic torquers

[Full documentation here](https://loas.feg.ovh/)

## Requirements
If there are missing libraries, please open an issue!

- **System libraries:**
```bash
sudo apt install libspatialindex-dev
```

- **Python libraries:**
```bash
pip3 install -r requirements.txt
```


## Usage
```bash
# examples/test files
cd examples && python3 viewer.py
```

## Known Issues
### `pyglet.gl.lib.GLException: b'invalid operation'`
Fix:
```bash
export PYGLET_GRAPHICS_VBO=0
```
