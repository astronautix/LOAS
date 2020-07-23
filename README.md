# LOAS - Low Orbit Attitude Simulator (WIP)
WIP Satellite attitude simulator, equipped with reaction wheels and magnetic torquers

[Full documentation here](https://loas.feg.ovh/)

## Installation

If there are missing libraries, please open an issue!

### Global install (needs sudo access)

- **System libraries:**
```bash
sudo apt install libspatialindex-dev
```

- **Python libraries:**
```bash
pip3 install -r requirements.txt
```

### Virtual Environement install

```bash
cd LOAS
python3 -m venv venv
source venv/bin/activate
wget http://download.osgeo.org/libspatialindex/spatialindex-src-1.8.5.tar.gz
tar xzf spatialindex-src-1.8.5.tar.gz
cd spatialindex-src-1.8.5
./configure --prefix=$VIRTUAL_ENV
make
make install
cd ..
rm -rf spatialindex-src-1.8.5 spatialindex-src-1.8.5.tar.gz
echo 'export SPATIALINDEX_C_LIBRARY=$VIRTUAL_ENV/lib/libspatialindex_c.so' >> venv/bin/activate
source venv/bin/activate
pip install -r requirements.txt
```
