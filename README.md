# LOAS - Low Orbit Attitude Simulator (WIP)
WIP Satellite attitude simulator, equipped with reaction wheels and magnetic torquers

[Full documentation here](https://loas.feg.ovh/)

## Installation

### Virtual Environement install (recommended)

A bash script is provided to build the proper virtual environment for loas:

```bash
cd LOAS && ./install_venv
```

Then activate the virtual environment to use loas:

```bash
$ source venv/bin/activate
$ python3
Python 3.7.3 (default, Jul 25 2020, 13:03:44)
[GCC 8.3.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import loas
>>>
```

### Global install (easy way, requires sudo access)

- **System libraries:**
```bash
sudo apt install libspatialindex-dev
```

- **Python libraries:**
```bash
pip3 install -r requirements.txt
```

loas becomes accessible from the folder `LOAS`:

```bash
$ cd LOAS
$ python3
Python 3.7.3 (default, Jul 25 2020, 13:03:44)
[GCC 8.3.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import loas
>>>
```
