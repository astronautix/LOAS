install:
	sudo apt install -y libcgal-dev libeigen3-dev
	python3 -m pip install -r requirements.txt
	cp src/conf.default.py src/conf.py

run:
	cd src && python3 main.py
