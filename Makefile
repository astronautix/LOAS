install:
	python3 -m pip install -r requirements.txt
	cp src/conf.default.py src/conf.py

run:
	python3 src/main.py
