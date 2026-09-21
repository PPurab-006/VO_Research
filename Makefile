.PHONY: all env test analysis figures rerun-vo

all: analysis figures test

env:
	pip install -r requirements.txt

test:
	python3 -m unittest discover -s tests

analysis:
	python3 src/analysis/gt_heading_audit.py
	python3 src/analysis/threshold_sweep.py

figures:
	python3 figures/make_all.py
	python3 figures/make_captions.py

rerun-vo:
	@echo "Rerunning offline VO pipeline across datasets..."
	python3 src/pipelines/run_phase3_full_eval.py
	python3 src/analysis/threshold_sweep.py --rerun

