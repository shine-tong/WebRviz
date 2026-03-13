SPHINXBUILD ?= sphinx-build
SPHINXOPTS ?= -W --keep-going

.PHONY: html-zh html-en html-all clean

html-zh:
	DOC_LANG=zh $(SPHINXBUILD) -b html -c . zh _build/html-zh $(SPHINXOPTS)

html-en:
	DOC_LANG=en $(SPHINXBUILD) -b html -c . en _build/html-en $(SPHINXOPTS)

html-all: html-zh html-en
	rm -rf _build/site
	mkdir -p _build/site/en
	cp -a _build/html-zh/. _build/site/
	cp -a _build/html-en/. _build/site/en/
	touch _build/site/.nojekyll

clean:
	rm -rf _build
