SRC=src
OUT=out
FIG=fig
REF=ref

SRCS=$(wildcard $(SRC)/*.tex)

target=main

all: $(OUT)/$(target).pdf

.PHONE: edit pdf view

view: $(OUT)/$(target).pdf
	xdg-open $? 2> /dev/null &

$(OUT)/%.pdf: $(SRC)/%.tex
	@mkdir -p $(OUT) 
	@pdflatex --output-directory=./out --file-line-error $< 2> /dev/null 

edit: $(SRCS)
	vim -p$(shell echo "$?" | wc -w) $(SRCS);

clean:
	rm -r $(OUT)
