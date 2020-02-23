PROJ=top
TRELLIS=/usr/share/trellis

all: ${PROJ}.bit

%.json: %.v
	yosys -p "synth_ecp5 -json $@" $<

%_out.config: %.json
	nextpnr-ecp5 --json $< --textcfg $@ --25k --package CABGA381 --lpf top.lpf

%.bit: %_out.config
	ecppack --svf ${PROJ}.svf $< $@

${PROJ}.svf : ${PROJ}.bit

clean:
	rm -f *.svf *.bit *.config *.json

.PHONY: prog clean
